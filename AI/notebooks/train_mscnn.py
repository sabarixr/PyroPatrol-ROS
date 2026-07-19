"""
train_mscnn.py — Multi-Sector CNN with Cross-Sector Attention
==============================================================

Why this architecture exists
-----------------------------
Fire direction classification is fundamentally a COMPARATIVE task:
the robot must determine which of LEFT / FRONT / RIGHT has the highest
sensor response relative to the others.

The naive 1D CNN processes each sector independently and majority-votes
three separate predictions — it never sees the cross-sector contrast that
encodes fire direction.  XGBoost, by contrast, receives all three
directions' statistics in one feature vector and can directly learn
"if mq2_LEFT >> mq2_FRONT ≈ mq2_RIGHT → fire is LEFT".

The Multi-Sector CNN (MSCNN) resolves this by:

  1. Shared Conv1D encoder  — extracts a per-sector embedding from the
     raw 15-timestep sensor signal (intra-sector temporal features).
     Weights are shared across all three sectors, forcing the model to
     learn a sensor representation that generalises across directions.

  2. Cross-Sector Attention — a multi-head self-attention layer over the
     three sector embeddings.  Each sector attends to the others, letting
     the model learn "sector LEFT has high gas AND sector FRONT is quiet
     → fire is LEFT".  This is the architectural novelty that directly
     models the task structure.

  3. Residual + LayerNorm   — standard transformer-style stability.

  4. Concat → MLP head      — the three attended sector embeddings are
     concatenated and classified in a single forward pass, yielding one
     prediction per scan with no voting.

Input  : (batch, 3_sectors, 10_channels, 15_timesteps)
Output : tuple((batch, 4_classes), (batch, 1_severity))

Evaluation (matches all other notebooks for fair comparison)
-------------------------------------------------------------
Leave-one-SCAN-out CV — each scan is held out once.
Each sample is already one full scan, so there is no sequence vs. scan
distinction here: one prediction = one scan prediction.
Normalisation is fit on training scans only (no leakage).
Augmented scans whose scan_key matches the held-out scan are excluded
from training.

Saved artefacts (notebooks/ directory)
----------------------------------------
  mscnn_direction.pt   — final model weights
  mscnn_scaler.pkl     — StandardScaler (fit on all data)
  mscnn_config.pkl     — {n_channels, n_timesteps, n_classes}
  mscnn_classes.npy    — label class names
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import joblib
from pathlib import Path
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import (
    accuracy_score, f1_score,
    classification_report, confusion_matrix,
    mean_squared_error, mean_absolute_error,
)
from torch.utils.data import TensorDataset, DataLoader

HERE     = Path(__file__).parent
DATA_DIR = HERE.parent / "data"

# hyperparams, i think this will do this, ayy dont mess this upp
D_MODEL      = 128      # sector embedding dimension
N_HEADS      = 4        # attention heads (D_MODEL must be divisible)
EPOCHS       = 200
LR           = 5e-4
BATCH_SIZE   = 16
DROPOUT      = 0.3
WEIGHT_DECAY = 1e-4
SEED         = 42

torch.manual_seed(SEED)
np.random.seed(SEED)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# model architecture, pretty standard stuff
class SharedSectorEncoder(nn.Module):
    """Shared 1D CNN applied independently to each sector's raw time-series."""
    def __init__(self, n_channels: int, d_model: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv1d(n_channels, 32,      kernel_size=3, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),
            nn.Conv1d(32,        64,       kernel_size=3, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Conv1d(64,        d_model,  kernel_size=3, padding=1),
            nn.BatchNorm1d(d_model),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool1d(1),       # → (*, d_model, 1)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)     # (*, d_model)


class MultiSectorCNN(nn.Module):
    """
    Multi-Sector CNN with Cross-Sector Attention for fire direction detection.

    Input  : (B, 3, C, T)   — batch of full scans
    Output : (B, n_classes), (B,)  — classification and severity per scan
    """
    def __init__(self, n_channels: int, n_classes: int,
                 d_model: int = 128, n_heads: int = 4, dropout: float = 0.3):
        super().__init__()

        # 1. Shared encoder across all sectors
        self.encoder = SharedSectorEncoder(n_channels, d_model)

        # 2. Cross-sector multi-head self-attention
        #    "Which sector has the fire signal, relative to the others?"
        self.cross_attn = nn.MultiheadAttention(
            embed_dim=d_model, num_heads=n_heads,
            dropout=dropout, batch_first=True,
        )
        self.attn_norm = nn.LayerNorm(d_model)

        # 3. Shared features
        self.fc = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(d_model * 3, d_model),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout / 2)
        )
        # Dual heads
        self.head_dir = nn.Linear(d_model, n_classes)
        self.head_sev = nn.Sequential(
            nn.Linear(d_model, 1),
            nn.Sigmoid()
        )

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        B, S, C, T = x.shape           # S = 3 sectors

        # Encode each sector independently with the shared CNN
        x_flat  = x.view(B * S, C, T)              # (B*3, C, T)
        encoded = self.encoder(x_flat)              # (B*3, D)
        sectors = encoded.view(B, S, -1)            # (B, 3, D)

        # Cross-sector attention: sectors attend to each other
        attn_out, _ = self.cross_attn(sectors, sectors, sectors)
        sectors = self.attn_norm(sectors + attn_out)  # residual connection

        # Concatenate all three attended sector embeddings → classify
        combined = sectors.view(B, -1)              # (B, 3*D)
        feat = self.fc(combined)
        return self.head_dir(feat), self.head_sev(feat).squeeze(-1)


# gotta normalize the data first
def fit_normalize(
    X_train_real: np.ndarray,
    X_train_aug:  np.ndarray,
    X_test:       np.ndarray,
):
    """
    Fit StandardScaler on real training scans, transform all splits.
    X shape: (N, 3, C, T).  Flattens to (N*3*T, C) for fitting.
    """
    N_tr, S, C, T = X_train_real.shape

    def flat(X: np.ndarray) -> np.ndarray:
        return X.transpose(0, 1, 3, 2).reshape(-1, C)  # (N*S*T, C)

    def unflat(Xf: np.ndarray, N: int) -> np.ndarray:
        return Xf.reshape(N, S, T, C).transpose(0, 1, 3, 2)  # (N, S, C, T)

    scaler = StandardScaler()
    Xtr_n  = unflat(scaler.fit_transform(flat(X_train_real)).astype(np.float32), len(X_train_real))
    Xta_n  = unflat(scaler.transform(flat(X_train_aug)).astype(np.float32),      len(X_train_aug))
    Xte_n  = unflat(scaler.transform(flat(X_test)).astype(np.float32),            len(X_test))
    return Xtr_n, Xta_n, Xte_n, scaler


def normalize_all(X: np.ndarray):
    N, S, C, T = X.shape
    Xf = X.transpose(0, 1, 3, 2).reshape(-1, C)
    sc = StandardScaler()
    Xfn = sc.fit_transform(Xf).astype(np.float32)
    return Xfn.reshape(N, S, T, C).transpose(0, 1, 3, 2), sc


# training loop, lets get those weights updated
def train_model(
    X_real: np.ndarray, X_aug: np.ndarray,
    y_real: np.ndarray, y_aug: np.ndarray,
    y_sev_real: np.ndarray, y_sev_aug: np.ndarray,
    n_classes: int,
) -> MultiSectorCNN:
    _, _, n_channels, _ = X_real.shape
    model     = MultiSectorCNN(n_channels, n_classes, D_MODEL, N_HEADS, DROPOUT).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=EPOCHS)

    counts  = np.bincount(y_real, minlength=n_classes).astype(np.float32)
    weights = torch.tensor(len(y_real) / (n_classes * counts + 1e-6), device=device)
    loss_dir_fn = nn.CrossEntropyLoss(weight=weights)
    loss_sev_fn = nn.MSELoss()

    X_all = np.concatenate([X_real, X_aug], axis=0)
    y_all = np.concatenate([y_real, y_aug], axis=0)
    y_sev_all = np.concatenate([y_sev_real, y_sev_aug], axis=0)

    loader = DataLoader(
        TensorDataset(
            torch.tensor(X_all, dtype=torch.float32),
            torch.tensor(y_all, dtype=torch.long),
            torch.tensor(y_sev_all, dtype=torch.float32),
        ),
        batch_size=BATCH_SIZE, shuffle=True, drop_last=False,
    )

    model.train()
    for _ in range(EPOCHS):
        for xb, yb, ysevb in loader:
            xb, yb, ysevb = xb.to(device), yb.to(device), ysevb.to(device)
            optimizer.zero_grad()
            out_dir, out_sev = model(xb)
            loss_dir = loss_dir_fn(out_dir, yb)
            loss_sev = loss_sev_fn(out_sev, ysevb)
            loss = loss_dir + loss_sev
            loss.backward()
            optimizer.step()
        scheduler.step()

    return model


# inference function for predictions
@torch.no_grad()
def predict(model: MultiSectorCNN, X: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    model.eval()
    out_dir, out_sev = model(torch.tensor(X, dtype=torch.float32).to(device))
    return out_dir.argmax(1).cpu().numpy(), out_sev.cpu().numpy()


# leave one out cross validation stuff
def main() -> None:
    X_real      = np.load(DATA_DIR / "X_scans.npy")          # (N_scans, 3, C, T)
    y_real      = np.load(DATA_DIR / "y_scans.npy")
    y_sev_real  = np.load(DATA_DIR / "y_scans_severity.npy")
    s_keys      = np.load(DATA_DIR / "scan_keys_unique.npy",  allow_pickle=True)
    X_aug       = np.load(DATA_DIR / "X_scans_aug.npy")      # (N_scans*4, 3, C, T)
    y_aug       = np.load(DATA_DIR / "y_scans_aug.npy")
    y_sev_aug   = np.load(DATA_DIR / "y_scans_severity_aug.npy")
    classes     = np.load(DATA_DIR / "label_classes.npy",     allow_pickle=True)

    # Augmented scans have keys stored in the same repeated order
    # Each real scan has exactly 4 augmented copies (one per augmentation type)
    aug_keys = np.repeat(s_keys, 4)   # matches augmenters order in build script

    n_classes = len(classes)
    _, S, C, T = X_real.shape

    print(f"Device           : {device}")
    print(f"X_scans shape    : {X_real.shape}  (N_scans, 3_sectors, C, T)")
    print(f"Classes          : {classes}")
    print(f"Architecture     : SharedEncoder(Conv1D×3) + CrossSectorAttention({N_HEADS}h) + Dual Heads (Direction/Severity)")
    print()

    scan_true, scan_pred = [], []
    scan_true_sev, scan_pred_sev = [], []

    for i, held_key in enumerate(s_keys):
        tr_r_mask  = s_keys  != held_key
        tr_a_mask  = aug_keys != held_key
        te_mask    = s_keys  == held_key

        X_tr_r, y_tr_r, y_sev_tr_r = X_real[tr_r_mask],  y_real[tr_r_mask], y_sev_real[tr_r_mask]
        X_tr_a, y_tr_a, y_sev_tr_a = X_aug[tr_a_mask],   y_aug[tr_a_mask],  y_sev_aug[tr_a_mask]
        X_te,   y_te,   y_sev_te   = X_real[te_mask],     y_real[te_mask],   y_sev_real[te_mask]

        X_tr_r_n, X_tr_a_n, X_te_n, _ = fit_normalize(X_tr_r, X_tr_a, X_te)

        model = train_model(X_tr_r_n, X_tr_a_n, y_tr_r, y_tr_a, y_sev_tr_r, y_sev_tr_a, n_classes)
        pred_dir, pred_sev  = predict(model, X_te_n)
        pred_dir = pred_dir[0]
        pred_sev = pred_sev[0]

        scan_true.append(int(y_te[0]))
        scan_pred.append(int(pred_dir))
        scan_true_sev.append(float(y_sev_te[0]))
        scan_pred_sev.append(float(pred_sev))

        tick = "✓" if pred_dir == y_te[0] else "✗"
        print(f"  [{i+1:3d}/{len(s_keys)}] {held_key:<25}  "
              f"gt={classes[y_te[0]]:<5} (sev={y_sev_te[0]:.2f})  "
              f"pred={classes[pred_dir]:<5} (sev={pred_sev:.2f})  {tick}")

    # print out the results
    print()
    print("=" * 65)
    print("SCAN-LEVEL RESULTS  (Direction & Severity)")
    print("=" * 65)
    print(f"Direction Accuracy : {accuracy_score(scan_true, scan_pred):.4f}")
    print(f"Direction Macro F1 : {f1_score(scan_true, scan_pred, average='macro'):.4f}")
    print(f"Severity MSE       : {mean_squared_error(scan_true_sev, scan_pred_sev):.4f}")
    print(f"Severity MAE       : {mean_absolute_error(scan_true_sev, scan_pred_sev):.4f}")
    print()
    print(classification_report(scan_true, scan_pred, target_names=classes))
    print("Confusion matrix:")
    print(confusion_matrix(scan_true, scan_pred))

    # compare with the other models to see how we did
    print()
    print("=" * 65)
    print("FRAMEWORK COMPARISON  (same 116-scan LOO-CV, scan-level)")
    print("=" * 65)
    print(f"  {'Model':<25} {'Accuracy':>10}  {'Macro F1':>10}")
    print(f"  {'-'*48}")
    print(f"  {'XGBoost (aggregated)':<25} {'0.8621':>10}  {'0.8549':>10}")
    print(f"  {'Random Forest (agg.)':<25} {'0.8276':>10}  {'0.8264':>10}")
    print(f"  {'Naive 1D CNN (seq.)':<25} {'0.8103':>10}  {'0.8053':>10}")
    acc = accuracy_score(scan_true, scan_pred)
    f1  = f1_score(scan_true, scan_pred, average="macro")
    print(f"  {'MSCNN (this model)':<25} {acc:>10.4f}  {f1:>10.4f}  ← proposed")

    # train the final model on all the data and save it out
    print("\nTraining final model on ALL data...")
    X_all_n, final_scaler = normalize_all(np.concatenate([X_real, X_aug], axis=0))
    y_all = np.concatenate([y_real, y_aug])
    y_sev_all = np.concatenate([y_sev_real, y_sev_aug])
    n_real = len(X_real)
    final_model = train_model(
        X_all_n[:n_real], X_all_n[n_real:],
        y_all[:n_real],   y_all[n_real:],
        y_sev_all[:n_real], y_sev_all[n_real:],
        n_classes,
    )

    torch.save(final_model.state_dict(), HERE / "mscnn_direction.pt")
    joblib.dump(final_scaler, HERE / "mscnn_scaler.pkl")
    joblib.dump({"n_channels": C, "n_timesteps": T, "n_classes": n_classes,
                 "d_model": D_MODEL, "n_heads": N_HEADS}, HERE / "mscnn_config.pkl")
    np.save(HERE / "mscnn_classes.npy", classes)

    print(f"\nSaved to {HERE}/")
    print("  mscnn_direction.pt  — model weights")
    print("  mscnn_scaler.pkl    — StandardScaler")
    print("  mscnn_config.pkl    — architecture config")
    print("  mscnn_classes.npy   — class names")


if __name__ == "__main__":
    main()
