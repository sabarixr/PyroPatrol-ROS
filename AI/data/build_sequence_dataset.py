"""
build_sequence_dataset.py

Also produces scan-level tensors for the Multi-Sector CNN:
  X_scans.npy          — (N_scans, 3, 10, 15)  real scans only
  X_scans_aug.npy      — (N_scans×4, 3, 10, 15) augmented scans
  y_scans.npy          — (N_scans,)  labels (one per scan)
  y_scans_aug.npy      — (N_scans×4,) augmented labels
  scan_keys_unique.npy — (N_scans,)  unique scan keys
=========================
Combines raw_data_old.csv + raw_data_new.csv into a single augmented
sequence dataset ready for 1D CNN training.

Dataset curation strategy
--------------------------
Multiple independently collected experimental datasets are merged to capture
diversity across fire conditions (fire-only, fire+smoke, low/moderate/high
intensity) and environments.  The merged dataset is then augmented with
physics-aware perturbations so the model learns robust decision boundaries
rather than memorising only extreme sensor signatures.

Augmentation applied to every real sequence (training only):
  1. Gaussian noise  — sensor acquisition noise (different amplitude per channel)
  2. Amplitude scale — simulates sensor gain drift / calibration variance
  3. Time-shift      — circular shift ±2 steps; sensor patterns arrive at
                       slightly different sample indices across repetitions
  4. Channel dropout — randomly zeros one non-critical channel per sequence to
                       prevent over-reliance on a single sensor
Each technique is applied independently, yielding 4× more sequences.

Key design decisions
---------------------
- Unique scan key = (session_id, scan_id) — avoids confusion from identical
  scan_id numbers across sessions.
- Only "holding" phase rows are used (all rows in this dataset happen to be
  holding, but the filter is kept for robustness).
- Sequences are fixed to N_TIMESTEPS=15 by truncation or last-row padding.
- servo_angle is included as a channel so the CNN can learn which direction
  the sensor was pointing at each timestep.
- NaNs are filled with per-column medians before shaping.

Output files (saved to same directory as this script)
-------------------------------------------------------
  raw_data.csv          — merged raw CSV (old + new, deduplicated)
  X_sequences.npy       — (N, 10, 15)  float32, channels-first
  y_sequences.npy       — (N,)         int64   encoded labels
  scan_keys.npy         — (N,)         str     "session_scan" unique key
  sectors.npy           — (N,)         str     LEFT / FRONT / RIGHT
  label_classes.npy     — (n_classes,) str     class name mapping
  augmentation_mask.npy — (N,)         bool    True = augmented sample
"""

from __future__ import annotations

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.preprocessing import LabelEncoder

HERE = Path(__file__).parent

# i think this will do the paths, ayy dont mess this upp
RAW_OLD = HERE / "raw_data_old.csv"
RAW_NEW = HERE / "raw_data_new.csv"
RAW_OUT = HERE / "raw_data.csv"          # merged output

# config stuff, basically just some numbers we need
N_TIMESTEPS = 15
TARGET      = "target_direction"

# 10 sensor channels (order matters — Conv1d sees them as separate signals)
SENSOR_COLS = [
    "servo_angle",   # direction context: where the sensor head is pointing
    "mq2",           # smoke / combustion byproducts
    "mq5",           # LPG / general flammable gas
    "temperature",   # ambient thermal
    "flame_left",    # IR flame detector — left element
    "flame_center",  # IR flame detector — centre element
    "flame_right",   # IR flame detector — right element
    "ax",            # accelerometer X  (vibration / tilt context)
    "ay",            # accelerometer Y
    "az",            # accelerometer Z
]
assert len(SENSOR_COLS) == 10

# Channels that are "durable" targets for dropout augmentation
# (we avoid dropping servo_angle as it is the primary direction signal)
DROPOUT_CANDIDATE_CHANNELS = [1, 2, 3, 4, 5, 6, 7, 8, 9]  # indices into SENSOR_COLS

# Approximate per-channel noise std (relative to typical signal range)
# Tuned to match real sensor noise characteristics:
#   servo_angle : deterministic → very small noise
#   gas sensors : high acquisition noise → larger noise
#   temperature : very stable → tiny noise
#   flame       : discrete/noisy  → moderate noise
#   IMU         : moderate noise
NOISE_STD = np.array([
    0.5,    # servo_angle (degrees jitter)
    8.0,    # mq2
    5.0,    # mq5
    0.05,   # temperature
    30.0,   # flame_left
    30.0,   # flame_center
    30.0,   # flame_right
    0.02,   # ax
    0.02,   # ay
    0.02,   # az
], dtype=np.float32)


# first we merge the raw csvs together so we have all the data

def merge_raw() -> pd.DataFrame:
    """
    Combine old and new raw CSVs.
    Deduplicates on (session_id, scan_id, sample_index, sector) so that any
    rows truly identical across files appear only once.
    Adds a 'scan_key' column = '<session_id>__<scan_id>' for use as the
    unique-scan identifier throughout the pipeline.
    """
    old = pd.read_csv(RAW_OLD)
    new = pd.read_csv(RAW_NEW)

    print(f"  raw_data_old : {len(old):5d} rows, {old['scan_id'].nunique()} unique scan_ids")
    print(f"  raw_data_new : {len(new):5d} rows, {new['scan_id'].nunique()} unique scan_ids")

    combined = pd.concat([old, new], ignore_index=True)

    # Deduplicate on natural primary key
    pk = ["session_id", "scan_id", "sector", "sample_index"]
    before = len(combined)
    combined = combined.drop_duplicates(subset=pk)
    after   = len(combined)
    if before != after:
        print(f"  Removed {before - after} duplicate rows.")

    # Create stable unique scan key
    combined["scan_key"] = (
        combined["session_id"].astype(str) + "__" + combined["scan_id"].astype(str)
    )

    combined.to_csv(RAW_OUT, index=False)
    print(f"  Merged CSV   : {len(combined):5d} rows → saved to {RAW_OUT.name}")
    print(f"  Unique scan_keys : {combined['scan_key'].nunique()}")
    return combined


# build the real sequences from the merged data

def extract_sequences(df: pd.DataFrame):
    """
    Group by (scan_key, sector), sort by sample_index, extract sensor readings.
    Returns parallel lists of arrays, labels, scan_keys, sectors.
    """
    if "phase" in df.columns:
        df = df[df["phase"] == "holding"].copy()

    # Fill NaNs with per-column median
    for col in SENSOR_COLS:
        if col in df.columns:
            med = df[col].median()
            df[col] = df[col].fillna(med if not pd.isna(med) else 0.0)

    arrays, labels, severities, keys, sectors = [], [], [], [], []

    for (scan_key, sector), grp in df.groupby(["scan_key", "sector"]):
        grp = grp.sort_values("sample_index")

        label_val = grp[TARGET].dropna()
        if label_val.empty:
            continue
        label = str(label_val.iloc[0]).strip().lower()
        if label == "" or label == "nan":
            continue

        # Synthesize severity score [0.0, 1.0]
        fp = grp["fire_present"].dropna()
        dist = grp["distance_to_fire_cm"].dropna()

        severity = 0.0
        is_fire = bool(fp.iloc[0]) if not fp.empty else False

        if is_fire:
            if not dist.empty:
                d = float(dist.iloc[0])
                # Scale: closer = higher severity. e.g. 5cm -> ~1.0, 300cm -> ~0.1
                severity = max(0.1, 1.0 - (d / 350.0))
            else:
                severity = 0.5
        else:
            severity = 0.0

        readings = grp[SENSOR_COLS].values.astype(np.float32)   # (n, C)
        n = len(readings)

        if n >= N_TIMESTEPS:
            readings = readings[:N_TIMESTEPS]
        else:
            pad = np.tile(readings[-1:], (N_TIMESTEPS - n, 1))
            readings = np.vstack([readings, pad])

        arrays.append(readings.T)   # (C, T) — channels-first for Conv1d
        labels.append(label)
        severities.append(severity)
        keys.append(scan_key)
        sectors.append(sector)

    return arrays, labels, severities, keys, sectors


# augmentation to make the model tough

def aug_gaussian_noise(seq: np.ndarray) -> np.ndarray:
    """Add per-channel Gaussian noise matching real sensor characteristics."""
    noise = np.random.randn(*seq.shape).astype(np.float32)
    noise *= NOISE_STD[:, None]          # broadcast over timesteps
    return seq + noise


def aug_amplitude_scale(seq: np.ndarray) -> np.ndarray:
    """Per-channel amplitude scaling ±8% — simulates sensor gain drift."""
    scale = 1.0 + (np.random.rand(seq.shape[0], 1).astype(np.float32) - 0.5) * 0.16
    return seq * scale


def aug_time_shift(seq: np.ndarray, max_shift: int = 2) -> np.ndarray:
    """Circular time-shift ±max_shift steps — sensor arrival timing variance."""
    shift = np.random.randint(-max_shift, max_shift + 1)
    return np.roll(seq, shift, axis=1)


def aug_channel_dropout(seq: np.ndarray) -> np.ndarray:
    """Zero out one randomly chosen non-critical channel."""
    ch = np.random.choice(DROPOUT_CANDIDATE_CHANNELS)
    seq = seq.copy()
    seq[ch, :] = 0.0
    return seq


AUGMENTERS = [aug_gaussian_noise, aug_amplitude_scale, aug_time_shift, aug_channel_dropout]


def augment_sequences(arrays: list[np.ndarray]) -> tuple[list[np.ndarray], list[bool]]:
    """
    Apply each augmentation independently to every real sequence.
    Returns the 4 augmented copies alongside a mask (True = augmented).
    """
    aug_arrays, aug_mask = [], []
    for seq in arrays:
        for fn in AUGMENTERS:
            aug_arrays.append(fn(seq.copy()))
            aug_mask.append(True)
    return aug_arrays, aug_mask


# assemble everything and save it out

def main() -> None:
    print("=" * 60)
    print("STEP 1 — Merge raw CSVs")
    print("=" * 60)
    df = merge_raw()

    print()
    print("=" * 60)
    print("STEP 2 — Extract real sequences")
    print("=" * 60)
    real_arrays, real_labels, real_severities, real_keys, real_sectors = extract_sequences(df)
    n_real = len(real_arrays)
    print(f"  Real sequences   : {n_real}")
    from collections import Counter
    print(f"  Label counts     : {dict(Counter(real_labels))}")

    print()
    print("=" * 60)
    print("STEP 3 — Augment (×4 per real sequence)")
    print("=" * 60)
    aug_arrays, aug_mask_flags = augment_sequences(real_arrays)

    # Repeat labels / keys / sectors for augmented copies (same order as AUGMENTERS)
    aug_labels     = [lbl for lbl in real_labels     for _ in AUGMENTERS]
    aug_severities = [sev for sev in real_severities for _ in AUGMENTERS]
    aug_keys       = [k   for k   in real_keys       for _ in AUGMENTERS]
    aug_sectors    = [s   for s   in real_sectors    for _ in AUGMENTERS]

    print(f"  Augmented sequences : {len(aug_arrays)}")

    # Combine real + augmented
    all_arrays     = real_arrays     + aug_arrays
    all_labels     = real_labels     + aug_labels
    all_severities = real_severities + aug_severities
    all_keys       = real_keys       + aug_keys
    all_sectors    = real_sectors    + aug_sectors
    all_is_aug     = [False] * n_real + aug_mask_flags

    print()
    print("=" * 60)
    print("STEP 4 — Encode and save")
    print("=" * 60)

    X = np.stack(all_arrays, axis=0).astype(np.float32)   # (N, C, T)

    # Final NaN guard
    nan_n = np.isnan(X).sum()
    if nan_n:
        print(f"  WARNING: {nan_n} NaNs replaced with 0.")
        X = np.nan_to_num(X, nan=0.0)

    le = LabelEncoder()
    y  = le.fit_transform(all_labels).astype(np.int64)
    y_sev = np.array(all_severities, dtype=np.float32)

    scan_keys      = np.array(all_keys)
    sectors_arr    = np.array(all_sectors)
    aug_mask_arr   = np.array(all_is_aug, dtype=bool)
    classes        = le.classes_

    print(f"  X shape          : {X.shape}  (N, C, T)")
    print(f"  Classes          : {classes}")
    total_counts   = Counter(all_labels)
    real_counts    = Counter(real_labels)
    print(f"  Label dist (real)    : { {c: real_counts[c] for c in classes} }")
    print(f"  Label dist (total)   : { {c: total_counts[c] for c in classes} }")

    np.save(HERE / "X_sequences.npy",      X)
    np.save(HERE / "y_sequences.npy",      y)
    np.save(HERE / "y_sequences_severity.npy", y_sev)
    np.save(HERE / "scan_keys.npy",        scan_keys)
    np.save(HERE / "sectors.npy",          sectors_arr)
    np.save(HERE / "label_classes.npy",    classes)
    np.save(HERE / "augmentation_mask.npy", aug_mask_arr)

    print()
    print("Saved:")
    for fname, arr in [
        ("X_sequences.npy",       X),
        ("y_sequences.npy",       y),
        ("y_sequences_severity.npy", y_sev),
        ("scan_keys.npy",         scan_keys),
        ("sectors.npy",           sectors_arr),
        ("label_classes.npy",     classes),
        ("augmentation_mask.npy", aug_mask_arr),
    ]:
        print(f"  {fname:<28} {arr.shape}")

    print()
    print(f"Real sequences   : {n_real}")
    print(f"Augmented added  : {len(aug_arrays)}")
    print(f"Total            : {len(X)}")
    print()
    print("=" * 60)
    print("STEP 5 — Build scan-level tensors for Multi-Sector CNN")
    print("=" * 60)
    _build_scan_tensors(HERE)
    print()
    print("Done.")


SECTOR_ORDER = ["LEFT", "FRONT", "RIGHT"]


def _build_scan_tensors(out_dir: Path) -> None:
    """
    Reshape per-sector sequences into per-scan tensors.
    Each scan has exactly 3 sectors (LEFT, FRONT, RIGHT) in fixed order.
    Produces both real and augmented scan tensors.
    """
    X        = np.load(out_dir / "X_sequences.npy")
    y        = np.load(out_dir / "y_sequences.npy")
    y_sev    = np.load(out_dir / "y_sequences_severity.npy")
    skeys    = np.load(out_dir / "scan_keys.npy",        allow_pickle=True)
    sects    = np.load(out_dir / "sectors.npy",          allow_pickle=True)
    aug_mask = np.load(out_dir / "augmentation_mask.npy")

    def _group(X_g, y_g, y_sev_g, keys_g, sects_g, n_augs=1):
        scan_X, scan_y, scan_y_sev, scan_key_out = [], [], [], []
        for key in np.unique(keys_g):
            m = keys_g == key
            if not all(s in sects_g[m] for s in SECTOR_ORDER):
                continue  # skip incomplete scans
            for aug_idx in range(n_augs):
                tensor = np.stack(
                    [X_g[m][sects_g[m] == s][aug_idx] for s in SECTOR_ORDER], axis=0
                )  # (3, C, T)
                scan_X.append(tensor)
                scan_y.append(int(y_g[m][0]))
                scan_y_sev.append(float(y_sev_g[m][0]))
                scan_key_out.append(key)
        return np.stack(scan_X), np.array(scan_y, dtype=np.int64), np.array(scan_y_sev, dtype=np.float32), np.array(scan_key_out)

    # Real scans
    rm = ~aug_mask
    X_scans, y_scans, y_scans_sev, skeys_unique = _group(X[rm], y[rm], y_sev[rm], skeys[rm], sects[rm], n_augs=1)

    # Augmented scans (built from augmented per-sector sequences)
    am = aug_mask
    X_scans_aug, y_scans_aug, y_scans_sev_aug, _ = _group(X[am], y[am], y_sev[am], skeys[am], sects[am], n_augs=4)

    np.save(out_dir / "X_scans.npy",         X_scans)
    np.save(out_dir / "y_scans.npy",         y_scans)
    np.save(out_dir / "y_scans_severity.npy", y_scans_sev)
    np.save(out_dir / "scan_keys_unique.npy", skeys_unique)
    np.save(out_dir / "X_scans_aug.npy",     X_scans_aug)
    np.save(out_dir / "y_scans_aug.npy",     y_scans_aug)
    np.save(out_dir / "y_scans_severity_aug.npy", y_scans_sev_aug)

    print(f"  X_scans shape     : {X_scans.shape}   (N_scans, 3_sectors, C, T)")
    print(f"  X_scans_aug shape : {X_scans_aug.shape}")
    print(f"  y_scans dist      : {dict(zip(*np.unique(y_scans, return_counts=True)))}")
    print(f"  Saved to {out_dir}/")


if __name__ == "__main__":
    main()
