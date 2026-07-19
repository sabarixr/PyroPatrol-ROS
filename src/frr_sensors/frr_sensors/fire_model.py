import torch
import torch.nn as nn


class SharedSectorEncoder(nn.Module):
    """Shared 1D CNN applied independently to each sector time-series."""

    def __init__(self, n_channels: int, d_model: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv1d(n_channels, 32, kernel_size=3, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),
            nn.Conv1d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.Conv1d(64, d_model, kernel_size=3, padding=1),
            nn.BatchNorm1d(d_model),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool1d(1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)


class MultiSectorCNN(nn.Module):
    """MSCNN with cross-sector attention and dual heads."""

    def __init__(
        self,
        n_channels: int,
        n_classes: int,
        d_model: int = 128,
        n_heads: int = 4,
        dropout: float = 0.3,
    ):
        super().__init__()
        self.encoder = SharedSectorEncoder(n_channels, d_model)
        self.cross_attn = nn.MultiheadAttention(
            embed_dim=d_model,
            num_heads=n_heads,
            dropout=dropout,
            batch_first=True,
        )
        self.attn_norm = nn.LayerNorm(d_model)
        self.fc = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(d_model * 3, d_model),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout / 2),
        )
        self.head_dir = nn.Linear(d_model, n_classes)
        self.head_sev = nn.Sequential(nn.Linear(d_model, 1), nn.Sigmoid())

    def forward(self, x: torch.Tensor):
        batch_size, sectors, channels, timesteps = x.shape
        x_flat = x.view(batch_size * sectors, channels, timesteps)
        encoded = self.encoder(x_flat)
        sector_embeddings = encoded.view(batch_size, sectors, -1)
        attn_out, _ = self.cross_attn(
            sector_embeddings,
            sector_embeddings,
            sector_embeddings,
        )
        sector_embeddings = self.attn_norm(sector_embeddings + attn_out)
        combined = sector_embeddings.view(batch_size, -1)
        features = self.fc(combined)
        return self.head_dir(features), self.head_sev(features).squeeze(-1)
