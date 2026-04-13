"""
1D CNN with channel attention for real-time EMG gesture classification.

Architecture: ChannelAttention → 3 × (Conv1d + BN + ReLU + Pool) → Flatten → Dropout → Linear
The SE-style attention module learns which of the 3 input sensors (Flexors /
Extensors / Pronators) is most relevant for each gesture, letting the network
dynamically weight axes per inference.

Designed for 3 input channels and 7 output gesture classes.
Total ~55K parameters — <5ms TensorRT inference on Jetson Orin Nano.

Usage:
    from model import EMGCNN
    net = EMGCNN()
    logits = net(torch.randn(1, 3, 205))  # (batch, channels, time)
"""

from __future__ import annotations

import torch
import torch.nn as nn

NUM_CLASSES = 7
IN_CHANNELS = 3
WINDOW_SIZE = 205


class ChannelAttention(nn.Module):
    """Squeeze-and-Excitation attention over input EMG channels.
    Learns per-channel importance weights so the model can focus on the
    sensor axis most informative for the current gesture."""

    def __init__(self, channels: int):
        super().__init__()
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.fc = nn.Sequential(
            nn.Linear(channels, channels),
            nn.ReLU(inplace=True),
            nn.Linear(channels, channels),
            nn.Sigmoid(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, C, T)
        w = self.pool(x).squeeze(-1)   # (B, C)
        w = self.fc(w).unsqueeze(-1)   # (B, C, 1)
        return x * w


class EMGCNN(nn.Module):

    def __init__(self, in_channels: int = IN_CHANNELS,
                 num_classes: int = NUM_CLASSES,
                 dropout: float = 0.3):
        super().__init__()

        self.attention = ChannelAttention(in_channels)

        self.features = nn.Sequential(
            # Block 1:  (B, 3, 205) → (B, 32, 102)
            nn.Conv1d(in_channels, 32, kernel_size=7, padding=3),
            nn.BatchNorm1d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool1d(kernel_size=2),

            # Block 2:  (B, 32, 102) → (B, 64, 51)
            nn.Conv1d(32, 64, kernel_size=5, padding=2),
            nn.BatchNorm1d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool1d(kernel_size=2),

            # Block 3:  (B, 64, 51) → (B, 128, 1)
            nn.Conv1d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool1d(1),
        )

        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Dropout(dropout),
            nn.Linear(128, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (batch, in_channels, window_size)  e.g. (B, 3, 205)
        Returns:
            logits: (batch, num_classes)
        """
        x = self.attention(x)
        return self.classifier(self.features(x))


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    net = EMGCNN()
    print(net)
    print(f"\nTrainable parameters: {count_parameters(net):,}")

    dummy = torch.randn(2, IN_CHANNELS, WINDOW_SIZE)
    out = net(dummy)
    print(f"Input shape:  {dummy.shape}")
    print(f"Output shape: {out.shape}")
