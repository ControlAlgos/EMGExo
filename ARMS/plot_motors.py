#!/usr/bin/env python3
"""Plot motor position per motor from motor_log.csv."""
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "motor_log.csv"
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} samples from {csv_path}")
    print(f"Motor IDs found: {sorted(df['motor_id'].unique())}")

    # Use time_s if available, otherwise use sample index
    x_col = "time_s" if "time_s" in df.columns else "sample"
    x_label = "Time (s)" if x_col == "time_s" else "Sample"

    motor_ids = sorted(df["motor_id"].unique())
    n = len(motor_ids)
    colors = {1: "#e74c3c", 2: "#2ecc71", 3: "#3498db"}
    labels = {1: "Motor 1 (Radial/Ulnar)", 2: "Motor 2 (Flex/Ext)", 3: "Motor 3 (Pro/Sup)"}

    fig, axes = plt.subplots(n, 1, figsize=(12, 3.5 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for i, mid in enumerate(motor_ids):
        mdf = df[df["motor_id"] == mid]
        c = colors.get(mid, "#95a5a6")
        label = labels.get(mid, f"Motor {mid}")

        axes[i].plot(mdf[x_col], mdf["position_rad"], color=c, linewidth=0.8)
        axes[i].set_ylabel("Position (rad)")
        axes[i].set_title(label)
        axes[i].grid(True, alpha=0.3)
        axes[i].axhline(0, color="gray", linewidth=0.5, linestyle="--")

        pos_min = mdf["position_rad"].min()
        pos_max = mdf["position_rad"].max()
        axes[i].annotate(f"min={pos_min:.3f} rad  max={pos_max:.3f} rad",
                         xy=(0.01, 0.95), xycoords="axes fraction",
                         fontsize=9, verticalalignment="top",
                         bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.5))

    axes[-1].set_xlabel(x_label)
    fig.suptitle("Motor Feedback — Position", fontsize=14, fontweight="bold")
    plt.tight_layout()

    out_png = csv_path.replace(".csv", ".png")
    plt.savefig(out_png, dpi=150)
    print(f"Saved plot to {out_png}")
    # Don't call plt.show() — it blocks the terminal. Just open the file.
    os.system(f"xdg-open {out_png} 2>/dev/null &")


if __name__ == "__main__":
    main()
