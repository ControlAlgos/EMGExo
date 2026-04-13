#!/usr/bin/env python3
"""Generate per-joint position & torque-proxy plots from an EMGExo session CSV."""

import sys
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def main():
    if len(sys.argv) < 2:
        print("Usage: plot_session.py <session.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    base = os.path.splitext(csv_path)[0]
    t = df["timestamp_ms"].values / 1000.0  # convert to seconds

    joint_names = ["M1 Rad/Uln", "M2 Flx/Ext", "M3 Pro/Sup"]
    motor_cols = ["motor_m1", "motor_m2", "motor_m3"]
    colors = ["#00bcd4", "#4caf50", "#2196f3"]

    # --- Position plot ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Joint Positions Over Time", fontsize=14, fontweight="bold")

    for i, (ax, col, name, c) in enumerate(zip(axes, motor_cols, joint_names, colors)):
        if col not in df.columns:
            ax.text(0.5, 0.5, f"{col} not in CSV", transform=ax.transAxes, ha="center")
            continue
        pos = df[col].values
        ax.plot(t, pos, color=c, linewidth=0.8, label=name)
        ax.fill_between(t, pos, alpha=0.15, color=c)
        ax.set_ylabel("Position (rad)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
        ax.set_title(name, fontsize=11)

        p_max = np.max(np.abs(pos))
        ax.text(0.01, 0.95, f"Range: [{np.min(pos):.3f}, {np.max(pos):.3f}] rad",
                transform=ax.transAxes, fontsize=9, va="top",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    pos_path = base + "_positions.png"
    fig.savefig(pos_path, dpi=150)
    plt.close(fig)
    print(f"[plot] Position plot → {pos_path}")

    # --- Torque proxy plot (Kp * position_error ≈ commanded torque) ---
    # Since we don't have actual torque feedback, we compute the velocity
    # (position derivative) as a proxy for the effort/torque profile.
    fig2, axes2 = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    fig2.suptitle("Joint Velocity (Torque Proxy) Over Time", fontsize=14, fontweight="bold")

    dt = np.median(np.diff(t)) if len(t) > 1 else 0.005

    for i, (ax, col, name, c) in enumerate(zip(axes2, motor_cols, joint_names, colors)):
        if col not in df.columns:
            continue
        pos = df[col].values
        vel = np.gradient(pos, dt)
        # Smooth with a 50-sample rolling mean
        kernel = np.ones(50) / 50
        vel_smooth = np.convolve(vel, kernel, mode="same")

        ax.plot(t, vel_smooth, color=c, linewidth=0.8, label=f"{name} velocity")
        ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
        ax.set_ylabel("Velocity (rad/s)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
        ax.set_title(name, fontsize=11)

        ax.text(0.01, 0.95,
                f"Peak: {np.max(np.abs(vel_smooth)):.2f} rad/s",
                transform=ax.transAxes, fontsize=9, va="top",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    axes2[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    torque_path = base + "_torque_proxy.png"
    fig2.savefig(torque_path, dpi=150)
    plt.close(fig2)
    print(f"[plot] Torque proxy plot → {torque_path}")

    # --- Latency plot ---
    if "total_us" in df.columns:
        fig3, ax3 = plt.subplots(figsize=(14, 4))
        lat_ms = df["total_us"].values / 1000.0
        ax3.plot(t, lat_ms, color="#ff9800", linewidth=0.5, alpha=0.6)
        kernel = np.ones(100) / 100
        lat_smooth = np.convolve(lat_ms, kernel, mode="same")
        ax3.plot(t, lat_smooth, color="#e65100", linewidth=1.5, label="100-pt avg")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Sensor→Motor Latency (ms)")
        ax3.set_title("End-to-End Latency", fontsize=14, fontweight="bold")
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.text(0.01, 0.95,
                 f"Avg: {np.mean(lat_ms):.2f} ms  Max: {np.max(lat_ms):.2f} ms",
                 transform=ax3.transAxes, fontsize=10, va="top",
                 bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
        plt.tight_layout()
        lat_path = base + "_latency.png"
        fig3.savefig(lat_path, dpi=150)
        plt.close(fig3)
        print(f"[plot] Latency plot → {lat_path}")

    print("[plot] All plots generated.")

if __name__ == "__main__":
    main()
