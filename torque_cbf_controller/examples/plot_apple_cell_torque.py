#!/usr/bin/env python3
"""Generate publication-grade 4-subplot analysis figure (Figure 5) for the
UR10e Apple Picking Cell simulation with Pinocchio dynamics and Direct Joint-Torque CBF control.

Subplots Generated:
  (5a) Multi-Surface Control Barrier Function Values B_i(t)
  (5b) Normal Velocities vs. Safe Velocity Envelopes v_min(zeta_i)
  (5c) Optimal CBF Joint Torques tau_cbf(t) & Command Torques tau_cmd(t)
  (5d) End-Effector Surface Distances zeta_i(t) relative to p_ths

Usage:
  python3 examples/plot_apple_cell_torque.py [--csv <path>] [--out <path>]
"""

import argparse
from pathlib import Path
import shutil
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ARTIFACT_DIR = Path("/home/galileo/.gemini/antigravity/brain/b763bcc4-8a9d-406e-8af6-1738d0c551b5")

def set_style():
    matplotlib.rcParams.update({
        "font.family": "serif",
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 11.5,
        "xtick.labelsize": 9.5,
        "ytick.labelsize": 9.5,
        "lines.linewidth": 1.8,
        "axes.grid": True,
        "grid.alpha": 0.35,
        "grid.linestyle": "--",
        "legend.fontsize": 8.5,
        "legend.framealpha": 0.92,
        "figure.dpi": 150,
        "savefig.dpi": 200,
        "savefig.bbox": "tight",
    })

def copy_to_artifact(src_path: Path):
    if ARTIFACT_DIR.is_dir():
        dst = ARTIFACT_DIR / src_path.name
        shutil.copy2(src_path, dst)
        print(f"Copied {src_path.name} to artifact directory: {dst}")

def plot_apple_cell_torque(csv_path: Path, out_plot: Path):
    if not csv_path.is_file():
        print(f"Error: CSV trajectory file not found: {csv_path}")
        return

    df = pd.read_csv(csv_path)
    set_style()

    has_forces = "f_ext_norm" in df.columns and "f_cbf_norm" in df.columns
    fig, axs = plt.subplots(2, 3 if has_forces else 2, figsize=(18 if has_forces else 14, 9.5))

    # (5a) Control Barrier Function Values B_i(t)
    ax = axs[0, 0]
    ax.plot(df["t"], df["B1"], label=r"$B_1(t)$ (Table Surface)", color="#1f77b4")
    ax.plot(df["t"], df["B2"], label=r"$B_2(t)$ ($45^\circ$ Wall Obstacle)", color="#d62728")
    ax.axhline(0, color="k", linestyle="--", linewidth=1.2, label=r"Safe Boundary $B_i=0$")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Barrier Value $B_i(q, \dot{q})$ [m/s]")
    ax.set_title(r"(a) Control Barrier Function Values $B_i(t) \geq 0$")
    ax.legend(loc="lower right")

    # (5b) Approach Velocities vs Safe Envelopes
    ax = axs[0, 1]
    ax.plot(df["t"], df["dzeta2"], label=r"Normal Velocity $\dot{\zeta}_2(t)$", color="#d62728")
    ax.plot(df["t"], df["v_min2"], label=r"Safe Envelope $v_{\min}(\zeta_2)$", color="k", linestyle="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Normal Velocity [m/s]")
    ax.set_title(r"(b) Velocity Envelope Tracking Along $45^\circ$ Wall")
    ax.legend(loc="lower right")

    if has_forces:
        # (5c) Kinesthetic Teaching Interaction Forces
        ax = axs[0, 2]
        ax.plot(df["t"], df["f_ext_norm"], label=r"Operator Guiding Force $\|F_{\mathrm{ext}}\|$", color="#0088cc", linewidth=1.8)
        ax.plot(df["t"], df["f_cbf_norm"], label=r"CBF Braking Reaction $\|F_{\mathrm{cbf}}\|$", color="#d62728", linewidth=2.0)
        ax.axhline(0, color="gray", linestyle=":", alpha=0.6)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Cartesian Force [N]")
        ax.set_title(r"(c) Kinesthetic Interaction: Operator Push vs. CBF Braking")
        ax.legend(loc="upper right")

    # (5d) Optimal CBF Joint Torques
    ax = axs[1, 0]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
    for j in range(6):
        ax.plot(df["t"], df[f"tau_cbf{j}"], label=f"Joint {j+1}", color=colors[j], alpha=0.85)
    ax.plot(df["t"], df["tau_cbf_norm"], "k--", linewidth=2.0, label=r"$\|\tau_{\mathrm{cbf}}\|$ (Norm)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"CBF Joint Torque $\tau_{\mathrm{cbf}}$ [Nm]")
    ax.set_title(r"(d) Optimal CBF Joint Torques $\tau_{\mathrm{cbf}}^*(t)$")
    ax.legend(loc="upper right", ncol=2)

    # (5e) Orthogonal Distances
    ax = axs[1, 1]
    ax.plot(df["t"], df["zeta1"], label=r"$\zeta_1(t)$ (Table Surface)", color="#1f77b4")
    ax.plot(df["t"], df["zeta2"], label=r"$\zeta_2(t)$ ($45^\circ$ Wall)", color="#d62728")
    ax.axhline(0.35, color="gray", linestyle=":", label=r"Braking Zone $p_{\mathrm{ths}} = 0.35$ m")
    ax.axhline(0, color="k", linestyle="-", linewidth=1.0, label="Contact Surface")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Orthogonal Distance $\zeta_i$ [m]")
    ax.set_title(r"(e) End-Effector Surface Distances $\zeta_i(t)$")
    ax.legend(loc="upper right")

    if has_forces:
        # (5f) Command Torques vs Pure Gravity Baseline
        ax = axs[1, 2]
        ax.plot(df["t"], df["tau_cmd_norm"], label=r"Command Torque $\|\tau_{\mathrm{cmd}}\|$", color="#2ca02c", linewidth=1.8)
        ax.plot(df["t"], df["tau_cbf_norm"], label=r"CBF Intervention $\|\tau_{\mathrm{cbf}}\|$", color="#d62728", linestyle="--")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Torque Norm [Nm]")
        ax.set_title(r"(f) Total Actuation: Gravity Comp + CBF Braking")
        ax.legend(loc="upper right")

    plt.tight_layout()
    out_plot.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_plot)
    plt.close()
    print(f"Saved Figure 5 to: {out_plot}")
    copy_to_artifact(out_plot)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("/home/galileo/projects/damping_cbf_ws/src/torque_cbf_controller/examples/csv/sim_apple_cell_torque_trajectory.csv")
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("/home/galileo/projects/damping_cbf_ws/src/torque_cbf_controller/examples/plots/fig5_apple_cell_torque_pinocchio.png")
    )
    args = parser.parse_args()
    plot_apple_cell_torque(args.csv, args.out)

if __name__ == "__main__":
    main()
