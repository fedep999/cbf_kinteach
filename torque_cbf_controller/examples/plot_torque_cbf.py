#!/usr/bin/env python3
"""Plot and analyze the benchmark simulation results produced by `example_torque_cbf`.

Generates 5 publication-grade figures:
  1. `fig1_torque_profiles.png`     - Comparison of shaping profiles h_1..h_5 in phase
                                      space, spatial torque distribution, and time.
  2. `fig2_torque_entry_speeds.png` - Safe-set invariance (B > 0), boundary tracking (B = 0),
                                      and exponential recovery (B < 0) across entry speeds.
  3. `fig3_torque_external_push.png`- Effect of downward external force (F_ext = -15 N)
                                      with and without drift acceleration compensation (a_drift).
  4. `fig4_torque_manipulator_7dof.png` - 7-DoF manipulator joint torques and 2-surface corner
                                          coupled CBF-QP torque synthesis.
  5. `cbf_torque_summary.png`       - Combined overview dashboard of all four benchmarks.

Usage:
  python3 examples/plot_torque_cbf.py [--csv-dir DIR] [--out-dir DIR]
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

def plot_experiment_1(csv_dir: Path, out_dir: Path):
    csv_file = csv_dir / "torque_cbf_profiles.csv"
    if not csv_file.is_file():
        print(f"File not found: {csv_file}")
        return
    df = pd.read_csv(csv_file)
    profiles = df["profile"].unique()

    colors = {
        "h1 (Constant)": "#1f77b4",
        "h2 (Root x^1/5)": "#ff7f0e",
        "h3 (Linear x)": "#2ca02c",
        "h4 (Quintic x^5)": "#d62728",
        "h5 (Smoothstep)": "#9467bd"
    }

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))

    # Subplot 1: Phase Plane (zeta vs dzeta)
    ax = axs[0]
    for p in profiles:
        sub = df[df["profile"] == p]
        ax.plot(sub["zeta"] * 1e3, sub["dzeta"] * 1e3, label=p, color=colors.get(p, "k"))
    # Theoretical smoothstep envelope
    sub5 = df[df["profile"] == "h5 (Smoothstep)"]
    ax.plot(sub5["zeta"] * 1e3, sub5["v_min"] * 1e3, "k--", label=r"Safe Envelope $v_{\min}(\zeta)$", alpha=0.7)
    ax.axvline(0, color="gray", linestyle=":", label="Surface")
    ax.set_xlabel(r"Distance $\zeta$ [mm]")
    ax.set_ylabel(r"Normal Velocity $\dot{\zeta}$ [mm/s]")
    ax.set_title("Phase-Space Trajectories")
    ax.legend(loc="lower right")

    # Subplot 2: Distance vs Time
    ax = axs[1]
    for p in profiles:
        sub = df[df["profile"] == p]
        ax.plot(sub["t"], sub["zeta"] * 1e3, label=p, color=colors.get(p, "k"))
    ax.axhline(0, color="gray", linestyle=":", label="Surface")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Distance $\zeta$ [mm]")
    ax.set_title(r"Approach Distance $\zeta(t)$")
    ax.legend(loc="upper right")

    # Subplot 3: CBF Joint Torque vs Distance
    ax = axs[2]
    for p in profiles:
        sub = df[df["profile"] == p]
        ax.plot(sub["zeta"] * 1e3, sub["tau_cbf"], label=p, color=colors.get(p, "k"))
    ax.set_xlabel(r"Distance $\zeta$ [mm]")
    ax.set_ylabel(r"Optimal CBF Torque $\tau_{\mathrm{cbf}}$ [Nm]")
    ax.set_title(r"Synthesized Braking Torque $\tau_{\mathrm{cbf}}(\zeta)$")
    ax.legend(loc="upper right")

    plt.tight_layout()
    out_path = out_dir / "fig1_torque_profiles.png"
    plt.savefig(out_path)
    plt.close()
    copy_to_artifact(out_path)

def plot_experiment_2(csv_dir: Path, out_dir: Path):
    csv_file = csv_dir / "torque_cbf_entry_speeds.csv"
    if not csv_file.is_file():
        return
    df = pd.read_csv(csv_file)
    cases = df["case"].unique()
    colors = {"Slow": "#2ca02c", "Nominal": "#1f77b4", "Fast": "#d62728"}

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))

    # Subplot 1: Phase plane
    ax = axs[0]
    for c in cases:
        sub = df[df["case"] == c]
        ax.plot(sub["zeta"] * 1e2, sub["dzeta"], label=f"{c} entry", color=colors.get(c, "k"))
    sub_nom = df[df["case"] == "Nominal"]
    ax.plot(sub_nom["zeta"] * 1e2, sub_nom["v_min"], "k--", label=r"$v_{\min}(\zeta)$", alpha=0.8)
    ax.set_xlabel(r"Distance $\zeta$ [cm]")
    ax.set_ylabel(r"Velocity $\dot{\zeta}$ [m/s]")
    ax.set_title("Entry Velocity Phase Portraits")
    ax.legend(loc="lower right")

    # Subplot 2: Barrier B(t)
    ax = axs[1]
    for c in cases:
        sub = df[df["case"] == c]
        ax.plot(sub["t"], sub["B"], label=f"{c} ($B(0) {'<' if c=='Fast' else ('>' if c=='Slow' else '=')} 0$)", color=colors.get(c, "k"))
    ax.axhline(0, color="k", linestyle=":", label=r"Safe Boundary $B=0$")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Barrier Function $B(q, \dot{q})$ [m/s]")
    ax.set_title(r"Forward Invariance & Recovery $B(t) \geq 0$")
    ax.legend(loc="lower right")

    # Subplot 3: CBF Torque
    ax = axs[2]
    for c in cases:
        sub = df[df["case"] == c]
        ax.plot(sub["t"], sub["tau_cbf"], label=f"{c} entry", color=colors.get(c, "k"))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"CBF Torque $\tau_{\mathrm{cbf}}$ [Nm]")
    ax.set_title(r"Braking Torque Response $\tau_{\mathrm{cbf}}(t)$")
    ax.legend(loc="upper right")

    plt.tight_layout()
    out_path = out_dir / "fig2_torque_entry_speeds.png"
    plt.savefig(out_path)
    plt.close()
    copy_to_artifact(out_path)

def plot_experiment_3(csv_dir: Path, out_dir: Path):
    csv_file = csv_dir / "torque_cbf_external_push.csv"
    if not csv_file.is_file():
        return
    df = pd.read_csv(csv_file)
    modes = df["mode"].unique()
    colors = {"With Drift Term (a_drift)": "#1f77b4", "Without Drift Term": "#d62728"}

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))

    # Subplot 1: Distance zeta(t)
    ax = axs[0]
    for m in modes:
        sub = df[df["mode"] == m]
        ax.plot(sub["t"], sub["zeta"] * 1e3, label=m, color=colors.get(m, "k"))
    ax.axhline(0, color="gray", linestyle=":", label="Contact Plane")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Distance $\zeta$ [mm]")
    ax.set_title(r"Operator Push ($F_{\mathrm{ext}, z} = -15$ N) Trajectory")
    ax.legend(loc="upper right")

    # Subplot 2: Phase space
    ax = axs[1]
    for m in modes:
        sub = df[df["mode"] == m]
        ax.plot(sub["zeta"] * 1e3, sub["dzeta"] * 1e3, label=m, color=colors.get(m, "k"))
    ax.axvline(0, color="gray", linestyle=":", label="Surface")
    ax.set_xlabel(r"Distance $\zeta$ [mm]")
    ax.set_ylabel(r"Velocity $\dot{\zeta}$ [mm/s]")
    ax.set_title("Push Response in Phase Space")
    ax.legend(loc="lower right")

    # Subplot 3: CBF Torque
    ax = axs[2]
    for m in modes:
        sub = df[df["mode"] == m]
        ax.plot(sub["t"], sub["tau_cbf"], label=m, color=colors.get(m, "k"))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"CBF Torque $\tau_{\mathrm{cbf}}$ [Nm]")
    ax.set_title(r"Torque Response under Operator Disturbance")
    ax.legend(loc="upper right")

    plt.tight_layout()
    out_path = out_dir / "fig3_torque_external_push.png"
    plt.savefig(out_path)
    plt.close()
    copy_to_artifact(out_path)

def plot_experiment_4(csv_dir: Path, out_dir: Path):
    csv_file = csv_dir / "torque_cbf_manipulator_7dof.csv"
    if not csv_file.is_file():
        return
    df = pd.read_csv(csv_file)
    row = df.iloc[0]

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))

    # Subplot 1: Bar chart of barrier values and bounds
    ax = axs[0]
    bars = ["Plane 1 (Floor)\n" + r"$B_1$", "Plane 1 (Floor)\n" + r"$b_{\mathrm{cbf}, 1}$",
            "Plane 2 (Wall)\n" + r"$B_2$", "Plane 2 (Wall)\n" + r"$b_{\mathrm{cbf}, 2}$"]
    vals = [row["dzeta1"] - row["v_min1"], row["b_cbf1"],
            row["dzeta2"] - row["v_min2"], row["b_cbf2"]]
    colors = ["#1f77b4", "#aec7e8", "#ff7f0e", "#ffbb78"]
    ax.bar(bars, vals, color=colors, edgecolor="k", width=0.55)
    ax.axhline(0, color="k", linewidth=1.0)
    ax.set_ylabel("Value [m/s or m/s$^2$]")
    ax.set_title("7-DOF Multi-Surface CBF Telemetry")

    # Subplot 2: Torque norm comparison
    ax = axs[1]
    t_labels = [r"CBF Torque $\|\tau_{\mathrm{cbf}}\|$", r"Total Commanded $\|\tau_{\mathrm{cmd}}\|$"]
    t_vals = [row["tau_cbf_norm"], row["tau_cmd_norm"]]
    ax.bar(t_labels, t_vals, color=["#2ca02c", "#9467bd"], edgecolor="k", width=0.45)
    for i, v in enumerate(t_vals):
        ax.text(i, v + 8, f"{v:.1f} Nm", ha="center", fontweight="bold")
    ax.set_ylabel("Torque Norm [Nm]")
    ax.set_title("7-DOF QP Optimal Joint Torques")
    ax.set_ylim(0, max(t_vals) * 1.2)

    plt.tight_layout()
    out_path = out_dir / "fig4_torque_manipulator_7dof.png"
    plt.savefig(out_path)
    plt.close()
    copy_to_artifact(out_path)

def plot_summary(csv_dir: Path, out_dir: Path):
    f1 = csv_dir / "torque_cbf_profiles.csv"
    f2 = csv_dir / "torque_cbf_entry_speeds.csv"
    f3 = csv_dir / "torque_cbf_external_push.csv"
    if not (f1.is_file() and f2.is_file() and f3.is_file()):
        return

    df1 = pd.read_csv(f1)
    df2 = pd.read_csv(f2)
    df3 = pd.read_csv(f3)

    fig, axs = plt.subplots(2, 2, figsize=(13, 9))

    # Top-Left: Profiles phase space
    ax = axs[0, 0]
    for p in df1["profile"].unique():
        sub = df1[df1["profile"] == p]
        ax.plot(sub["zeta"] * 1e3, sub["dzeta"] * 1e3, label=p)
    ax.set_xlabel(r"$\zeta$ [mm]")
    ax.set_ylabel(r"$\dot{\zeta}$ [mm/s]")
    ax.set_title("(a) Profiles $h_1..h_5$ in Phase Space")
    ax.legend(fontsize=8)

    # Top-Right: Entry speeds
    ax = axs[0, 1]
    for c in df2["case"].unique():
        sub = df2[df2["case"] == c]
        ax.plot(sub["t"], sub["B"], label=f"{c} entry")
    ax.axhline(0, color="k", linestyle=":")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Barrier $B(t)$ [m/s]")
    ax.set_title(r"(b) Safe-Set Invariance $B(t) \geq 0$")
    ax.legend(fontsize=8)

    # Bottom-Left: External push
    ax = axs[1, 0]
    for m in df3["mode"].unique():
        sub = df3[df3["mode"] == m]
        ax.plot(sub["t"], sub["tau_cbf"], label=m)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\tau_{\mathrm{cbf}}$ [Nm]")
    ax.set_title(r"(c) Downward Push ($F_{\mathrm{ext}}=-15$ N) Reaction")
    ax.legend(fontsize=8)

    # Bottom-Right: Profile torques
    ax = axs[1, 1]
    for p in df1["profile"].unique():
        sub = df1[df1["profile"] == p]
        ax.plot(sub["zeta"] * 1e3, sub["tau_cbf"], label=p)
    ax.set_xlabel(r"$\zeta$ [mm]")
    ax.set_ylabel(r"$\tau_{\mathrm{cbf}}$ [Nm]")
    ax.set_title(r"(d) Spatial Torque Distribution $\tau_{\mathrm{cbf}}(\zeta)$")
    ax.legend(fontsize=8)

    plt.tight_layout()
    out_path = out_dir / "cbf_torque_summary.png"
    plt.savefig(out_path)
    plt.close()
    copy_to_artifact(out_path)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv-dir", type=Path, default=Path("/home/galileo/projects/damping_cbf_ws/src/torque_cbf_controller/examples/csv"))
    parser.add_argument("--out-dir", type=Path, default=Path("/home/galileo/projects/damping_cbf_ws/src/torque_cbf_controller/examples/plots"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    set_style()

    print(f"Generating benchmark plots from CSVs in: {args.csv_dir}")
    plot_experiment_1(args.csv_dir, args.out_dir)
    plot_experiment_2(args.csv_dir, args.out_dir)
    plot_experiment_3(args.csv_dir, args.out_dir)
    plot_experiment_4(args.csv_dir, args.out_dir)
    plot_summary(args.csv_dir, args.out_dir)
    print(f"All benchmark figures saved to: {args.out_dir}")

if __name__ == "__main__":
    main()
