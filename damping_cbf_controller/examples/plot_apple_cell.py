#!/usr/bin/env python3
"""Generate publication-grade 4-subplot analysis figure (Figure 5) for the
UR10e Apple Picking Cell simulation with Pinocchio dynamics and C++ CBF control.

This script purely handles data loading and figure generation from the simulation
trajectory CSV (e.g. `sim_apple_cell_trajectory.csv`), maintaining a clean
separation between C++ physics/control simulation and Python data visualization.
All controller thresholds and styling are read from the shared YAML configuration.

Subplots Generated:
  (5a) Multi-Surface Control Barrier Function Values B_i(t) (Active filter only)
  (5b) Normal Velocities vs. Safe Envelopes v_min(zeta_i) (Inside braking zone)
  (5c) Synthesized C++ CBF Damping Coefficients d_1*(t), d_2*(t) & trace(D_IMP)
  (5d) End-Effector Surface Distances zeta_i(t) relative to p_ths and contact plane

Usage:
  python3 examples/plot_apple_cell.py [--config <path>] [--csv <path>] [--out <path>] [--show]
"""

import argparse
from pathlib import Path
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


def set_matplotlib_style():
    """Apply consistent publication-grade matplotlib style."""
    matplotlib.rcParams.update({
        "font.family": "serif",
        "font.size": 9.5,
        "axes.labelsize": 10.5,
        "axes.titlesize": 11.0,
        "xtick.labelsize": 9.0,
        "ytick.labelsize": 9.0,
        "lines.linewidth": 1.8,
        "axes.grid": True,
        "grid.alpha": 0.35,
        "grid.linestyle": "--",
        "legend.fontsize": 8.0,
        "legend.framealpha": 0.90,
        "figure.dpi": 150,
        "savefig.dpi": 200,
        "savefig.bbox": "tight",
    })


def find_default_config() -> Path:
    """Locate the default apple_cell_cbf.yaml configuration file."""
    candidates = [
        Path(__file__).resolve().parent.parent / "config" / "apple_cell_cbf.yaml",
        Path("/home/galileo/projects/damping_cbf_ws/src/damping_cbf_controller/config/apple_cell_cbf.yaml"),
        Path("/home/galileo/projects/damping_cbf_ws/install/damping_cbf/share/damping_cbf/config/apple_cell_cbf.yaml"),
        Path("config/apple_cell_cbf.yaml"),
    ]
    for c in candidates:
        if c.is_file():
            return c
    return candidates[0]


def load_config(config_path: Path) -> dict:
    """Load configuration from YAML file."""
    if config_path and config_path.is_file():
        try:
            with open(config_path, "r") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            print(f"Warning: Failed to load YAML config {config_path}: {e}")
    return {}


def load_trajectory_data(csv_path: Path) -> pd.DataFrame:
    """Load and validate simulation trajectory CSV."""
    if not csv_path.is_file():
        raise FileNotFoundError(
            f"Trajectory CSV file not found: {csv_path}\n"
            f"Please run the C++ simulation first: ./install/damping_cbf/bin/simulate_apple_cell"
        )
    df = pd.read_csv(csv_path)
    required_cols = [
        "t", "x", "y", "z", "dx", "dy", "dz",
        "zeta1", "dzeta1", "v_min1", "B1", "act1",
        "zeta2", "dzeta2", "v_min2", "B2", "act2",
        "d1", "d2", "trace_D"
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"CSV file {csv_path} is missing required columns: {missing}")
    return df


def plot_apple_cell_analysis(df: pd.DataFrame, out_plot: Path, config: dict, show_plot: bool = False):
    """Render and save the 4-subplot analysis figure using parameters from config."""
    set_matplotlib_style()

    t_arr = df["t"].to_numpy()
    duration = t_arr[-1]

    # Convert distance and velocity quantities to mm and mm/s
    zeta1_arr = df["zeta1"].to_numpy() * 1e3
    dzeta1_arr = df["dzeta1"].to_numpy() * 1e3
    vmin1_arr = df["v_min1"].to_numpy() * 1e3
    B1_arr = df["B1"].to_numpy() * 1e3
    act1_mask = df["act1"].to_numpy().astype(bool)

    zeta2_arr = df["zeta2"].to_numpy() * 1e3
    dzeta2_arr = df["dzeta2"].to_numpy() * 1e3
    vmin2_arr = df["v_min2"].to_numpy() * 1e3
    B2_arr = df["B2"].to_numpy() * 1e3
    act2_mask = df["act2"].to_numpy().astype(bool)

    d1_arr = df["d1"].to_numpy()
    d2_arr = df["d2"].to_numpy()
    trace_D_arr = df["trace_D"].to_numpy()

    # Read controller safety parameters from YAML config (with fallbacks)
    cbf_cfg = config.get("cbf", {})
    p_ths = float(cbf_cfg.get("p_ths", 0.35))
    d_max = float(cbf_cfg.get("d_max", 50.0))

    planes_cfg = config.get("planes", {})
    plane1_name = planes_cfg.get("plane1", {}).get("name", "Table Surface")
    plane2_name = planes_cfg.get("plane2", {}).get("name", "45° Wall")

    fig, axes = plt.subplots(2, 2, figsize=(14, 9.0))
    fig.suptitle(
        "UR10e Apple Picking Cell — Pinocchio Dynamics & C++ Multi-Surface CBF Control",
        fontsize=14, fontweight="bold", y=0.99
    )

    # -----------------------------------------------------------------------
    # (5a) Multi-Surface Control Barrier Function Values B_i(t) (Active Filter Only)
    # -----------------------------------------------------------------------
    ax_cbf = axes[0, 0]
    ax_cbf.axhline(0.0, color="black", ls="--", lw=1.5, zorder=4, label=r"Safety Threshold $B = 0$")

    B1_active = np.where(act1_mask, B1_arr, np.nan)
    B2_active = np.where(act2_mask, B2_arr, np.nan)

    valid_B = []
    if np.any(act1_mask):
        valid_B.extend(B1_arr[act1_mask])
    if np.any(act2_mask):
        valid_B.extend(B2_arr[act2_mask])

    y_max = max(max(valid_B) * 1.15, 30.0) if len(valid_B) > 0 else 50.0
    y_min = min(min(valid_B) * 1.15, -10.0) if len(valid_B) > 0 else -10.0

    ax_cbf.fill_between(t_arr, 0.0, y_max, color="#2ca02c", alpha=0.08, label=r"Safe Domain $\mathcal{C}$ ($B \geq 0$)")
    ax_cbf.plot(t_arr, B1_active, color="#1f77b4", lw=2.5, label=f"{plane1_name} CBF $B_1(t)$ (Active Filter)")
    ax_cbf.plot(t_arr, B2_active, color="#ff7f0e", lw=2.5, label=f"{plane2_name} CBF $B_2(t)$ (Active Filter)")
    ax_cbf.set_xlim(0.0, duration)
    ax_cbf.set_ylim(y_min, y_max)
    ax_cbf.set_xlabel("Time $t$ [s]")
    ax_cbf.set_ylabel(r"Barrier Function $B_i(t)$ [mm/s]")
    ax_cbf.set_title(r"(5a) CBF Values $B_i(t)$ (Active Filter Intervals Only)")
    ax_cbf.grid(True, linestyle="--", alpha=0.5)
    ax_cbf.legend(loc="upper right")

    # -----------------------------------------------------------------------
    # (5b) Multi-Surface Normal Velocities vs. Safe Envelopes (Inside Braking Zone)
    # -----------------------------------------------------------------------
    vmin1_active = np.where(zeta1_arr <= p_ths * 1e3, vmin1_arr, np.nan)
    vmin2_active = np.where(zeta2_arr <= p_ths * 1e3, vmin2_arr, np.nan)

    ax_vel = axes[0, 1]
    ax_vel.plot(t_arr, dzeta1_arr, color="#1f77b4", lw=2.2, label=f"{plane1_name} Normal Vel $\\dot{{\\zeta}}_1(t)$")
    ax_vel.plot(t_arr, vmin1_active, color="#1f77b4", ls=":", lw=1.8, label=f"{plane1_name} Envelope $v_{{\\min}}(\\zeta_1)$")
    ax_vel.plot(t_arr, dzeta2_arr, color="#ff7f0e", lw=2.2, label=f"{plane2_name} Normal Vel $\\dot{{\\zeta}}_2(t)$")
    ax_vel.plot(t_arr, vmin2_active, color="#ff7f0e", ls=":", lw=1.8, label=f"{plane2_name} Envelope $v_{{\\min}}(\\zeta_2)$")
    ax_vel.set_xlim(0.0, duration)
    ax_vel.set_xlabel("Time $t$ [s]")
    ax_vel.set_ylabel(r"Normal Velocity $\dot{\zeta}_i$ [mm/s]")
    ax_vel.set_title(r"(5b) Pinocchio Velocities vs. Safe Envelopes $v_{\min}(\zeta_i)$")
    ax_vel.legend(loc="upper right")

    # -----------------------------------------------------------------------
    # (5c) Coupled 2-Surface QP Damping Gains
    # -----------------------------------------------------------------------
    ax_damp = axes[1, 0]
    ax_damp.plot(t_arr, d1_arr, color="#1f77b4", lw=2.2, label=f"$d_1^*(t)$ ({plane1_name})")
    ax_damp.plot(t_arr, d2_arr, color="#ff7f0e", lw=2.2, label=f"$d_2^*(t)$ ({plane2_name})")
    ax_damp.plot(t_arr, trace_D_arr, color="#9467bd", ls="--", lw=2.0, label=r"$\mathrm{trace}(D_{\mathrm{IMP}}) = d_1^* + d_2^*$")
    ax_damp.axhline(d_max, color="k", ls=":", lw=1.2, label=f"$d_{{\\max}} = {d_max:.1f}$ Ns/m")
    ax_damp.set_xlim(0.0, duration)
    ax_damp.set_xlabel("Time $t$ [s]")
    ax_damp.set_ylabel("Damping Gain [Ns/m]")
    ax_damp.set_title("(5c) Synthesized C++ CBF Damping Coefficients")
    ax_damp.legend(loc="upper right")

    # -----------------------------------------------------------------------
    # (5d) Surface Distances zeta_1(t), zeta_2(t)
    # -----------------------------------------------------------------------
    ax_dist = axes[1, 1]
    ax_dist.axhline(0.0, color="black", lw=1.8, zorder=4, label=r"Contact Surface ($\zeta = 0$)")
    ax_dist.axhline(p_ths * 1e3, color="gray", ls="--", lw=1.4, zorder=3, label=rf"Braking Boundary ($p_{{\mathrm{{ths}}}} = {int(p_ths*1e3)}$ mm)")
    ax_dist.fill_between(t_arr, 0.0, p_ths * 1e3, color="gray", alpha=0.10, label="Active CBF Braking Zone")

    ax_dist.plot(t_arr, zeta1_arr, color="#1f77b4", lw=2.3, label=f"{plane1_name} Distance $\\zeta_1(t)$")
    ax_dist.plot(t_arr, zeta2_arr, color="#ff7f0e", lw=2.3, label=f"{plane2_name} Distance $\\zeta_2(t)$")

    ax_dist.set_xlim(0.0, duration)
    ax_dist.set_ylim(-10, max(zeta1_arr.max(), zeta2_arr.max()) * 1.08)
    ax_dist.set_xlabel("Time $t$ [s]")
    ax_dist.set_ylabel(r"Distance to Surface $\zeta_i$ [mm]")
    ax_dist.set_title(r"(5d) End-Effector Surface Distances $\zeta_i(t)$")
    ax_dist.grid(True, linestyle="--", alpha=0.5)
    ax_dist.legend(loc="upper right")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out_plot.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_plot, dpi=200)
    print(f"Saved analysis plot: {out_plot}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def main():
    default_config_path = find_default_config()

    parser = argparse.ArgumentParser(
        description="Generate Figure 5 Analysis Graphs from Apple Cell Simulation Trajectory using YAML configuration"
    )
    parser.add_argument(
        "--config", type=Path, default=default_config_path,
        help=f"Path to YAML configuration file (default: {default_config_path})"
    )
    parser.add_argument(
        "--csv", type=Path, default=None,
        help="Path to input simulation CSV (overrides YAML config)"
    )
    parser.add_argument(
        "--out", type=Path, default=None,
        help="Path to output PNG plot (overrides YAML config)"
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display interactive matplotlib window"
    )
    args = parser.parse_args()

    config = load_config(args.config)
    if config:
        print(f"Loaded configuration from: {args.config}")

    pkg_root = Path(__file__).resolve().parent.parent

    # Resolve CSV path
    if args.csv is not None:
        csv_path = args.csv
    else:
        csv_rel = config.get("paths", {}).get("csv_output", "examples/csv/sim_apple_cell_trajectory.csv")
        csv_path = Path(csv_rel) if Path(csv_rel).is_absolute() else pkg_root / csv_rel

    # Resolve Plot output path
    if args.out is not None:
        out_plot = args.out
    else:
        plot_rel = config.get("paths", {}).get("plot_output", "examples/plots/fig5_apple_cell_pinocchio.png")
        out_plot = Path(plot_rel) if Path(plot_rel).is_absolute() else pkg_root / plot_rel

    df = load_trajectory_data(csv_path)
    print(f"Loaded {len(df)} simulation points from {csv_path}")
    plot_apple_cell_analysis(df, out_plot, config, show_plot=args.show)


if __name__ == "__main__":
    main()
