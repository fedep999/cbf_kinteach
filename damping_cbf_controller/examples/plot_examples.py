#!/usr/bin/env python3
"""Plot and analyze the simulation results produced by `example_damping_cbf`.

Generates five publication-quality figures:
  1. `fig1_profiles.png`         - Comparison of shaping profiles h_1..h_5 in phase
                                   space, spatial force/damping distribution, and time.
  2. `fig2_entry_speeds.png`     - Safe-set invariance (B > 0), boundary tracking (B = 0),
                                   and exponential recovery (B < 0) across entry speeds.
  3. `fig3_external_push.png`    - Effect of downward external force (F_ext = -15 N)
                                   with and without drift acceleration compensation (a_drift).
  4. `fig4_manipulator_7dof.png` - 7-DoF manipulator joint torques and 2-surface corner
                                   coupled CBF-QP trajectory & damping synthesis.
  5. `cbf_examples_summary.png`  - Combined 4x4 overview dashboard of all four experiments.

Usage:
  python3 examples/plot_examples.py [--csv-dir DIR] [--out-dir DIR] [--show]
"""

import argparse
import csv
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Dict, List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
# CSV Loading & Auto-Generation Helpers
# ---------------------------------------------------------------------------

def ensure_csv_files(csv_dir: Path) -> None:
    """Ensure the 4 simulation CSV files exist in `csv_dir`, running the binary if needed."""
    required = [
        "sim_profiles.csv",
        "sim_entry_speeds.csv",
        "sim_external_push.csv",
        "sim_manipulator_7dof.csv",
    ]
    if all((csv_dir / f).is_file() for f in required):
        return

    # Search common locations in the workspace for the compiled `example_damping_cbf` binary
    script_dir = Path(__file__).resolve().parent
    pkg_dir = script_dir.parent
    ws_dir = pkg_dir.parent.parent

    candidates = [
        ws_dir / "install" / "damping_cbf" / "bin" / "example_damping_cbf",
        ws_dir / "build" / "damping_cbf" / "example_damping_cbf",
        pkg_dir / "build" / "example_damping_cbf",
        Path.cwd() / "example_damping_cbf",
    ]

    binary = next((cand for cand in candidates if cand.is_file() and os.access(cand, os.X_OK)), None)
    if binary is None:
        binary_in_path = shutil.which("example_damping_cbf")
        if binary_in_path:
            binary = Path(binary_in_path)

    if binary is None:
        raise FileNotFoundError(
            f"Could not find CSV files in '{csv_dir}' and could not locate "
            "'example_damping_cbf' executable. Build the workspace with `colcon build` first."
        )

    print(f"[plot_examples] Generating CSV logs using {binary} -> {csv_dir} ...")
    csv_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(binary)], cwd=str(csv_dir), check=True)


def load_grouped_csv(csv_path: Path, group_col: str) -> Dict[str, Dict[str, np.ndarray]]:
    """Load a simulation CSV file and group numeric columns by `group_col`."""
    raw: Dict[str, Dict[str, List[float]]] = {}
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            key = row[group_col]
            if key not in raw:
                raw[key] = {k: [] for k in row.keys() if k != group_col}
            for k, val in row.items():
                if k != group_col:
                    raw[key][k].append(float(val))

    return {
        key: {col: np.asarray(vals, dtype=float) for col, vals in cols.items()}
        for key, cols in raw.items()
    }


def load_flat_csv(csv_path: Path) -> Dict[str, np.ndarray]:
    """Load a flat numeric CSV file without a grouping key column."""
    raw: Dict[str, List[float]] = {}
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, val in row.items():
                if k not in raw:
                    raw[k] = []
                raw[k].append(float(val))
    return {col: np.asarray(vals, dtype=float) for col, vals in raw.items()}


# ---------------------------------------------------------------------------
# Plot Styling
# ---------------------------------------------------------------------------

def configure_matplotlib_style() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linestyle": "--",
        "legend.fontsize": 8.5,
        "legend.framealpha": 0.9,
        "figure.dpi": 150,
        "savefig.dpi": 220,
        "savefig.bbox": "tight",
    })


PROFILE_STYLES = {
    "h1 (Constant)":    {"color": "#1f77b4", "ls": "-",  "lw": 2.0},
    "h2 (Root x^1/5)":  {"color": "#2ca02c", "ls": "--", "lw": 2.0},
    "h3 (Linear x)":    {"color": "#ff7f0e", "ls": "-.", "lw": 2.0},
    "h4 (Quintic x^5)": {"color": "#d62728", "ls": ":",  "lw": 2.2},
    "h5 (Smoothstep)":  {"color": "#9467bd", "ls": "-",  "lw": 2.6},
}

ENTRY_STYLES = {
    "Slow (B > 0, Inside Safe Zone)":  {"color": "#2ca02c", "ls": "-",  "lw": 2.3},
    "Nominal Border (B = 0)":          {"color": "#1f77b4", "ls": "--", "lw": 2.3},
    "Fast (B < 0, Outside Safe Zone)": {"color": "#d62728", "ls": "-",  "lw": 2.3},
}

PUSH_STYLES = {
    "Unforced Damping Only":         {"color": "#d62728", "ls": "--", "lw": 2.3},
    "With CBF Drift Term (a_drift)": {"color": "#1f77b4", "ls": "-",  "lw": 2.5},
}


# ---------------------------------------------------------------------------
# Figure 1: Profile Comparison (h_1 .. h_5)
# ---------------------------------------------------------------------------

def plot_experiment_1_profiles(
    data: Dict[str, Dict[str, np.ndarray]],
    axes: List[plt.Axes],
) -> None:
    """Populate 4 subplots for Experiment 1 (Profiles h_1..h_5)."""
    ax_phase, ax_force, ax_damp, ax_time = axes

    for name, d in data.items():
        st = PROFILE_STYLES.get(name, {"color": "black", "ls": "-", "lw": 1.8})
        zeta_mm = d["zeta"] * 1e3
        dzeta_mms = d["dzeta"] * 1e3
        f_abs = np.abs(d["f_imp"])
        d_cbf = d["d_cbf"]
        t = d["t"]

        ax_phase.plot(zeta_mm, dzeta_mms, label=name, **st)
        ax_force.plot(zeta_mm, f_abs, label=name, **st)
        ax_damp.plot(zeta_mm, d_cbf, label=name, **st)
        ax_time.plot(t, zeta_mm, label=name, **st)

    ax_phase.axhline(-2.0, color="k", ls=":", lw=1.2, label=r"$v_{\mathrm{ths}} = -2$ mm/s")
    ax_phase.axvline(0.0, color="k", lw=1.2)
    ax_phase.set_xlim(205, -5)
    ax_phase.set_xlabel(r"Distance to Surface $\zeta$ [mm] (Contact at 0)")
    ax_phase.set_ylabel(r"Normal Velocity $\dot{\zeta}$ [mm/s]")
    ax_phase.set_title(r"(1a) Phase Portrait: $\dot{\zeta}(\zeta)$ Envelope Shaping")
    ax_phase.legend(loc="upper left")

    ax_force.axvline(0.0, color="k", lw=1.2)
    ax_force.set_xlim(205, -5)
    ax_force.set_xlabel(r"Distance to Surface $\zeta$ [mm] (Contact at 0)")
    ax_force.set_ylabel(r"Braking Force $|f_{\mathrm{IMP}}|$ [N]")
    ax_force.set_title(r"(1b) Spatial Force Distribution $|f_{\mathrm{IMP}}(\zeta)| \propto h(u)$")
    ax_force.legend(loc="upper left")

    ax_damp.axvline(0.0, color="k", lw=1.2)
    ax_damp.set_xlim(205, -5)
    ax_damp.set_xlabel(r"Distance to Surface $\zeta$ [mm] (Contact at 0)")
    ax_damp.set_ylabel(r"Optimal Damping $d_{\mathrm{CBF}}$ [Ns/m]")
    ax_damp.set_title(r"(1c) QP Damping Gain $d_{\mathrm{CBF}}(\zeta)$")
    ax_damp.legend(loc="upper left")

    ax_time.axhline(0.0, color="k", lw=1.2, ls=":", label=r"Contact Surface ($\zeta = 0$)")
    ax_time.set_xlim(0.0, 2.5)
    ax_time.set_xlabel("Time $t$ [s]")
    ax_time.set_ylabel(r"Distance $\zeta(t)$ [mm]")
    ax_time.set_title(r"(1d) Approach Trajectory $\zeta(t)$ vs. Time")
    ax_time.legend(loc="upper right")


# ---------------------------------------------------------------------------
# Figure 2: Entry Velocities (B > 0, B = 0, B < 0)
# ---------------------------------------------------------------------------

def plot_experiment_2_entry_speeds(
    data: Dict[str, Dict[str, np.ndarray]],
    axes: List[plt.Axes],
) -> None:
    """Populate 4 subplots for Experiment 2 (Safe Set Invariance & Recovery)."""
    ax_phase, ax_barrier, ax_damp, ax_force = axes

    zeta_grid = np.linspace(0.0, 200.0, 300)
    u_grid = np.clip((200.0 - zeta_grid) / 200.0, 0.0, 1.0)
    H_u = -0.5 * (u_grid ** 4) + (u_grid ** 3)
    v_min_mms = -2.0 - (-2.0 - (-350.0)) * (1.0 - H_u / 0.5)

    ax_phase.plot(
        zeta_grid, v_min_mms, color="black", ls=":", lw=2.0,
        label=r"Barrier Boundary $v_{\min}(\zeta)$ ($B=0$)"
    )
    ax_phase.fill_between(
        zeta_grid, v_min_mms, 20.0, color="#2ca02c", alpha=0.10,
        label=r"Safe Set $\mathcal{C}$ ($B \geq 0$)"
    )
    ax_phase.fill_between(
        zeta_grid, -550.0, v_min_mms, color="#d62728", alpha=0.08,
        label=r"Unsafe Set ($B < 0$)"
    )

    for label, d in data.items():
        st = ENTRY_STYLES.get(label, {"color": "black", "ls": "-", "lw": 2.0})
        zeta_mm = d["zeta"] * 1e3
        dzeta_mms = d["dzeta"] * 1e3
        B_mms = d["B"] * 1e3
        t = d["t"]

        ax_phase.plot(zeta_mm, dzeta_mms, label=label, **st)
        ax_barrier.plot(t, B_mms, label=label, **st)
        ax_damp.plot(t, d["d_cbf"], label=label, **st)
        ax_force.plot(t, d["f_imp"], label=label, **st)

        if "Slow" in label:
            active_idx = np.where(d["d_cbf"] > 0.1)[0]
            if active_idx.size > 0:
                idx0 = int(active_idx[0])
                ax_phase.scatter(
                    [zeta_mm[idx0]], [dzeta_mms[idx0]],
                    color=st["color"], s=45, zorder=5
                )
                ax_phase.annotate(
                    f"Engages at\n$\\zeta = {zeta_mm[idx0]:.1f}$ mm",
                    xy=(zeta_mm[idx0], dzeta_mms[idx0]),
                    xytext=(zeta_mm[idx0] + 55, dzeta_mms[idx0] + 85),
                    arrowprops={"arrowstyle": "->", "color": st["color"], "lw": 1.3},
                    fontsize=8.5,
                    color=st["color"],
                    weight="bold",
                )

    ax_phase.set_xlim(205, -5)
    ax_phase.set_ylim(-530, 20)
    ax_phase.set_xlabel(r"Distance to Surface $\zeta$ [mm] (Contact at 0)")
    ax_phase.set_ylabel(r"Normal Velocity $\dot{\zeta}$ [mm/s]")
    ax_phase.set_title(r"(2a) Phase-Space Trajectories vs. Safe Set $\mathcal{C}$")
    ax_phase.legend(loc="lower right", fontsize=7.8)

    ax_barrier.axhline(0.0, color="k", ls=":", lw=1.5, label="Boundary $B = 0$")
    ax_barrier.set_xlim(0.0, 1.5)
    ax_barrier.set_xlabel("Time $t$ [s]")
    ax_barrier.set_ylabel(r"Barrier Function $B = \dot{\zeta} - v_{\min}(\zeta)$ [mm/s]")
    ax_barrier.set_title(r"(2b) Barrier Value $B(t)$: Invariance & Exponential Recovery")
    ax_barrier.legend(loc="lower right")

    ax_damp.set_xlim(0.0, 1.5)
    ax_damp.set_xlabel("Time $t$ [s]")
    ax_damp.set_ylabel(r"Optimal Damping $d_{\mathrm{CBF}}$ [Ns/m]")
    ax_damp.set_title(r"(2c) Synthesized Damping Gain $d_{\mathrm{CBF}}(t)$")
    ax_damp.legend(loc="upper right")

    ax_force.set_xlim(0.0, 1.5)
    ax_force.set_xlabel("Time $t$ [s]")
    ax_force.set_ylabel(r"Damping Force $f_{\mathrm{IMP}} = d_{\mathrm{CBF}}\dot{\zeta}$ [N]")
    ax_force.set_title(r"(2d) Damping Force $f_{\mathrm{IMP}}(t)$")
    ax_force.legend(loc="lower right")


# ---------------------------------------------------------------------------
# Figure 3: External Downward Push (F_ext,z = -15 N)
# ---------------------------------------------------------------------------

def plot_experiment_3_external_push(
    data: Dict[str, Dict[str, np.ndarray]],
    axes: List[plt.Axes],
) -> None:
    """Populate 4 subplots for Experiment 3 (External Push Compensation)."""
    ax_phase, ax_vel, ax_damp, ax_force = axes

    zeta_grid = np.linspace(0.0, 200.0, 300)
    u_grid = np.clip((200.0 - zeta_grid) / 200.0, 0.0, 1.0)
    H_u = -0.5 * (u_grid ** 4) + (u_grid ** 3)
    v_min_mms = -2.0 - (-2.0 - (-350.0)) * (1.0 - H_u / 0.5)
    ax_phase.plot(
        zeta_grid, v_min_mms, color="black", ls=":", lw=2.0,
        label=r"Nominal Envelope $v_{\min}(\zeta)$"
    )

    for mode, d in data.items():
        st = PUSH_STYLES.get(mode, {"color": "black", "ls": "-", "lw": 2.0})
        zeta_mm = d["zeta"] * 1e3
        dzeta_mms = d["dzeta"] * 1e3
        t = d["t"]

        pre_contact = d["zeta"] > 1e-4
        ax_phase.plot(zeta_mm[pre_contact], dzeta_mms[pre_contact], label=mode, **st)
        ax_vel.plot(t, dzeta_mms, label=mode, **st)
        ax_damp.plot(t, d["d_cbf"], label=mode, **st)
        ax_force.plot(t, d["f_imp"], label=mode, **st)

    ax_phase.axhline(
        -60.0, color="#1f77b4", ls="-.", lw=1.2, alpha=0.8,
        label=r"Saturated Limit $F_{\mathrm{ext}}/d_{\max} = -60$ mm/s"
    )
    ax_phase.set_xlim(205, -5)
    ax_phase.set_xlabel(r"Distance to Surface $\zeta$ [mm] (Contact at 0)")
    ax_phase.set_ylabel(r"Normal Velocity $\dot{\zeta}$ [mm/s]")
    ax_phase.set_title(r"(3a) Phase Portrait Under $-15\,$N Push")
    ax_phase.legend(loc="upper left", fontsize=8.0)

    ax_vel.axhline(
        -60.0, color="#1f77b4", ls="-.", lw=1.2, alpha=0.8,
        label=r"$F_{\mathrm{ext}}/d_{\max} = -60$ mm/s"
    )
    ax_vel.set_xlim(0.0, 2.0)
    ax_vel.set_xlabel("Time $t$ [s]")
    ax_vel.set_ylabel(r"Normal Velocity $\dot{\zeta}(t)$ [mm/s]")
    ax_vel.set_title(r"(3b) Normal Velocity $\dot{\zeta}(t)$ ($-220$ vs. $-60$ mm/s Impact)")
    ax_vel.legend(loc="upper right", fontsize=8.0)

    ax_damp.axhline(250.0, color="k", ls=":", lw=1.4, label=r"$d_{\max} = 250$ Ns/m")
    ax_damp.set_xlim(0.0, 2.0)
    ax_damp.set_ylim(-5, 270)
    ax_damp.set_xlabel("Time $t$ [s]")
    ax_damp.set_ylabel(r"Optimal Damping $d_{\mathrm{CBF}}$ [Ns/m]")
    ax_damp.set_title(r"(3c) Damping Gain $d_{\mathrm{CBF}}(t)$ & Saturation at $d_{\max}$")
    ax_damp.legend(loc="center right")

    ax_force.axhline(-15.0, color="k", ls=":", lw=1.4, label=r"External Push $F_{\mathrm{ext},z} = -15$ N")
    ax_force.set_xlim(0.0, 2.0)
    ax_force.set_xlabel("Time $t$ [s]")
    ax_force.set_ylabel(r"Damping Force $f_{\mathrm{IMP}}(t)$ [N]")
    ax_force.set_title(r"(3d) Braking Force $f_{\mathrm{IMP}}(t)$ Counteracting $F_{\mathrm{ext},z}$")
    ax_force.legend(loc="lower right")


# ---------------------------------------------------------------------------
# Figure 4: 7-DoF Manipulator Compliance Torque & 2-Surface Corner QP
# ---------------------------------------------------------------------------

def plot_experiment_4_manipulator_7dof(
    d: Dict[str, np.ndarray],
    axes: List[plt.Axes],
) -> None:
    """Populate 4 subplots for Experiment 4 (7-DoF Torque & 2-Surface Corner QP)."""
    ax_corner, ax_vel, ax_damp, ax_torque = axes
    t = d["t"]
    x_mm = d["x"] * 1e3
    z_mm = d["z"] * 1e3

    # --- Subplot (4a): Multi-Surface Control Barrier Function Values B_i(t) (Active Filter Only) ---
    p_ths_4 = 200.0  # mm
    v_ths_4 = -2.0  # mm/s
    zeta1_4 = d["zeta1"] * 1e3
    zeta2_4 = d["zeta2"] * 1e3
    dzeta1_4 = d["dzeta1"] * 1e3
    dzeta2_4 = d["dzeta2"] * 1e3

    act1_4 = (zeta1_4 <= p_ths_4) & (zeta1_4 >= 0.0) & (dzeta1_4 < v_ths_4)
    act2_4 = (zeta2_4 <= p_ths_4) & (zeta2_4 >= 0.0) & (dzeta2_4 < v_ths_4)

    B1 = np.where(act1_4, (d["dzeta1"] - d["v_min1"]) * 1e3, np.nan)
    B2 = np.where(act2_4, (d["dzeta2"] - d["v_min2"]) * 1e3, np.nan)

    ax_corner.axhline(0.0, color="black", ls="--", lw=1.5, zorder=4, label=r"Safety Boundary $B = 0$")

    valid_B4 = []
    if np.any(act1_4):
        valid_B4.extend((d["dzeta1"][act1_4] - d["v_min1"][act1_4]) * 1e3)
    if np.any(act2_4):
        valid_B4.extend((d["dzeta2"][act2_4] - d["v_min2"][act2_4]) * 1e3)

    y_max = max(max(valid_B4) * 1.15, 50.0) if len(valid_B4) > 0 else 50.0
    y_min = min(min(valid_B4) * 1.15, -10.0) if len(valid_B4) > 0 else -10.0
    ax_corner.fill_between(t, 0.0, y_max, color="#2ca02c", alpha=0.08, label=r"Safe Set $\mathcal{C}$ ($B \geq 0$)")

    ax_corner.plot(t, B1, color="#1f77b4", lw=2.3, label=r"Floor Barrier $B_1(t)$ (Active Filter)")
    ax_corner.plot(t, B2, color="#ff7f0e", lw=2.3, label=r"$45^\circ$ Wall Barrier $B_2(t)$ (Active Filter)")

    ax_corner.set_xlim(0.0, 1.5)
    ax_corner.set_ylim(y_min, y_max)
    ax_corner.set_xlabel("Time $t$ [s]")
    ax_corner.set_ylabel(r"Barrier Function $B_i(t)$ [mm/s]")
    ax_corner.set_title(r"(4a) CBF Values $B_i(t)$ (Active Filter Intervals Only)")
    ax_corner.grid(True, linestyle="--", alpha=0.5)
    ax_corner.legend(loc="upper right", fontsize=8.0)

    # --- Subplot (4b): Normal Velocities vs. Safe Envelopes (Inside Braking Zone) ---
    vmin1_4 = np.where(zeta1_4 <= p_ths_4, d["v_min1"] * 1e3, np.nan)
    vmin2_4 = np.where(zeta2_4 <= p_ths_4, d["v_min2"] * 1e3, np.nan)

    ax_vel.plot(t, dzeta1_4, color="#1f77b4", lw=2.3, label=r"Floor Normal Vel $\dot{\zeta}_1(t)$")
    ax_vel.plot(t, vmin1_4, color="#1f77b4", ls=":", lw=1.8, label=r"Floor Envelope $v_{\min}(\zeta_1)$")
    ax_vel.plot(t, dzeta2_4, color="#ff7f0e", lw=2.3, label=r"$45^\circ$ Wall Normal Vel $\dot{\zeta}_2(t)$")
    ax_vel.plot(t, vmin2_4, color="#ff7f0e", ls=":", lw=1.8, label=r"$45^\circ$ Wall Envelope $v_{\min}(\zeta_2)$")
    ax_vel.set_xlim(0.0, 1.5)
    ax_vel.set_xlabel("Time $t$ [s]")
    ax_vel.set_ylabel(r"Normal Velocity $\dot{\zeta}_i$ [mm/s]")
    ax_vel.set_title(r"(4b) Multi-Surface Normal Velocities vs. Safe Bounds $v_{\min}(\zeta_i)$")
    ax_vel.legend(loc="upper right", fontsize=8.0)

    # --- Subplot (4c): Coupled 2-Surface QP Damping Gains d_1*, d_2* & trace(D_IMP) ---
    ax_damp.plot(t, d["d1_multi"], color="#1f77b4", lw=2.3, label=r"$d_1^*(t)$ (Floor Normal $n_1$)")
    ax_damp.plot(t, d["d2_multi"], color="#ff7f0e", lw=2.3, label=r"$d_2^*(t)$ ($45^\circ$ Wall Normal $n_2$)")
    ax_damp.plot(t, d["trace_D_imp"], color="#9467bd", ls="--", lw=2.2, label=r"$\mathrm{trace}(D_{\mathrm{IMP}}) = d_1^* + d_2^*$")
    ax_damp.axhline(250.0, color="k", ls=":", lw=1.3, label=r"Per-Surface Bound $d_{\max} = 250$ Ns/m")
    ax_damp.set_xlim(0.0, 1.5)
    ax_damp.set_xlabel("Time $t$ [s]")
    ax_damp.set_ylabel("Damping Coefficient [Ns/m]")
    ax_damp.set_title(r"(4c) Coupled 2-Surface QP Solution $d^* = [d_1^*, d_2^*]^T$")
    ax_damp.legend(loc="center right", fontsize=8.0)

    # --- Subplot (4d): Surface Distances zeta_1(t), zeta_2(t) ---
    ax_dist = ax_torque
    p_ths_4 = 200.0  # mm (p_ths = 0.20 m in Exp 4)
    zeta1_4 = d["zeta1"] * 1e3
    zeta2_4 = d["zeta2"] * 1e3

    ax_dist.axhline(0.0, color="black", lw=1.8, zorder=4, label=r"Contact Surface ($\zeta = 0$)")
    ax_dist.axhline(p_ths_4, color="gray", ls="--", lw=1.4, zorder=3, label=r"Braking Boundary ($p_{\mathrm{ths}} = 200$ mm)")
    ax_dist.fill_between(t, 0.0, p_ths_4, color="gray", alpha=0.10, label="Active CBF Braking Zone")

    ax_dist.plot(t, zeta1_4, color="#1f77b4", lw=2.3, label=r"Floor Distance $\zeta_1(t)$")
    ax_dist.plot(t, zeta2_4, color="#ff7f0e", lw=2.3, label=r"$45^\circ$ Wall Distance $\zeta_2(t)$")

    ax_dist.set_xlim(0.0, 1.5)
    ax_dist.set_ylim(-10, max(zeta1_4.max(), zeta2_4.max()) * 1.10)
    ax_dist.set_xlabel("Time $t$ [s]")
    ax_dist.set_ylabel(r"Distance to Surface $\zeta_i$ [mm]")
    ax_dist.set_title(r"(4d) End-Effector Surface Distances $\zeta_i(t)$")
    ax_dist.grid(True, linestyle="--", alpha=0.5)
    ax_dist.legend(loc="upper right", fontsize=7.8)


# ---------------------------------------------------------------------------
# Main Entry Point
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot simulation results from example_damping_cbf."
    )
    script_dir = Path(__file__).resolve().parent
    parser.add_argument(
        "--csv-dir",
        type=Path,
        default=script_dir / "csv",
        help="Directory containing sim_*.csv files (default: examples/csv, will auto-run example_damping_cbf if missing).",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=script_dir / "plots",
        help="Output directory for generated PNG/PDF figures (default: examples/plots).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display interactive matplotlib windows after saving.",
    )
    args = parser.parse_args()

    if not args.show and not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")

    configure_matplotlib_style()
    ensure_csv_files(args.csv_dir)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    profiles_data = load_grouped_csv(args.csv_dir / "sim_profiles.csv", "profile")
    entry_data = load_grouped_csv(args.csv_dir / "sim_entry_speeds.csv", "case")
    push_data = load_grouped_csv(args.csv_dir / "sim_external_push.csv", "mode")
    manip_data = load_flat_csv(args.csv_dir / "sim_manipulator_7dof.csv")

    # 1. Standalone Figure 1: Profiles h_1..h_5
    fig1, axes1 = plt.subplots(2, 2, figsize=(13, 8.5))
    fig1.suptitle(
        "Experiment 1: Shaping Profiles $h_1(u) \\dots h_5(u)$ "
        "($\\dot{\\zeta}_0 = -0.35\\,\\mathrm{m/s},\\; v_{\\mathrm{ths}} = -2\\,\\mathrm{mm/s},\\; F_{\\mathrm{ext}} = 0$)",
        fontsize=13, fontweight="bold", y=0.99,
    )
    plot_experiment_1_profiles(profiles_data, list(axes1.flat))
    fig1.tight_layout(rect=[0, 0, 1, 0.96])
    fig1_path = args.out_dir / "fig1_profiles.png"
    fig1.savefig(fig1_path)
    print(f"[plot_examples] Saved {fig1_path}")

    # 2. Standalone Figure 2: Entry Velocities (B > 0, B = 0, B < 0)
    fig2, axes2 = plt.subplots(2, 2, figsize=(13, 8.5))
    fig2.suptitle(
        "Experiment 2: Phase-Space Safe Set Invariance ($B > 0$), Boundary Tracking ($B = 0$), "
        "and Recovery ($B < 0$)",
        fontsize=13, fontweight="bold", y=0.99,
    )
    plot_experiment_2_entry_speeds(entry_data, list(axes2.flat))
    fig2.tight_layout(rect=[0, 0, 1, 0.96])
    fig2_path = args.out_dir / "fig2_entry_speeds.png"
    fig2.savefig(fig2_path)
    print(f"[plot_examples] Saved {fig2_path}")

    # 3. Standalone Figure 3: External Operator Push (F_ext = -15 N)
    fig3, axes3 = plt.subplots(2, 2, figsize=(13, 8.5))
    fig3.suptitle(
        "Experiment 3: Downward External Push ($F_{\\mathrm{ext},z} = -15\\,\\mathrm{N}$) — "
        "Unforced Damping vs. CBF Drift Compensation ($a_{\\mathrm{drift}}$)",
        fontsize=13, fontweight="bold", y=0.99,
    )
    plot_experiment_3_external_push(push_data, list(axes3.flat))
    fig3.tight_layout(rect=[0, 0, 1, 0.96])
    fig3_path = args.out_dir / "fig3_external_push.png"
    fig3.savefig(fig3_path)
    print(f"[plot_examples] Saved {fig3_path}")

    # 4. Standalone Figure 4: 7-DoF Manipulator Torque & 2-Surface Corner QP
    fig4, axes4 = plt.subplots(2, 2, figsize=(13, 8.5))
    fig4.suptitle(
        "Experiment 4: 7-DoF Manipulator Compliance Control Torque & Coupled 2-Surface Corner QP",
        fontsize=13, fontweight="bold", y=0.99,
    )
    plot_experiment_4_manipulator_7dof(manip_data, list(axes4.flat))
    fig4.tight_layout(rect=[0, 0, 1, 0.96])
    fig4_path = args.out_dir / "fig4_manipulator_7dof.png"
    fig4.savefig(fig4_path)
    print(f"[plot_examples] Saved {fig4_path}")

    # 5. Combined 4x4 Master Summary Dashboard
    fig_all, axes_all = plt.subplots(4, 4, figsize=(20, 18.0))
    fig_all.suptitle(
        "CBF Variable-Damping Cartesian Compliance Controller — Verification Suite Summary",
        fontsize=15, fontweight="bold", y=0.995,
    )
    plot_experiment_1_profiles(profiles_data, list(axes_all[0, :]))
    plot_experiment_2_entry_speeds(entry_data, list(axes_all[1, :]))
    plot_experiment_3_external_push(push_data, list(axes_all[2, :]))
    plot_experiment_4_manipulator_7dof(manip_data, list(axes_all[3, :]))
    fig_all.tight_layout(rect=[0, 0, 1, 0.98])

    summary_png = args.out_dir / "cbf_examples_summary.png"
    summary_pdf = args.out_dir / "cbf_examples_summary.pdf"
    fig_all.savefig(summary_png)
    fig_all.savefig(summary_pdf)
    print(f"[plot_examples] Saved {summary_png}")
    print(f"[plot_examples] Saved {summary_pdf}")

    if args.show:
        plt.show()
    else:
        plt.close("all")

    return 0


if __name__ == "__main__":
    sys.exit(main())
