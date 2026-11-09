# CBF controllers: changes and signal flow

As of 9 Nov 2026. Covers the torque CBF (`torque_cbf_simulink/Controller_trorque_cbf_22Franka.slx`, saved 11:05) and the damping CBF (`damping_cbf_simulink/Controller_damping_cbf_22Franka.slx`, saved 10:48).

Along the plane normal, both controllers now leave the motion to the human and the CBF alone. The lateral spring-damper is dynamically decoupled from the plane normal, and each CBF is given a model of the normal dynamics that matches what the robot actually does. Nothing is committed to git yet.

Contents:
1. [Torque CBF: how it is fed](#1-torque-cbf-how-it-is-fed)
2. [Torque CBF: what changed](#2-torque-cbf-what-changed)
3. [Damping CBF: how it is fed](#3-damping-cbf-how-it-is-fed)
4. [Damping CBF: what changed](#4-damping-cbf-what-changed)
5. [Shared code and real-time build](#5-shared-code-and-real-time-build)
6. [Open items before the next robot run](#6-open-items-before-the-next-robot-run)
7. [How to re-apply and rebuild](#7-how-to-re-apply-and-rebuild)

Notation:

| Symbol | Meaning |
|---|---|
| n | unit plane normal (3D) |
| n6 | [n; 0; 0; 0] |
| ζ | distance from the flange to the plane |
| ζ̇ | normal velocity |
| M, C, J, J̇ | analytic Panda model (Maple, bare arm without the hand) |
| Λ⁻¹ | J·M⁻¹·Jᵀ |
| ρ | effective mass along the normal, 1/(n6ᵀΛ⁻¹n6) |
| Ṽ | end-effector twist J·q̇ (the reference velocity is 0) |

---

## 1. Torque CBF: how it is fed

The S-function `sfun_torque_cbf` (block `S-Function Torque CBF` in `task space controller`) receives everything it needs to predict the normal acceleration it will cause. It returns the joint torque `tau_cbf`, which is added to the command.

### Inputs

| Port | Signal | Source in the model | Notes |
|---|---|---|---|
| 1 | q | From `q` | joint positions |
| 2 | q̇ | From `q_dot` | joint velocities |
| 3 | x_ee | `Extract_EE_Kinematics`: H_IT(1:3,4) | flange (`panda_link8`) position |
| 4 | v_ee | `Extract_EE_Kinematics`: Jg(1:3,:)·q̇ | flange linear velocity |
| 5 | M | From `M_mat` | analytic, checked against the URDF to 1e-11 |
| 6 | C | From `C_mat` | Coriolis matrix |
| 7 | J | From `Jg_mat` | 6×7, linear rows first |
| 8 | J̇ | From `Jg_dot_mat` | |
| 9 | τ_nom | `Sum_tau_nom_cbf` = τ_null + τ_stiff | every torque summed into the command besides τ_cbf |
| 10 | g | From `g_vec` | used **only** for the total-torque limit, never added to the command |
| 11 | F_ext | From `F_ext_full` (your block `Fext_full_flange`) | −`O_F_ext_hat_K`, filtered by `FIR_Fext_full` (causal SG, N = 21), moved from the K frame to the flange with `cbf_K_offset`; full 6D wrench on the robot |
| 12 | use_fext | `Constant_use_fext` = 1 | F_ext is compensated |
| 13 | params | `cbf_torque_params` (24×1) | see the layout below |

Parameter vector: `[p_ths, v_ths, v_0, gamma, w_lin, w_slack, tau_limit, dt, n(3), plane offset(3), latch_entry, zone_hyst, latch_alpha, tau_max_1..7]`. The per-joint limits τ_max replace the old scalar `tau_limit`, which is kept only as a fallback.

### What the CBF computes

The predicted end-effector acceleration without CBF force:

$$a_{drift} = \dot J \dot q - J M^{-1} C \dot q + J M^{-1}\tau_{nom} + \Lambda^{-1} F_{ext}$$

- The τ_null part contributes nothing, because the null-space projector is dynamically consistent.
- With `cbf_stiff_decouple = 1`, the τ_stiff part has zero normal component (section 2).

Barrier, envelope v_min(ζ) with the smoothstep profile, active for −0.05 ≤ ζ ≤ p_ths:

$$B = \dot\zeta - v_{min}(\zeta)$$

QP in the normal force f and the slack δ:

$$\min_{f,\delta}\ \tfrac12 f^2 + \tfrac12 w_{slack}\delta^2 + w_{lin}\delta$$

$$\text{s.t.}\quad \frac{f}{\rho} + \delta \ \ge\ v_{min}'(\zeta)\,\dot\zeta - n_6^\top a_{drift} - \gamma B$$

$$0 \le f \le f_{cap},\qquad f_{cap} = \begin{cases}\rho\left(\dfrac{|\dot\zeta|}{dt} + \max(0,\,-n_6^\top a_{drift})\right) & \dot\zeta \le 0\\[4pt] 0 & \dot\zeta > 0\end{cases}$$

$$-\tau_{max} \le g + \tau_{nom} + J^\top n_6 f \le \tau_{max}\quad\text{(bounds on } J^\top n_6 f \text{ relaxed so that } f = 0 \text{ always stays feasible)}$$

Output:

$$\tau_{cbf} = J^\top n_6\, f$$

### Where the torque goes

`Sum`: τ_c = τ_null + 0 + τ_cbf + τ_stiff → root `Rate Limiter` (±999 Nm/s) → `Apply Control`. Franka adds gravity itself.

Telemetry (output 2, 8 values): is_active, ζ, ζ̇, v_min, B, f_n, slack, QP cost. These go to `CBF_Telemetry_Visualizer1` and `CBF_Logger`.

---

## 2. Torque CBF: what changed

### C++ library (`torque_cbf_controller`)

| Change | Why |
|---|---|
| Pinocchio overload of `computeControlTorque` now passes `data.g` to the solver | the total-torque limit ignored gravity in that overload |
| Only planes inside the zone become QP rows; out-of-zone planes no longer get a dummy row `b = −1e3` | with several planes and coupling in Λ⁻¹, the dummy row could still constrain the solution |
| `enforce_total_torque_limits` bounds g + τ_nom + τ_cbf, with the bounds on τ_cbf relaxed to contain 0 | real joint limits on the total torque; if g + τ_nom alone exceeds a limit, the CBF can't make it worse and the QP never fails |
| Braking cap: new drift term max(0, −n·a_drift)·ρ, and the condition is now ζ̇ ≤ 0 (was < 0); flag `brake_cap_include_drift`, default true | lets the CBF hold the robot at `v_ths` or at rest under a push (before: creep at a·dt, about 6.7 mm/s at 20 N) |
| `#pragma push_macro("RT")` / `#undef RT` around the Pinocchio includes (header and .cpp) | Simulink compiles with `-DRT`, which breaks a Boost.Math template parameter named `RT` |

On passivity: the CBF still adds no energy at any sample instant (f > 0 only while ζ̇ ≤ 0). The one exception is when the drift estimate is too large, e.g. F_ext lagging after you release. Then the robot can be pushed away for at most one cycle: a separation speed of about (a_est − a_true)·dt and roughly 44 µJ in the worst case simulated.

The Pinocchio-optional build switch (`TORQUE_CBF_WITH_PINOCCHIO`) was added and then reverted at your request. Pinocchio is always required.

### S-function (`sfun_torque_cbf.cpp`)

| Change | Why |
|---|---|
| Parameter vector 17 → 24: entries 18–24 are the per-joint τ_max; the S-function sets `tau_min = −tau_max` and `enforce_total_torque_limits = true` | real Panda limits on the total torque |
| Computes only in major time steps and re-emits the last output in minor steps | in the `ode4` simulation models the latch/filter state was updated about 4× per ms; no effect on the robot (discrete 1 ms) |
| Telemetry written on every path (zeros if unconfigured or the step fails) | it previously kept stale values |

### MATLAB scripts (`torque_cbf_simulink/`)

| File | Change |
|---|---|
| `init_torque_cbf.m` | `cbf_tau_max = [87 87 87 87 12 12 12]`; spring-damper parameters (`cbf_Kd_trans = [2000 2000 0]`, `cbf_Kd_rot = [120 120 120]`, `cbf_zeta_trans = [0.7 0.7 0]`, `cbf_zeta_rot = [0.7 0.7 0.7]`); `cbf_stiff_decouple = 1`; `cbf_log_decimation = 1`; 24-element parameter vector with a size check. Your own tuning in the same file: v₀ = −0.8, γ = 12, w_lin = 1e10, w_slack = 1e8, SG causal with N = 21 |
| `build_mex.m` | also searches `../pinocchio_install/include` |
| `add_lateral_stiffness.m` (new) | builds `Lateral_Stiffness`, which computes τ_stiff = −Jᵀ·F. F is the spring F_k (same formula as the damping model's `Stiffnessforces`) plus damping F_d = D·Ṽ with Dᵢ = 2ζᵢ√(Kᵢ·Λᵢᵢ). With `cbf_stiff_decouple = 1`, F ← F − n6·(n6ᵀΛ⁻¹F)/(n6ᵀΛ⁻¹n6). It also wires τ_stiff into the command and into the CBF's τ_nom, and drives `n_hp` from `plane_normal` |
| `add_cbf_logger.m`, `load_cbf_log.m` (new) | 49-column `cbf_log` (time, telemetry, F_ext fed to the CBF, use_fext, filtered and raw F_ext, τ_cbf, τ_stiff, τ_null); the loader unpacks it and plots it |

### Simulink model, as saved

| Block or connection | Status |
|---|---|
| `Lateral_Stiffness` with 11 inputs (including `n_hp` and `cbf_stiff_decouple`) | applied |
| `Sum` with 4 inputs: τ_null + 0 + τ_cbf + τ_stiff | applied |
| `Sum_tau_nom_cbf` (τ_null + τ_stiff) → S-function input 9 | applied |
| `n_hp` constant = `[plane_normal/‖plane_normal‖; 0; 0; 0]` | applied |
| `CBF_Logger` | applied, but its "F_ext fed to the CBF" column still points at the old signal (see section 6) |
| `Fext_full_flange` + `FIR_Fext_full` → input 11; `use_fext = 1`; Pinocchio in Custom Code | your own changes |

Why decoupling: at four typical poses, a sideways force F_x accelerates the end-effector along z with 50–70% of the effect of the same vertical force. A moment about y gives −1.1 to −1.9 m/s² per Nm. The CBF modelled this, but it was spending braking force cancelling the spring, and any model error in these large terms showed up as pulsing in f_n. After decoupling, the spring-damper's normal acceleration is below 1e-14 m/s², checked numerically at three poses.

---

## 3. Damping CBF: how it is fed

The S-function `sfun_damping_cbf` computes a damping coefficient d along the normal. That damping is injected into the impedance law's damping matrix.

### Inputs

| Port | Signal | Source in the model | Notes |
|---|---|---|---|
| 1 | ζ | From `x_c` (`MATLAB Function2`) | n_hpᵀ·(flange position − table height on z) |
| 2 | ζ̇ | From `v_c` = `Subsystem2`(n_hpᵀ·Ṽ) | first-order low-pass at 100 rad/s (about 10 ms lag) |
| 3 | ρ | From `mass_c` = 1/(n_hpᵀΛ⁻¹n_hp) | |
| 4 | F_nominal | `Compute_F_nominal` | −F_k − D₀·Ṽ; with `cbf_stiff_decouple = 1` its normal component is removed |
| 5 | F_ext | From `F_ext` = min(−1e-3, −SG(n_hp ∘ `O_F_ext_hat_K`)) | centered SG, N = 51 (25 ms delay); clipped element by element |
| 6 | use_fext | `Constant_use_fext_damp` = 1 | |
| 7 | params | `Compute eff d_max`(`cbf_damping_params`, ρ, `cbf_kappa`) | d_max_eff = min(d_max, κ·ρ) |

### What the CBF computes

$$a_{drift} = \frac{n_6^\top (F_{nominal} + F_{ext})}{\rho}$$

This uses only the normal component of the forces: no Coriolis or J̇q̇ terms, and no Λ⁻¹ cross-coupling.

The constraint, in the damping d and the slack ε:

$$\dot B = a_{drift} - v_{min}'(\zeta)\dot\zeta - \frac{d}{\rho}\dot\zeta \ \ge\ -\gamma B - \varepsilon,\qquad 0 \le d \le d_{max,eff}$$

The QP is two-dimensional in [d, ε].

### Where the damping goes

1. d → Goto `damp` → `MatrixMultiply3` builds d·n nᵀ → `Manual Switch1`.
2. `Sum3` adds it to the impedance damping: D_d = diag(0.6√K) + d·n nᵀ.
3. `impedance controller1` turns it into the force F_cbf = −d·n·(nᵀṼ). Note that this uses the **unfiltered** Ṽ.
4. τ_task = Jᵀ·F_control → `Sum`: τ_null + 0 + τ_task → `Rate Limiter` → robot.

The impedance law, with the decoupling in place:

$$F_{control} = \mu\dot q + \Lambda\dot V_{ref} - \mu J^{\#}\tilde V + F_{lat} + F_{cbf}$$

- F_lat = −D_lat·Ṽ − F_k, decoupled from the normal.
- D_lat = D_d − (nᵀD_d n)·n nᵀ.
- μ = Λ(J M⁻¹ C − J̇).

---

## 4. Damping CBF: what changed

| Where | Change | Why |
|---|---|---|
| `impedance controller1` (MATLAB Function) | splits D_d into the lateral damping D_lat and the CBF damping d·n nᵀ; decouples only F_lat = −D_lat·Ṽ − F_k from the normal; keeps F_cbf unchanged; new input 15 = `cbf_stiff_decouple` | spring and lateral damping no longer accelerate the end-effector along n; with the flag at 0 the output is identical to the original (checked to ≤ 4e-15) |
| `Compute_F_nominal` (MATLAB Function) | new inputs `n_hp` and `cbf_stiff_decouple`; removes the normal component of the nominal force | the decoupled impedance has zero normal effect, so the CBF must see zero nominal normal force |
| `n_hp` constant (`Constant7`) | `[plane_normal/‖plane_normal‖; 0; 0; 0]` instead of a hardcoded `[0;0;1;0;0;0]` | same plane as the CBF parameters |
| `init_damping_cbf.m` | `cbf_stiff_decouple = 1` | switch |
| `decouple_damping_impedance.m` (new) | applies the three model changes above; safe to re-run | |

Model status: all applied (impedance has 15 inputs, `Compute_F_nominal` has 5).

Key finding: before decoupling, the damping CBF was given a nominal normal force of exactly 0, because K_z = 0. Meanwhile the impedance accelerated the end-effector along the normal by 3.9, 25.8 and 6.2 m/s² at three test poses, a drift the CBF did not model at all. Afterwards the residual is about 1e-15 m/s².

The damping C++ library and `sfun_damping_cbf.cpp` were **not** modified in this work. Their uncommitted git diffs predate it.

---

## 5. Shared code and real-time build

| Item | State |
|---|---|
| `cbf_common/quadprog.hpp`: Goldfarb-Idnani multiplier fix (`u_plus`); staged before this work | verified on 200,000 random single-plane CBF problems, including the degenerate box 0 ≤ f ≤ 0: no failures, no mismatches |
| Pinocchio in the torque model's real-time build | Code Generation → Custom Code: include `pinocchio_install/include`; libraries `libpinocchio.so`, `libboost_filesystem/serialization/system.so`; define `PINOCCHIO_WITH_URDFDOM`; at run time the library is found through an rpath linker flag or `LD_LIBRARY_PATH` |
| Analytic dynamics blocks (`H_0e`, J, J̇, M, C, g) | match the URDF to about 1e-11; J has linear rows first, as both S-functions assume |

---

## 6. Open items before the next robot run

- [ ] **Logger points at the old F_ext.** S-function input 11 now gets `F_ext_full`, but the logger's "F_ext fed to the CBF" column is still wired to the old `F_ext` signal (`From3`). Re-run `add_cbf_logger`, which rebuilds the logger from whatever feeds input 11.
- [ ] **`cbf_K_offset` is not defined in any script.** `Const_K_offset` needs it; add it to `init_torque_cbf.m`.
- [ ] **Entries faster than v₀.** In the 52 s log you entered the zone at −0.9 to −1.1 m/s against v₀ = −0.8, giving B ≈ −0.3 at entry. Enable `cbf_latch_entry = 1`, or set v₀ faster than your real entry speed.
- [ ] **Robot-side MAT file keeps only the last 1000 samples.** This comes from *Limit data points to last: 1000* under Data Import/Export.
- [ ] **Rate limiter not modelled.** Neither CBF accounts for the ±999 Nm/s rate limiter before `Apply Control`.
- [ ] **Damping model, F_ext:** clipped and 25 ms late (centered SG, N = 51).
- [ ] **Damping model, ζ̇:** low-passed at 100 rad/s, while the applied damping uses the unfiltered velocity.
- [ ] **Damping model, a_drift:** has no Coriolis or J̇q̇ terms.
- [ ] **Damping model, tilted planes:** `x_c` subtracts the table height on z only, so ζ is the true plane distance only for a horizontal plane. The torque CBF uses the plane offset from its parameters and is general.
- [ ] **Not yet run on the robot.** Neither decoupled version has been run; the 52 s torque log predates decoupling.
- [ ] **Proposed, not implemented:** a diagnostic S-function output (a_drift split, ρ, braking cap), and a disturbance-estimation term in the torque CBF.

---

## 7. How to re-apply and rebuild

Torque model:
1. `cd torque_cbf_simulink`, then `init_torque_cbf`.
2. `add_lateral_stiffness` (only needed after changing the script).
3. `add_cbf_logger` (needed now, see section 6).
4. Build and run. Afterwards: `L = load_cbf_log;`

Damping model:
1. `cd damping_cbf_simulink`, then `init_damping_cbf`.
2. `decouple_damping_impedance` (already applied; re-run only after changing the script).
3. Build and run.

C++ (library, MEX, examples): `cmake --build build -j8` from the repository root. Alternatively, `build_mex` in MATLAB for the torque S-function.

All model scripts save the model, so close it in other MATLAB sessions first. With `cbf_stiff_decouple = 0` in either init script, the decoupling is switched off and both controllers behave exactly as before it.
