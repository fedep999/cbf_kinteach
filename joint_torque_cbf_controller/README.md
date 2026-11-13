# jtorque_cbf: joint-space torque CBF

Variant of `torque_cbf` in which the QP decision variable is the CBF joint torque `tau_cbf` itself,
instead of a normal force `f_n` with `tau_cbf = J^T N f_n`. This lets the QP plan exactly the torques
the robot can reach under its per-joint torque-rate limit (Franka: 1000 Nm/s).

The properties of the normal-force design are kept as constraints:

| Property | Constraint |
|---|---|
| Barrier (same as `torque_cbf`) | `n_i^T J M^-1 tau + delta >= b_cbf,i` |
| One-way normal action, no pushback | `0 <= f_eq,i <= cap_i`, `f_eq,i = rho_i n_i^T J M^-1 tau` (normal acceleration from ALL force/moment components, times rho) |
| Passivity | `tau^T dq <= 0` |
| Bounded lateral force / moment | each component of `(I - N N^+) F_lin` within `max_lateral_force`, of `F_rot` within `max_moment`, `F = Lambda J M^-1 tau` |
| Actuator (hard) | joint torque limits and per-joint rate limit `tau_rate_limit` on `tau_nom + tau_cbf` |

The guarantee rows have their own slacks (one-way normal > passivity > lateral), weighted far above the
CBF slack. They become non-zero only when the actuator box makes a guarantee physically impossible, and
are reported in the result (`sigma_normal`, `sigma_power`, `sigma_lateral`).

With `tau_rate_limit = 0` and `max_lateral_force = max_moment = 0` the controller reproduces
`torque_cbf` (verified on 200 random Panda states: max difference 2e-4 Nm).

## Comparison

`compare_jtorque_cbf <urdf>` runs the equivalence check and a closed-loop comparison on Panda dynamics
with a 999 Nm/s per-joint rate limiter (normal approach, operator push + release, late fast entry).

## Status

C++ library and simulation only; there is no Simulink S-function yet.
