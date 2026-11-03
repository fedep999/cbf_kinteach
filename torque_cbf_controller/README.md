# Direct Joint-Torque CBF Controller (`torque_cbf`)

A real-time ($1\text{ kHz}$) C++17 implementation of a **Direct Joint-Torque Control Barrier Function (CBF) Controller** for an $n$-DOF manipulator using **Eigen3**, **Pinocchio**, and the **Goldfarb-Idnani dual active-set Quadratic Program (QP) solver**.

---

## 1. Mathematical Formulation

### 1.1 Manipulator Equations of Motion & Control Law
The equations of motion for an $n$-DOF manipulator subject to external Cartesian interaction wrench $F_{\text{ext}} \in \mathbb{R}^6$ are:
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau_c + J(q)^{\mathrm{T}} F_{\text{ext}}$$

The control law cancels gravity and injects optional nominal torque $\tau_{\text{nom}}$ plus optimal CBF braking torque $\tau_{\text{cbf}} \in \mathbb{R}^n$:
$$\tau_c = g(q) + \tau_{\text{nom}} + \tau_{\text{cbf}}$$

Cancelling $g(q)$ yields the acceleration dynamics:
$$\ddot{q} = M(q)^{-1}(\tau_{\text{nom}} + \tau_{\text{cbf}}) - M(q)^{-1}C(q,\dot{q})\dot{q} + M(q)^{-1}J(q)^{\mathrm{T}}F_{\text{ext}}$$

Inside the safe region when $b_{\text{cbf}} \le 0$ (and $\tau_{\text{nom}} = \mathbf{0}$), the QP yields $\tau_{\text{cbf}}^* = \mathbf{0}$, ensuring pure free-floating gravity compensation ($\tau_c = g(q)$).

---

### 1.2 Barrier Function Definition
For each boundary hyperplane $i \in \{1, \dots, m\}$ defined by outward unit normal $n_{6,i} = [n_i^{\mathrm{T}}, \mathbf{0}_{1 \times 3}]^{\mathrm{T}} \in \mathbb{R}^6$ and point $x_{0,i} \in \mathbb{R}^3$:
- Orthogonal distance: $\zeta_i = n_i^{\mathrm{T}}(x_{\text{ee}}(q) - x_{0,i})$
- Normal approach velocity: $\dot{\zeta}_i = n_{6,i}^{\mathrm{T}} J(q)\dot{q} = n_i^{\mathrm{T}} \dot{x}_{\text{ee}}$
- Safe velocity profile: $v_{\min}(\zeta_i) = v_{\text{ths}} - (v_{\text{ths}} - v_0)(1 - H(u)/H(1))$ with spatial derivative $v_{\min}'(\zeta_i)$
- Barrier function:
  $$B_i(q, \dot{q}) = \dot{\zeta}_i - v_{\min}(\zeta_i) \ge 0$$

---

### 1.3 CBF Inequality on Joint Torques
Enforcing class-$\mathcal{K}$ safe-set forward invariance $\dot{B}_i \ge -\gamma_i B_i$:
$$\dot{B}_i = \ddot{\zeta}_i - v_{\min}'(\zeta_i)\dot{\zeta}_i \ge -\gamma_i B_i$$

Substituting the Cartesian acceleration:
$$\ddot{\zeta}_i = n_{6,i}^{\mathrm{T}} \dot{J}\dot{q} + n_{6,i}^{\mathrm{T}} J M^{-1}(\tau_{\text{cbf}} + \tau_{\text{nom}} - C\dot{q} + J^{\mathrm{T}}F_{\text{ext}})$$

Yields the linear inequality constraint directly on $\tau_{\text{cbf}}$:
$$A_{\text{cbf}, i}(q) \, \tau_{\text{cbf}} + \delta \ge b_{\text{cbf}, i}(q, \dot{q}, F_{\text{ext}})$$

Where:
1. **Control Jacobian Row:**
   $$A_{\text{cbf}, i}(q) = n_{6,i}^{\mathrm{T}} J(q) M(q)^{-1} \in \mathbb{R}^{1 \times n}$$
2. **Acceleration Drift Term:**
   $$a_{\text{drift}, i} = n_{6,i}^{\mathrm{T}}(\dot{J}\dot{q} - J M^{-1} C\dot{q}) + n_{6,i}^{\mathrm{T}} J M^{-1}\tau_{\text{nom}} + [n_{6,i}^{\mathrm{T}} \Lambda^{-1} F_{\text{ext}} \text{ if compensated}]$$
   *(with Cartesian inverse inertia $\Lambda(q)^{-1} = J(q)M(q)^{-1}J(q)^{\mathrm{T}}$)*.
3. **Constraint Lower Bound:**
   $$b_{\text{cbf}, i} = v_{\min}'(\zeta_i)\dot{\zeta}_i - a_{\text{drift}, i} - \gamma_i (\dot{\zeta}_i - v_{\min}(\zeta_i))$$

---

### 1.4 Quadratic Program (QP) Formulation
At each 1 kHz cycle, solve for $z = [\tau_{\text{cbf}}^{\mathrm{T}}, \delta]^{\mathrm{T}} \in \mathbb{R}^{n+1}$:

$$\min_{\tau_{\text{cbf}} \in \mathbb{R}^n, \, \delta \ge 0} \quad \frac{1}{2} \|\tau_{\text{cbf}}\|_2^2 + \frac{1}{2} w_\delta \delta^2 + w_{\text{lin}} \delta$$

Subject to:
1. **CBF Safety Constraints ($m$ planes):**
   $$A_{\text{cbf}, i}(q) \tau_{\text{cbf}} + \delta \ge b_{\text{cbf}, i}, \quad \forall i \in \{1, \dots, m\}$$
2. **Physical Joint Torque Limits:**
   $$\tau_{\min} \le \tau_{\text{cbf}} \le \tau_{\max}$$
3. **Slack Non-negativity:**
   $$\delta \ge 0$$

Slack variable $\delta$ ensures that if external wrenches exceed motor limits, the QP degrades gracefully to maximum available braking torque rather than failing mathematically.

---

## 2. Package Architecture & Dependencies

The workspace is organized into three cooperating packages:
- **`cbf_common`**: Shared headers and utilities (`quadprog.hpp`, `hyperplane.hpp`, `profiles.hpp`, `profiles.cpp`).
- **`damping_cbf`**: Cartesian compliance variable-damping controller.
- **`torque_cbf`**: Direct joint-torque CBF controller (`TorqueCBFController`).

### Directory Layout
```text
src/torque_cbf_controller/
├── CMakeLists.txt                              # CMake build configuration
├── package.xml                                 # Package manifest
├── README.md                                   # Documentation
├── include/
│   └── torque_cbf/
│       └── torque_cbf_controller.hpp          # Controller class & state structures
├── src/
│   └── torque_cbf_controller.cpp              # Full Pinocchio + QP implementation
├── config/
│   └── apple_cell_torque_cbf.yaml              # Apple picking cell parameters
└── examples/
    ├── example_torque_cbf.cpp                 # 4 verification benchmark experiments
    ├── simulate_apple_cell_torque.cpp          # Full 1 kHz UR10e cell simulation
    ├── plot_torque_cbf.py                     # Benchmark plotting script
    ├── plot_apple_cell_torque.py              # Apple cell trajectory plotting script
    ├── csv/                                   # All simulation CSV logs
    └── plots/                                 # Generated publication-quality figures
```

---

## 3. Building and Running

> **Important**: Always compile from the workspace root (`/home/galileo/projects/damping_cbf_ws`).

```bash
cd /home/galileo/projects/damping_cbf_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Running the Verification Benchmark Suite
```bash
./install/torque_cbf/bin/example_torque_cbf
python3 src/torque_cbf_controller/examples/plot_torque_cbf.py
```

### Running the UR10e Apple Picking Cell Simulation
```bash
./install/torque_cbf/bin/simulate_apple_cell_torque --config src/torque_cbf_controller/config/apple_cell_torque_cbf.yaml --duration 9.0
python3 src/torque_cbf_controller/examples/plot_apple_cell_torque.py
```

### Visualizing in RViz2
Launch the full robot state publisher, 3D obstacle planes, end-effector trajectory, and live animated robot motion in RViz2:
```bash
ros2 launch torque_cbf visualize_torque_simulation.launch.py
```
*(or directly without sourcing install: `ros2 launch src/torque_cbf_controller/launch/visualize_torque_simulation.launch.py`)*

---

## 4. Verification Results Summary

1. **Profile Comparison ($h_1 \dots h_5$)**: All velocity shaping profiles successfully arrest the end-effector before contact ($\zeta \ge 0$). Smoothstep ($h_5$) produces continuous, smooth deceleration torques.
2. **Entry Velocities**:
   - *Slow ($B > 0$)*: Inside safe zone; no premature braking.
   - *Nominal ($B = 0$)*: Tracks safe velocity envelope smoothly from 18.5 cm.
   - *Fast ($B < 0$)*: Starts outside safe zone; CBF immediately activates at 20 cm boundary and restores forward invariance.
3. **External Disturbance Handling**: Tested under downward operator wrench $F_{\text{ext}, z} = -15$ N. Compensating $F_{\text{ext}}$ via Cartesian inverse inertia $\Lambda(q)^{-1}$ accurately cancels external push impact on drift.
4. **7-DOF Multi-Surface Corner Constraints**: Two orthogonal hyperplanes (Floor + Wall) are handled simultaneously in a single QP. Inside the safe region when $b_{\text{cbf}} \le 0$, $\|\tau_{\text{cbf}}\| = 0.000$ Nm and $\|\tau_{\text{cmd}} - g(q)\| = 0.000$ Nm (exact free-floating gravity compensation).
