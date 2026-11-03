# CBF Variable-Damping Cartesian Compliance Controller (`damping_cbf`)

This workspace contains:
1. **Google-Slides-Ready Presentation**: [`damping_cbf_methodology.pptx`](damping_cbf_methodology.pptx) (and PDF export [`presentation/damping_cbf_methodology.pdf`](presentation/damping_cbf_methodology.pdf)) detailing the step-by-step mathematical derivation of the Phase-Space Control Barrier Function (CBF) and Quadratic Program (QP) for variable-damping Cartesian compliance control.
2. **C++17 CMake Package (`damping_cbf`)**: A standalone, real-time ($1\text{ kHz}$) C++ implementation of the CBF variable-damping controller (`damping_cbf::DampingCBFController`) using `Eigen3`, along with a verification and simulation suite (`example_damping_cbf`).

---

## 1. Directory Structure

```text
damping_cbf_folder/
├── CMakeLists.txt                              # CMake build configuration (Eigen3, C++17)
├── README.md                                   # Documentation and usage instructions
├── damping_cbf_methodology.pptx                # Google-Slides-compatible 16:9 presentation
├── include/
│   └── damping_cbf/
│       └── damping_cbf_controller.hpp          # DampingCBFController class declaration
├── src/
│   └── damping_cbf_controller.cpp              # DampingCBFController class implementation
├── examples/
│   └── example_damping_cbf.cpp                 # 1 kHz simulation & verification script
└── presentation/
    ├── generate_presentation.py                # Script that renders LaTeX equations, plots & PPTX
    ├── damping_cbf_methodology.pptx            # 16:9 editable Google Slides presentation
    └── damping_cbf_methodology.pdf             # Rendered PDF version of the presentation
```

---

## 2. Importing the Presentation into Google Slides

The presentation [`damping_cbf_methodology.pptx`](damping_cbf_methodology.pptx) is specifically engineered so that **fonts, equations, and figures never break when imported into Google Slides**:
- All editable text boxes (titles, headers, bullet points, captions) use **Arial**, which is natively built into Google Slides with identical character metrics.
- All display equations are compiled with `pdflatex` and embedded as **450 DPI transparent PNGs** with locked aspect ratios, avoiding OMML/Computer-Modern font substitution bugs in Google Slides.

### How to Open in Google Slides:
1. Upload [`damping_cbf_methodology.pptx`](damping_cbf_methodology.pptx) to **Google Drive**, right-click it, and select **Open with -> Google Slides**; **OR**
2. Inside an existing Google Slides deck, click **File -> Import slides -> Upload** and select [`damping_cbf_methodology.pptx`](damping_cbf_methodology.pptx).

---

## 3. Building and Running the C++ Package

### Prerequisites
- CMake ($\ge 3.14$)
- C++17 compiler (`g++` or `clang++`)
- `Eigen3` (`sudo apt install libeigen3-dev`)

### Build & Run Instructions (`colcon` — Pure CMake)

```bash
cd /home/galileo/projects/damping_cbf_ws
colcon build
./install/damping_cbf/bin/example_damping_cbf
python3 src/damping_cbf_controller/examples/plot_examples.py --show
```

### Build & Run Instructions (Standalone CMake)

```bash
cd /home/galileo/projects/damping_cbf_ws/src/damping_cbf_controller
mkdir -p build && cd build
cmake ..
make -j4
./example_damping_cbf
python3 ../examples/plot_examples.py --show
```

---

## 4. Using `DampingCBFController` in Your Code

```cpp
#include "damping_cbf/damping_cbf_controller.hpp"

// 1. Configure hyperplane (normal pointing into free space, offset on surface)
damping_cbf::Hyperplane plane;
plane.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
plane.offset = Eigen::Vector3d(0.0, 0.0, 0.10);

// 2. Configure CBF & velocity envelope parameters
damping_cbf::ControllerConfig cfg;
cfg.p_ths = 0.20;      // Braking zone thickness [m]
cfg.v_ths = -0.002;    // Target touchdown velocity [m/s] (< 0)
cfg.v_0   = -0.35;     // Nominal entry velocity at zeta = p_ths [m/s]
cfg.gamma = 20.0;      // CBF convergence rate [1/s]
cfg.d_max = 250.0;     // Maximum damping factor [Ns/m]
cfg.profile = damping_cbf::ProfileType::SMOOTHSTEP;

damping_cbf::DampingCBFController controller(cfg, plane);

// 3a. Compute optimal damping matrix D_IMP directly in task space:
double zeta  = controller.computeDistance(x);
double dzeta = controller.computeNormalVelocity(dx);
double rho   = damping_cbf::DampingCBFController::computeEffectiveInertia(Lambda, plane.normal);
double a_drift = f_ext_normal / rho;

damping_cbf::CBFStepResult res = controller.solveDampingQP(zeta, dzeta, rho, a_drift);
Eigen::Matrix3d D_imp = res.D_imp;  // Symmetric PSD 3x3 damping matrix

// 3b. Or compute full n-DoF joint torques tau directly:
Eigen::VectorXd tau = controller.computeControlTorque(robot_state, x_ref, phi_ref, tau_null, &res);
```
