# Damping & Torque CBF Controllers

A modular C++ implementation of Control Barrier Functions (CBF) with damping-based collision avoidance and torque-level control, featuring full integration with MATLAB/Simulink (via MEX S-Functions) and dual compilation support for both **Pure CMake** (non-ROS environments) and **ROS 2 / Colcon**.

---

## 1. System Requirements & Dependencies

Before compiling, ensure the host machine has the necessary build tools and C++ libraries installed.

### Essential Build Tools
- **C++ Compiler**: GCC >= 9.0 or Clang >= 10 supporting C++17
- **CMake**: >= 3.16
- **Eigen3**: Linear algebra library (`libeigen3-dev`)
- **yaml-cpp**: YAML configuration parser (`libyaml-cpp-dev`)
- **Pinocchio**: Rigid-body dynamics library (required for torque-level controller)

### Installing Dependencies on Ubuntu (without ROS)
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libyaml-cpp-dev
```

#### Installing Pinocchio
If ROS 2 is not installed on the system, Pinocchio can be installed via:

- **Local Build & Installation in Current Folder (No Root Required)**:
  You can clone, compile, and install Pinocchio into a local `pinocchio_install/` prefix in the current directory:
  ```bash
  # 1. Install prerequisites (Boost and URDF DOM)
  sudo apt-get install -y libboost-all-dev liburdfdom-dev

  # 2. Clone Pinocchio repository into the current folder
  git clone --recursive https://github.com/stack-of-tasks/pinocchio.git
  cd pinocchio
  ```
  >[!TIP]
  **If your machine has CMake 3.16 (e.g. Ubuntu 20.04):**
  Upstream Pinocchio requires CMake >= 3.22. You can upgrade CMake in seconds without `sudo` via:
  ```bash
  pip install --upgrade cmake
  export PATH="$HOME/.local/bin:$PATH"
  ```
  Or download the official prebuilt binary tarball:
  ```bash
  wget https://github.com/Kitware/CMake/releases/download/v3.28.3/cmake-3.28.3-linux-x86_64.tar.gz
  tar -xzf cmake-3.28.3-linux-x86_64.tar.gz
  export PATH="$(pwd)/cmake-3.28.3-linux-x86_64/bin:$PATH"
  ```
  Alternatively, build older Pinocchio release `v2.6.20` which supports CMake 3.16:
  ```bash
  cd pinocchio && git checkout v2.6.20 && git submodule update --init --recursive
  ```
  ```bash
  # 3. Configure build with a local installation prefix
  mkdir build && cd build
  cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$(pwd)/../../pinocchio_install" \
      -DBUILD_WITH_COLLISION_SUPPORT=OFF \
      -DBUILD_UNIT_TESTS=OFF \
      -DBUILD_PYTHON_INTERFACE=OFF

  # 4. Compile and install locally
  cmake --build . -j$(nproc)
  cmake --install .

  # Return to the root folder
  cd ../../cbf_kinteach
  ```

- **Making CMake Find Local Pinocchio**:
  Once installed in `./pinocchio_install`, make CMake discover it using any of the following methods:

  - **Method 1: Export environment variables**
    ```bash
    export CMAKE_PREFIX_PATH="$(pwd)/pinocchio_install:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$(pwd)/pinocchio_install/lib:$LD_LIBRARY_PATH"
    ```
  - **Method 2: Pass `CMAKE_PREFIX_PATH` during CMake configuration**
    ```bash
    mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(pwd)/../pinocchio_install"
    ```
  - **Method 3: Pass `pinocchio_DIR` directly**
    ```bash
    cmake .. -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR="$(pwd)/../pinocchio_install/lib/cmake/pinocchio"
    ```
  - **For ROS 2 / Colcon**:
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH="$(pwd)/pinocchio_install"
    ```

- **robotpkg APT repository** (System-wide on Ubuntu):
  ```bash
  sudo apt-get install -y -qq gnupg2 curl
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
  echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
  sudo apt-get update
  sudo apt-get install -y robotpkg-pinocchio
  # Add robotpkg to CMAKE_PREFIX_PATH:
  export CMAKE_PREFIX_PATH="/opt/openrobots:$CMAKE_PREFIX_PATH"
  ```
- Or via **Conda**:
  ```bash
  conda install pinocchio -c conda-forge
  ```

---

## 2. Compilation Using Pure CMake (No ROS / No Colcon)

The repository root includes a top-level `CMakeLists.txt` configuring all packages in proper dependency order.

### Step 1: Configure and Build
From the repository root directory:
```bash
# Create and enter build directory
mkdir build && cd build

# Configure with Release optimization
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build all libraries, executables, and MEX targets (if MATLAB is detected)
cmake --build . -j$(nproc)
```

> **Note**: If Pinocchio was installed in a custom or local location (e.g., local `pinocchio_install/` or `/opt/openrobots`), pass its prefix to CMake:
> ```bash
> cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(pwd)/../pinocchio_install"
> ```

### Build Artifacts
Upon completion, the build products are placed in:
- `build/lib/`:
  - `libcbf_common.so`: Core profiles and geometric barrier functions
  - `libdamping_cbf_controller.so`: Task-space damping CBF controller
  - `libtorque_cbf_controller.so`: Torque-level CBF controller with Pinocchio dynamics
  - `sfun_damping_cbf.mexa64`: Simulink MEX S-function for damping CBF *(if MATLAB found)*
  - `sfun_torque_cbf.mexa64`: Simulink MEX S-function for torque CBF *(if MATLAB found)*
- `build/bin/`:
  - Standalone simulation and example executables (`example_damping_cbf`, `simulate_apple_cell`, etc.)

---

## 3. MATLAB & Simulink Integration

The repository includes two Simulink simulation environments:
1. `damping_cbf_simulink/`: Damping CBF velocity/damping S-Function and simulation model (`Controller_damping_cbf.slx`).
2. `torque_cbf_simulink/`: Torque-level CBF S-Function and simulation model (`Controller_torque_cbf.slx`).

### 3.1 Compiling MEX Files inside MATLAB (Alternative to CMake)
If MATLAB was not detected during the CMake build, or if you prefer to compile MEX binaries directly inside MATLAB:

1. Open MATLAB and navigate to the desired package:
   ```matlab
   cd /path/to/src/damping_cbf_simulink
   build_mex
   ```
2. Navigate to the torque package:
   ```matlab
   cd /path/to/src/torque_cbf_simulink
   build_mex
   ```
The `build_mex.m` scripts automatically:
- Locate the compiled shared libraries (`libdamping_cbf_controller.so`, `libtorque_cbf_controller.so`, and `libcbf_common.so`) from either the pure CMake `build/lib` directory, the ROS 2 `install/` directory, or system library paths.
- Embed the runtime library search path (`RPATH`) into the generated `.mexa64` binary so MATLAB loads the dependencies without requiring manual `LD_LIBRARY_PATH` exports.

### 3.2 Running the Initialization Scripts for Simulink

Before a Simulink model can run, its corresponding MATLAB initialization script must be executed to load robot kinematics, safety hull parameters, and control gains into MATLAB's base workspace:
- `damping_cbf_simulink/init_damping_cbf.m`
- `torque_cbf_simulink/init_torque_cbf.m`

#### Execution via MATLAB Prompt:
Run the script in MATLAB before starting the simulation:
```matlab
% For Damping Controller:
cd /path/to/src/damping_cbf_simulink
init_damping_cbf
open_system('Controller_damping_cbf')

% For Torque Controller:
cd /path/to/src/torque_cbf_simulink
init_torque_cbf
open_system('Controller_torque_cbf')
```

---

## 4. Compilation Using ROS 2 / Colcon

On machines with ROS 2 installed, the packages maintain full standard colcon compatibility.

From the workspace root directory:
```bash
cd /path/to/damping_cbf_ws
colcon build --symlink-install
source install/setup.bash
```

The dual-mode build system automatically detects `ament_cmake` and registers all Ament indexes, exports, and install hooks.
