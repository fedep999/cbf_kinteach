cmake_minimum_required(VERSION 3.16)
project(torque_cbf VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED PATHS /opt/ros/jazzy /usr /usr/local /opt/openrobots)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET cbf_common AND NOT TARGET cbf_common::cbf_common)
  find_package(cbf_common REQUIRED)
endif()

# Core Direct Joint-Torque CBF Controller Library
add_library(torque_cbf_controller SHARED
  src/torque_cbf_controller.cpp
)
add_library(${PROJECT_NAME}::torque_cbf_controller ALIAS torque_cbf_controller)

target_include_directories(torque_cbf_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(torque_cbf_controller
  PUBLIC
    Eigen3::Eigen
    cbf_common::cbf_common
    pinocchio::pinocchio
)

target_compile_options(torque_cbf_controller PRIVATE -Wall -Wextra -Wpedantic)

# Verification Benchmark Executable
add_executable(example_torque_cbf
  examples/example_torque_cbf.cpp
)

target_link_libraries(example_torque_cbf
  PRIVATE
    torque_cbf_controller
)

target_compile_options(example_torque_cbf PRIVATE -Wall -Wextra -Wpedantic)

# Real UR10e Apple Picking Cell Simulation Executable with Pinocchio & Direct Torque CBF
add_executable(simulate_apple_cell_torque
  examples/simulate_apple_cell_torque.cpp
)

target_link_libraries(simulate_apple_cell_torque
  PRIVATE
    torque_cbf_controller
    pinocchio::pinocchio
    yaml-cpp::yaml-cpp
)

target_compile_options(simulate_apple_cell_torque PRIVATE -Wall -Wextra -Wpedantic)

# Install headers
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install targets
install(
  TARGETS torque_cbf_controller example_torque_cbf simulate_apple_cell_torque
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Also install executables to lib/${PROJECT_NAME} so `ros2 run torque_cbf ...` works
install(
  TARGETS example_torque_cbf simulate_apple_cell_torque
  DESTINATION lib/${PROJECT_NAME}
)

# Install plotting scripts to bin
install(
  PROGRAMS
    examples/plot_torque_cbf.py
    examples/plot_apple_cell_torque.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install Python visualization node into lib/${PROJECT_NAME} so ros2 run/launch finds it as an executable
install(
  PROGRAMS
    scripts/torque_cbf_rviz_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz config and launch files to share/${PROJECT_NAME}
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_libraries(torque_cbf_controller)
  ament_export_targets(${PROJECT_NAME}Targets)
  ament_package()
else()
  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )
endif()
