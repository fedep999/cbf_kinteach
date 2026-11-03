cmake_minimum_required(VERSION 3.14)
project(damping_cbf VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Embed relative RPATH ($ORIGIN/../lib) so installed binaries run without LD_LIBRARY_PATH
set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED PATHS /opt/ros/jazzy /usr /usr/local /opt/openrobots)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET cbf_common AND NOT TARGET cbf_common::cbf_common)
  find_package(cbf_common REQUIRED)
endif()

# Core CBF Variable-Damping Controller Library
add_library(damping_cbf_controller SHARED
  src/damping_cbf_controller.cpp
)
add_library(${PROJECT_NAME}::damping_cbf_controller ALIAS damping_cbf_controller)

target_include_directories(damping_cbf_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(damping_cbf_controller
  PUBLIC
    Eigen3::Eigen
    cbf_common::cbf_common
)

target_compile_options(damping_cbf_controller PRIVATE -Wall -Wextra -Wpedantic)

# Example Simulation & Verification Executable
add_executable(example_damping_cbf
  examples/example_damping_cbf.cpp
)

target_link_libraries(example_damping_cbf
  PRIVATE
    damping_cbf_controller
)

target_compile_options(example_damping_cbf PRIVATE -Wall -Wextra -Wpedantic)

# Real UR10e Apple Picking Cell Simulation Executable with Pinocchio & CBF
add_executable(simulate_apple_cell
  examples/simulate_apple_cell.cpp
)

target_link_libraries(simulate_apple_cell
  PRIVATE
    damping_cbf_controller
    pinocchio::pinocchio
    yaml-cpp::yaml-cpp
)

target_compile_options(simulate_apple_cell PRIVATE -Wall -Wextra -Wpedantic)

# Install headers
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install library and executable
install(
  TARGETS damping_cbf_controller example_damping_cbf simulate_apple_cell
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  TARGETS example_damping_cbf simulate_apple_cell
  DESTINATION lib/${PROJECT_NAME}
)

# Install plotting and simulation scripts
install(
  PROGRAMS
    examples/plot_examples.py
    examples/plot_apple_cell.py
    scripts/cbf_rviz_publisher.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  PROGRAMS
    scripts/cbf_rviz_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz config and launch files
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
  ament_export_libraries(damping_cbf_controller)
  ament_export_targets(${PROJECT_NAME}Targets)
  ament_package()
else()
  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )
endif()
