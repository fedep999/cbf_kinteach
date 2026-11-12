cmake_minimum_required(VERSION 3.16)
project(jtorque_cbf VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED PATHS /opt/ros/jazzy /usr /usr/local /opt/openrobots)
if(NOT TARGET cbf_common AND NOT TARGET cbf_common::cbf_common)
  find_package(cbf_common REQUIRED)
endif()

# Joint-space torque CBF controller library (tau_cbf as QP decision variable)
add_library(joint_torque_cbf_controller SHARED
  src/joint_torque_cbf_controller.cpp
)
add_library(${PROJECT_NAME}::joint_torque_cbf_controller ALIAS joint_torque_cbf_controller)

target_include_directories(joint_torque_cbf_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(joint_torque_cbf_controller
  PUBLIC
    Eigen3::Eigen
    cbf_common::cbf_common
    pinocchio::pinocchio
)

target_compile_options(joint_torque_cbf_controller PRIVATE -Wall -Wextra -Wpedantic)

# Closed-loop comparison with torque_cbf (normal-force decision variable) under a torque-rate limiter.
# Built only when the torque_cbf package is part of the same build.
if(TARGET torque_cbf_controller)
  add_executable(compare_jtorque_cbf examples/compare_jtorque_cbf.cpp)
  target_link_libraries(compare_jtorque_cbf
    PRIVATE
      joint_torque_cbf_controller
      torque_cbf_controller
      pinocchio::pinocchio
  )
  target_compile_options(compare_jtorque_cbf PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS joint_torque_cbf_controller
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_libraries(joint_torque_cbf_controller)
  ament_export_targets(${PROJECT_NAME}Targets)
  ament_package()
else()
  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )
endif()
