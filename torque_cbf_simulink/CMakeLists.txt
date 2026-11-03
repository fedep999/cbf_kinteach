cmake_minimum_required(VERSION 3.16)
project(torque_cbf_simulink VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED PATHS /opt/ros/jazzy /usr /usr/local /opt/openrobots)
if(NOT TARGET cbf_common AND NOT TARGET cbf_common::cbf_common)
  find_package(cbf_common REQUIRED)
endif()
if(NOT TARGET torque_cbf_controller AND NOT TARGET torque_cbf::torque_cbf_controller)
  find_package(torque_cbf REQUIRED)
endif()

find_package(Matlab COMPONENTS MAIN_PROGRAM MEX_COMPILER SIMULINK QUIET)

if(Matlab_FOUND)
  matlab_add_mex(
    NAME sfun_torque_cbf
    SRC src/sfun_torque_cbf.cpp
    LINK_TO
      torque_cbf::torque_cbf_controller
      cbf_common::cbf_common
  )
  target_include_directories(sfun_torque_cbf PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${Matlab_INCLUDE_DIRS}
    ${Matlab_ROOT_DIR}/simulink/include
    /opt/ros/jazzy/include/pinocchio/deprecated
  )
  install(
    TARGETS sfun_torque_cbf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(
  FILES
    build_mex.m
    init_torque_cbf.m
    Controller_torque_cbf.slx
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(ament_cmake_FOUND)
  ament_package()
endif()
