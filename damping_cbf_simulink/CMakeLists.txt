cmake_minimum_required(VERSION 3.16)
project(damping_cbf_simulink VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
if(NOT TARGET cbf_common AND NOT TARGET cbf_common::cbf_common)
  find_package(cbf_common REQUIRED)
endif()
if(NOT TARGET damping_cbf_controller AND NOT TARGET damping_cbf::damping_cbf_controller)
  find_package(damping_cbf REQUIRED)
endif()

# Try to find MATLAB and build the MEX target
find_package(Matlab COMPONENTS MAIN_PROGRAM MEX_COMPILER SIMULINK QUIET)

if(Matlab_FOUND)
  matlab_add_mex(
    NAME sfun_damping_cbf
    SRC src/sfun_damping_cbf.cpp
    LINK_TO
      damping_cbf::damping_cbf_controller
      cbf_common::cbf_common
  )
  target_include_directories(sfun_damping_cbf PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${Matlab_INCLUDE_DIRS}
    ${Matlab_ROOT_DIR}/simulink/include
  )
  install(
    TARGETS sfun_damping_cbf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(
  FILES
    build_mex.m
    init_damping_cbf.m
    Controller_damping_cbf.slx
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(ament_cmake_FOUND)
  ament_package()
endif()
