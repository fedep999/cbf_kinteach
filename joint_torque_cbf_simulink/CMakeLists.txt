cmake_minimum_required(VERSION 3.16)
project(joint_torque_cbf_simulink VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED PATHS /opt/ros/jazzy /usr /usr/local /opt/openrobots)
if(NOT TARGET cbf_common AND NOT TARGET cbf_common::cbf_common)
  find_package(cbf_common REQUIRED)
endif()
if(NOT TARGET joint_torque_cbf_controller AND NOT TARGET jtorque_cbf::joint_torque_cbf_controller)
  find_package(jtorque_cbf REQUIRED)
endif()

find_package(Matlab COMPONENTS MAIN_PROGRAM MEX_COMPILER SIMULINK QUIET)

if(Matlab_FOUND)
  matlab_add_mex(
    NAME sfun_joint_torque_cbf
    SRC src/sfun_joint_torque_cbf.cpp
    LINK_TO
      jtorque_cbf::joint_torque_cbf_controller
      cbf_common::cbf_common
  )
  target_include_directories(sfun_joint_torque_cbf PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${Matlab_INCLUDE_DIRS}
    ${Matlab_ROOT_DIR}/simulink/include
  )
  if(EXISTS "/opt/ros/jazzy/include/pinocchio/deprecated")
    target_include_directories(sfun_joint_torque_cbf PRIVATE /opt/ros/jazzy/include/pinocchio/deprecated)
  endif()

  # Runtime search paths (RPATH) so the MEX binary finds the shared libraries
  set(_mex_rpaths "${CMAKE_BINARY_DIR}/lib;$ORIGIN/../build/lib")
  foreach(_prefix ${CMAKE_PREFIX_PATH})
    if(EXISTS "${_prefix}/lib")
      list(APPEND _mex_rpaths "${_prefix}/lib")
    endif()
  endforeach()
  set_target_properties(sfun_joint_torque_cbf PROPERTIES
    BUILD_RPATH "${_mex_rpaths}"
    INSTALL_RPATH "$ORIGIN/../build/lib"
  )

  # Copy the MEX binary next to the model
  add_custom_command(
    TARGET sfun_joint_torque_cbf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:sfun_joint_torque_cbf>
            "${CMAKE_CURRENT_SOURCE_DIR}/"
    COMMENT "Copying sfun_joint_torque_cbf MEX binary to ${CMAKE_CURRENT_SOURCE_DIR}"
  )

  install(
    TARGETS sfun_joint_torque_cbf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(
  FILES
    build_mex.m
    init_joint_torque_cbf.m
    setup_joint_torque_model.m
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(ament_cmake_FOUND)
  ament_package()
endif()
