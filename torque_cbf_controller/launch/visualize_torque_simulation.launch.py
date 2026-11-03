#!/usr/bin/env python3
"""Launch file to visualize the UR10e Apple Cell Direct Joint-Torque CBF simulation in RViz2.

Starts:
  1. robot_state_publisher with the full Apple Picking Cell URDF (from xacro)
  2. torque_cbf_rviz_publisher node (replaying the 2-plane CBF trajectory, torques, & obstacles)
  3. rviz2 loaded with config/cbf_apple_cell.rviz
"""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to trajectory CSV (defaults to src/torque_cbf_controller/examples/csv/sim_apple_cell_torque_trajectory.csv)
    default_csv_path = "/home/galileo/projects/damping_cbf_ws/src/torque_cbf_controller/examples/csv/sim_apple_cell_torque_trajectory.csv"
    csv_arg = DeclareLaunchArgument(
        "csv_path",
        default_value=default_csv_path,
        description="Path to the torque simulation trajectory CSV file"
    )

    # Path to RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("torque_cbf"),
        "config",
        "cbf_apple_cell.rviz"
    ])

    # Fallback to local source path if FindPackageShare fails
    cell_pkg_share = FindPackageShare("apple_picking_cell_description")
    xacro_path = PathJoinSubstitution([
        cell_pkg_share,
        "urdf",
        "apple_picking_cell.urdf.xacro"
    ])

    # Generate robot_description via xacro
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_path,
            " ",
            "generate_ros2_control:=False",
            " ",
            "robotiq_include_ros2_control:=False",
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 2. Torque CBF RViz Replay Node
    cbf_publisher_node = Node(
        package="torque_cbf",
        executable="torque_cbf_rviz_publisher.py",
        output="screen",
        parameters=[
            {"csv_path": LaunchConfiguration("csv_path")},
            {"playback_rate_hz": 50.0},
            {"loop": True},
        ],
    )

    # 3. RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    return LaunchDescription([
        csv_arg,
        robot_state_publisher_node,
        cbf_publisher_node,
        rviz_node,
    ])
