#!/usr/bin/env python3
"""Launch file to visualize the UR10e Apple Cell Multi-Surface Damping CBF in RViz2.

Starts:
  1. robot_state_publisher with the full Apple Picking Cell URDF (from xacro)
  2. cbf_rviz_publisher node (replaying the 2-plane CBF trajectory & publishing planes)
  3. rviz2 loaded with config/cbf_apple_cell.rviz
"""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_root = Path(__file__).resolve().parent.parent

    # Path to trajectory CSV
    default_csv_path = str(pkg_root / "examples" / "csv" / "sim_apple_cell_trajectory.csv")
    csv_arg = DeclareLaunchArgument(
        "csv_path",
        default_value=default_csv_path,
        description="Path to the simulation trajectory CSV file"
    )

    # Path to RViz configuration
    rviz_config_file = str(pkg_root / "config" / "cbf_apple_cell.rviz")

    # Path to replay script
    script_path = str(pkg_root / "scripts" / "cbf_rviz_publisher.py")

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

    # 2. CBF RViz Replay & Obstacle Publisher
    # Ran via ExecuteProcess to stay pure-CMake and not require an ament package.xml
    cbf_publisher_process = ExecuteProcess(
        cmd=[
            sys.executable,
            script_path,
            "--ros-args",
            "-p",
            ["csv_path:=", LaunchConfiguration("csv_path")],
            "-p",
            "playback_rate_hz:=50.0",
            "-p",
            "loop:=True",
        ],
        output="screen",
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
        cbf_publisher_process,
        rviz_node,
    ])
