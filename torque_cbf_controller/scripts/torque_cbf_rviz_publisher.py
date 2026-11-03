#!/usr/bin/env python3
"""ROS 2 visualization node that replays the UR10e Apple Cell Direct Joint-Torque CBF simulation in RViz2.

Publishes:
  - `/joint_states` (`sensor_msgs/JointState`): Real-time animation of UR10e arm and gripper
  - `/cbf_planes` (`visualization_msgs/MarkerArray`): 3D visual representations of Plane 1
    (Table Surface), Plane 2 (45° Inclined Wall), their normal vectors, and braking zones
  - `/cbf_trajectory` (`visualization_msgs/Marker`): 3D end-effector path (LINE_STRIP)
  - `/cbf_current_ee` (`visualization_msgs/Marker`): Current end-effector position (SPHERE)
    turning orange/red when CBF joint torque is actively applied
  - `/cbf_torque_arrow` (`visualization_msgs/Marker`): Normal deceleration arrow representing
    the CBF boundary intervention
"""

import csv
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class TorqueCbfRvizPublisher(Node):
    def __init__(self):
        super().__init__("torque_cbf_rviz_publisher")

        # Declare parameters
        self.declare_parameter("csv_path", "")
        self.declare_parameter("playback_rate_hz", 50.0)
        self.declare_parameter("loop", True)

        csv_param = self.get_parameter("csv_path").get_parameter_value().string_value
        if not csv_param:
            ws_root = Path(__file__).resolve().parent.parent.parent.parent
            csv_path = ws_root / "src" / "torque_cbf_controller" / "examples" / "csv" / "sim_apple_cell_torque_trajectory.csv"
        else:
            csv_path = Path(csv_param)

        # Generate trajectory CSV if missing
        if not csv_path.is_file():
            self.get_logger().info(f"Trajectory CSV not found at {csv_path}. Running simulate_apple_cell_torque ...")
            ws_root = Path(__file__).resolve().parent.parent.parent.parent
            candidates = [
                shutil.which("simulate_apple_cell_torque"),
                str(ws_root / "install" / "torque_cbf" / "bin" / "simulate_apple_cell_torque"),
                str(ws_root / "build" / "torque_cbf" / "simulate_apple_cell_torque"),
            ]
            sim_bin = next((c for c in candidates if c and os.path.isfile(c) and os.access(c, os.X_OK)), None)
            if sim_bin is None:
                raise FileNotFoundError(
                    f"Trajectory CSV missing and could not find 'simulate_apple_cell_torque' binary. "
                    f"Please build the workspace: colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
                )
            subprocess.run([sim_bin], check=True)

        self.get_logger().info(f"Loading trajectory from: {csv_path}")
        self.trajectory = self.load_trajectory(csv_path)
        self.get_logger().info(f"Loaded {len(self.trajectory)} trajectory frames.")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.planes_pub = self.create_publisher(MarkerArray, "/cbf_planes", 10)
        self.traj_pub = self.create_publisher(Marker, "/cbf_trajectory", 10)
        self.current_ee_pub = self.create_publisher(Marker, "/cbf_current_ee", 10)
        self.torque_arrow_pub = self.create_publisher(Marker, "/cbf_torque_arrow", 10)
        self.operator_arrow_pub = self.create_publisher(Marker, "/cbf_operator_arrow", 10)

        rate_hz = self.get_parameter("playback_rate_hz").get_parameter_value().double_value
        self.timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(self.timer_period, self.playback_callback)

        self.step_idx = 0
        # Downsample factor from 1000 Hz simulation to 50 Hz RViz playback (skip 20 sim steps per frame)
        self.stride = max(1, int(1000.0 / rate_hz))

        # Static plane definitions
        # Plane 1: Table surface at z = 1.05 m
        self.p1 = np.array([0.48, 0.16, 1.05])
        self.n1 = np.array([0.0, 0.0, 1.0])
        # Plane 2: 45° wall through (0.55, 0.16, 1.05) m with normal [-1/sqrt(2), 0, 1/sqrt(2)]
        self.p2 = np.array([0.55, 0.16, 1.05])
        self.n2 = np.array([-1.0 / math.sqrt(2.0), 0.0, 1.0 / math.sqrt(2.0)])

        self.joint_names = [
            "ur10e_shoulder_pan_joint",
            "ur10e_shoulder_lift_joint",
            "ur10e_elbow_joint",
            "ur10e_wrist_1_joint",
            "ur10e_wrist_2_joint",
            "ur10e_wrist_3_joint",
            # Gripper passive/mimic knuckle joints for complete visualization
            "ur10e_finger_joint",
            "ur10e_left_inner_finger_joint",
            "ur10e_left_inner_knuckle_joint",
            "ur10e_right_inner_knuckle_joint",
            "ur10e_right_outer_knuckle_joint",
            "ur10e_right_inner_finger_joint",
        ]

        self.get_logger().info("Torque CBF RViz Publisher initialized successfully.")

    def load_trajectory(self, path: Path):
        traj = []
        with path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                q = [float(row[f"q{i}"]) for i in range(6)]
                dq = [float(row[f"dq{i}"]) for i in range(6)]
                tau_cbf = [float(row[f"tau_cbf{i}"]) for i in range(6)]
                f_ext_vec = [
                    float(row.get("f_ext_x", 0.0)),
                    float(row.get("f_ext_y", 0.0)),
                    float(row.get("f_ext_z", 0.0))
                ]
                f_ext_norm = float(row.get("f_ext_norm", 0.0))
                f_cbf_norm = float(row.get("f_cbf_norm", 0.0))
                traj.append({
                    "t": float(row["t"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "zeta1": float(row["zeta1"]),
                    "zeta2": float(row["zeta2"]),
                    "act1": int(float(row.get("act1", 0))),
                    "act2": int(float(row.get("act2", 0))),
                    "tau_cbf_norm": float(row["tau_cbf_norm"]),
                    "q": q,
                    "dq": dq,
                    "tau_cbf": tau_cbf,
                    "f_ext_vec": f_ext_vec,
                    "f_ext_norm": f_ext_norm,
                    "f_cbf_norm": f_cbf_norm,
                })
        return traj

    def playback_callback(self):
        if not self.trajectory:
            return

        now = self.get_clock().now().to_msg()

        # Publish static 3D Plane markers and Trajectory line strip
        self.publish_planes(now)
        self.publish_trajectory_path(now)

        # Check loop at end of trajectory
        if self.step_idx >= len(self.trajectory):
            if self.get_parameter("loop").get_parameter_value().bool_value:
                self.step_idx = 0
            else:
                return

        frame = self.trajectory[self.step_idx]

        # 1. Publish JointState
        js = JointState()
        js.header.stamp = now
        js.header.frame_id = "world"
        js.name = self.joint_names
        # Arm positions + closed gripper
        js.position = list(frame["q"]) + [0.0] * 6
        js.velocity = list(frame["dq"]) + [0.0] * 6
        self.joint_pub.publish(js)

        # 2. Publish current End-Effector marker
        ee_marker = Marker()
        ee_marker.header.stamp = now
        ee_marker.header.frame_id = "world"
        ee_marker.ns = "cbf_ee_current"
        ee_marker.id = 0
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD
        ee_marker.pose.position.x = frame["x"]
        ee_marker.pose.position.y = frame["y"]
        ee_marker.pose.position.z = frame["z"]
        ee_marker.pose.orientation.w = 1.0
        ee_marker.scale.x = 0.035
        ee_marker.scale.y = 0.035
        ee_marker.scale.z = 0.035
        # If CBF torque is actively applied, highlight orange/red
        if frame["tau_cbf_norm"] > 0.5 or frame["act2"] == 1:
            ee_marker.color.r = 1.0
            ee_marker.color.g = 0.2
            ee_marker.color.b = 0.0
        else:
            ee_marker.color.r = 0.0
            ee_marker.color.g = 1.0
            ee_marker.color.b = 0.2
        ee_marker.color.a = 0.95
        self.current_ee_pub.publish(ee_marker)

        # 3. Publish Operator Guidance Force Arrow (Cyan) if F_ext is present
        if frame.get("f_ext_norm", 0.0) > 0.2:
            op_arrow = Marker()
            op_arrow.header.stamp = now
            op_arrow.header.frame_id = "world"
            op_arrow.ns = "operator_guidance_force"
            op_arrow.id = 11
            op_arrow.type = Marker.ARROW
            op_arrow.action = Marker.ADD
            p_ee = Point(x=frame["x"], y=frame["y"], z=frame["z"])
            # Scale length proportional to F_ext (capped at 0.25 m)
            op_len = min(0.25, 0.02 + frame["f_ext_norm"] * 0.01)
            f_vec = frame["f_ext_vec"]
            norm_val = max(1e-6, frame["f_ext_norm"])
            p_tip = Point(
                x=frame["x"] + (f_vec[0] / norm_val) * op_len,
                y=frame["y"] + (f_vec[1] / norm_val) * op_len,
                z=frame["z"] + (f_vec[2] / norm_val) * op_len
            )
            op_arrow.points = [p_ee, p_tip]
            op_arrow.scale.x = 0.012
            op_arrow.scale.y = 0.022
            op_arrow.scale.z = 0.025
            op_arrow.color.r = 0.0
            op_arrow.color.g = 0.75
            op_arrow.color.b = 1.0
            op_arrow.color.a = 0.90
            self.operator_arrow_pub.publish(op_arrow)
        else:
            op_arrow = Marker()
            op_arrow.header.stamp = now
            op_arrow.header.frame_id = "world"
            op_arrow.ns = "operator_guidance_force"
            op_arrow.id = 11
            op_arrow.action = Marker.DELETE
            self.operator_arrow_pub.publish(op_arrow)

        # 4. Publish CBF Deceleration Arrow (Orange/Red) along plane normal when active
        if frame["tau_cbf_norm"] > 0.2 or frame["act2"] == 1:
            arrow = Marker()
            arrow.header.stamp = now
            arrow.header.frame_id = "world"
            arrow.ns = "cbf_torque_intervention"
            arrow.id = 10
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            p_ee = Point(x=frame["x"], y=frame["y"], z=frame["z"])
            # Scale arrow proportional to torque norm or CBF force
            cbf_f_norm = frame.get("f_cbf_norm", frame["tau_cbf_norm"])
            arrow_len = min(0.25, 0.03 + cbf_f_norm * 0.02)
            p_tip = Point(
                x=frame["x"] + self.n2[0] * arrow_len,
                y=frame["y"] + self.n2[1] * arrow_len,
                z=frame["z"] + self.n2[2] * arrow_len
            )
            arrow.points = [p_ee, p_tip]
            arrow.scale.x = 0.016
            arrow.scale.y = 0.026
            arrow.scale.z = 0.032
            arrow.color.r = 1.0
            arrow.color.g = 0.2
            arrow.color.b = 0.0
            arrow.color.a = 0.95
            self.torque_arrow_pub.publish(arrow)
        else:
            # Delete arrow when inactive
            arrow = Marker()
            arrow.header.stamp = now
            arrow.header.frame_id = "world"
            arrow.ns = "cbf_torque_intervention"
            arrow.id = 10
            arrow.action = Marker.DELETE
            self.torque_arrow_pub.publish(arrow)

        self.step_idx += self.stride

    def publish_planes(self, stamp):
        marker_array = MarkerArray()

        # -------------------------------------------------------------------
        # Surface 1: Horizontal Table Plane (z = 1.05 m)
        # -------------------------------------------------------------------
        m1 = Marker()
        m1.header.stamp = stamp
        m1.header.frame_id = "world"
        m1.ns = "planes"
        m1.id = 1
        m1.type = Marker.CUBE
        m1.action = Marker.ADD
        m1.pose.position.x = self.p1[0]
        m1.pose.position.y = self.p1[1]
        m1.pose.position.z = self.p1[2]
        m1.pose.orientation.w = 1.0
        m1.scale.x = 0.50
        m1.scale.y = 0.50
        m1.scale.z = 0.008
        m1.color.r = 0.1
        m1.color.g = 0.5
        m1.color.b = 0.95
        m1.color.a = 0.70
        marker_array.markers.append(m1)

        # Surface 1: Normal Arrow (upward +z)
        a1 = Marker()
        a1.header.stamp = stamp
        a1.header.frame_id = "world"
        a1.ns = "plane_normals"
        a1.id = 2
        a1.type = Marker.ARROW
        a1.action = Marker.ADD
        p_start = Point(x=self.p1[0], y=self.p1[1], z=self.p1[2])
        p_end = Point(x=self.p1[0], y=self.p1[1], z=self.p1[2] + 0.15)
        a1.points = [p_start, p_end]
        a1.scale.x = 0.015
        a1.scale.y = 0.025
        a1.scale.z = 0.035
        a1.color.r = 0.0
        a1.color.g = 0.4
        a1.color.b = 1.0
        a1.color.a = 0.95
        marker_array.markers.append(a1)

        # -------------------------------------------------------------------
        # Surface 2: 45° Inclined Wall through (0.55, 0.16, 1.05) m
        # -------------------------------------------------------------------
        q_wall = rpy_to_quaternion(0.0, -math.pi / 4.0, 0.0)

        m2 = Marker()
        m2.header.stamp = stamp
        m2.header.frame_id = "world"
        m2.ns = "planes"
        m2.id = 5
        m2.type = Marker.CUBE
        m2.action = Marker.ADD
        m2.pose.position.x = self.p2[0] + 0.10 * math.cos(math.pi / 4.0)
        m2.pose.position.y = self.p2[1]
        m2.pose.position.z = self.p2[2] + 0.10 * math.sin(math.pi / 4.0)
        m2.pose.orientation = q_wall
        m2.scale.x = 0.45
        m2.scale.y = 0.50
        m2.scale.z = 0.008
        m2.color.r = 1.0
        m2.color.g = 0.5
        m2.color.b = 0.05
        m2.color.a = 0.70
        marker_array.markers.append(m2)

        # Surface 2: Normal Arrow
        a2 = Marker()
        a2.header.stamp = stamp
        a2.header.frame_id = "world"
        a2.ns = "plane_normals"
        a2.id = 6
        a2.type = Marker.ARROW
        a2.action = Marker.ADD
        p2_start = Point(x=self.p2[0], y=self.p2[1], z=self.p2[2])
        p2_end = Point(
            x=self.p2[0] + self.n2[0] * 0.15,
            y=self.p2[1] + self.n2[1] * 0.15,
            z=self.p2[2] + self.n2[2] * 0.15
        )
        a2.points = [p2_start, p2_end]
        a2.scale.x = 0.015
        a2.scale.y = 0.025
        a2.scale.z = 0.035
        a2.color.r = 1.0
        a2.color.g = 0.4
        a2.color.b = 0.0
        a2.color.a = 0.95
        marker_array.markers.append(a2)

        self.planes_pub.publish(marker_array)

    def publish_trajectory_path(self, stamp):
        if not self.trajectory:
            return

        # 3D Line Strip
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = "world"
        line.ns = "cbf_trajectory_line"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.008  # line width: 8 mm
        line.color.r = 0.1
        line.color.g = 0.9
        line.color.b = 0.3
        line.color.a = 0.90

        line.points = [
            Point(x=pt["x"], y=pt["y"], z=pt["z"])
            for pt in self.trajectory
        ]
        self.traj_pub.publish(line)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueCbfRvizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
