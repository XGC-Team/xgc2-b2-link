"""LAB-only ROS 2 B2 source simulator; it never writes the transport directly."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from xgc2_b2_link.rate import RateGate
from xgc2_b2_link.sim_models import (
    BASE_FRAME,
    DRIVER_LEG_JOINTS,
    odom_circle,
    walk_leg_positions,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish contract inputs on ROS 2 /b2/*")
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until stopped")
    return parser


def _ros_message_types():
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import BatteryState, JointState

        return (
            rclpy,
            Node,
            qos_profile_sensor_data,
            BatteryState,
            DiagnosticArray,
            DiagnosticStatus,
            KeyValue,
            Odometry,
            JointState,
        )
    except ImportError as exc:
        raise RuntimeError(f"ROS 2 simulator dependency unavailable: {exc}") from exc


def run(args: argparse.Namespace) -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "0":
        print("ROS_DOMAIN_ID must be 0 for b2_sim_publisher", file=sys.stderr)
        return 2
    try:
        (
            rclpy,
            Node,
            qos_profile_sensor_data,
            BatteryState,
            DiagnosticArray,
            DiagnosticStatus,
            KeyValue,
            Odometry,
            JointState,
        ) = _ros_message_types()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    class B2SimNode(Node):
        def __init__(self) -> None:
            super().__init__("xgc2_b2_sim_publisher")
            self.odom_pub = self.create_publisher(Odometry, "/b2/odom", qos_profile_sensor_data)
            self.joint_pub = self.create_publisher(
                JointState, "/b2/joint_states", qos_profile_sensor_data
            )
            self.low_pub = self.create_publisher(
                BatteryState, "/b2/battery", qos_profile_sensor_data
            )
            self.driver_pub = self.create_publisher(
                DiagnosticArray, "/b2/driver_status", qos_profile_sensor_data
            )
            self.started_monotonic = time.monotonic()
            self.gates = {
                "odom": RateGate(15),
                "joint": RateGate(15),
                "power": RateGate(2),
                "driver": RateGate(1),
            }
            self.timer = self.create_timer(1.0 / max(args.hz, 1.0), self._tick)
            self.get_logger().info(
                f"LAB ROS source robot_id={args.robot_id} domain=0; transport is owned by forwarder"
            )

        def _stamp(self):
            return self.get_clock().now().to_msg()

        def _tick(self) -> None:
            elapsed = time.monotonic() - self.started_monotonic
            if args.duration > 0 and elapsed >= args.duration:
                rclpy.shutdown()
                return
            if self.gates["odom"].allow():
                model = odom_circle(elapsed)
                message = Odometry()
                message.header.stamp = self._stamp()
                message.header.frame_id = model["header"]["frame_id"]
                message.child_frame_id = model["child_frame_id"]
                for axis in ("x", "y", "z"):
                    setattr(message.pose.pose.position, axis, model["pose"]["position"][axis])
                    setattr(message.twist.twist.linear, axis, model["twist"]["linear"][axis])
                    setattr(message.twist.twist.angular, axis, model["twist"]["angular"][axis])
                for axis in ("x", "y", "z", "w"):
                    setattr(
                        message.pose.pose.orientation, axis, model["pose"]["orientation"][axis]
                    )
                self.odom_pub.publish(message)
            if self.gates["joint"].allow():
                message = JointState()
                message.header.stamp = self._stamp()
                message.header.frame_id = BASE_FRAME
                message.name = list(DRIVER_LEG_JOINTS)
                message.position = walk_leg_positions(elapsed)
                message.velocity = [0.0] * len(message.name)
                message.effort = [0.0] * len(message.name)
                self.joint_pub.publish(message)
            if self.gates["power"].allow():
                message = BatteryState()
                message.header.stamp = self._stamp()
                message.percentage = 0.7 + 0.1 * math.sin(0.05 * elapsed)
                message.voltage = 48.0 - 0.5 * math.sin(0.1 * elapsed)
                message.current = -1.2
                message.temperature = 35.0
                self.low_pub.publish(message)
            if self.gates["driver"].allow():
                message = DiagnosticArray()
                message.header.stamp = self._stamp()
                status = DiagnosticStatus()
                status.name = "b2_sim_driver"
                status.level = DiagnosticStatus.OK
                status.message = "OK"
                status.values = [
                    KeyValue(key="motion_enabled", value="false"),
                    KeyValue(key="command_stale", value="true"),
                ]
                message.status = [status]
                self.driver_pub.publish(message)

    rclpy.init()
    node = B2SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
