"""G3 domain-0 forwarder: ROS 2 state topics to portable JSON over Zenoh/TCP."""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

from xgc2_b2_link.codec import (
    driver_status_json,
    forwarder_heartbeat,
    imu_json,
    joint_states_json,
    now_ms,
    odom_json,
    pack_json,
    power_summary_from_battery_message,
    power_summary_from_low_state_fields,
    validate_payload,
)
from xgc2_b2_link.contract import full_key, load_contract
from xgc2_b2_link.rate import RateGate
from xgc2_b2_link.runtime_config import (
    installed_runtime_config,
    load_runtime_defaults,
    require_identity,
)
from xgc2_b2_link.transport import open_transport


def _try_import_rclpy():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        return rclpy, Node, qos_profile_sensor_data
    except ImportError:
        return None, None, None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="XGC2 B2 domain-0 forwarder (G3)")
    parser.add_argument("--config")
    parser.add_argument("--robot-id")
    parser.add_argument("--transport", choices=["zenoh", "tcp"])
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=7448)
    parser.add_argument("--tcp-role", default="client", choices=["client", "server"])
    parser.add_argument("--zenoh-mode", default="peer", choices=["peer", "client"])
    parser.add_argument("--zenoh-listen", action="append")
    parser.add_argument("--zenoh-connect", action="append")
    parser.add_argument("--enable-imu", action="store_true")
    parser.add_argument("--enable-odin-odom", action="store_true")
    parser.add_argument(
        "--dry-run-no-ros", action="store_true", help="publish contract-valid fixtures"
    )
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config")
    known, _ = pre.parse_known_args(raw_argv)
    parser = build_arg_parser()
    repeatable_defaults = {}
    try:
        config_path = known.config or installed_runtime_config("forwarder_d0.yaml")
        defaults = load_runtime_defaults(config_path, "d0")
    except ValueError as exc:
        parser.error(str(exc))
    for name in ("zenoh_listen", "zenoh_connect"):
        repeatable_defaults[name] = defaults.pop(name, [])
    parser.set_defaults(config=config_path, **defaults)
    args = parser.parse_args(raw_argv)
    for name in ("zenoh_listen", "zenoh_connect"):
        if getattr(args, name) is None:
            setattr(args, name, repeatable_defaults.get(name, []))
    try:
        require_identity(args)
    except ValueError as exc:
        parser.error(str(exc))
    return args


@dataclass
class ChannelStats:
    rx_count: int = 0
    tx_count: int = 0
    last_rx_ms: int = 0
    last_tx_ms: int = 0
    first_rx_monotonic: float = 0.0

    def received(self) -> None:
        self.rx_count += 1
        self.last_rx_ms = now_ms()
        if self.first_rx_monotonic <= 0:
            self.first_rx_monotonic = time.monotonic()

    def transmitted(self) -> None:
        self.tx_count += 1
        self.last_tx_ms = now_ms()

    def snapshot(self) -> Dict[str, Any]:
        elapsed = max(0.0, time.monotonic() - self.first_rx_monotonic)
        effective_hz = self.tx_count / elapsed if elapsed > 0 else 0.0
        return {
            "rx_count": self.rx_count,
            "tx_count": self.tx_count,
            "last_rx_ms": self.last_rx_ms,
            "last_tx_ms": self.last_tx_ms,
            "effective_hz": round(effective_hz, 3),
        }


class ForwarderCore:
    """Contract, rate and health logic independent of ROS."""

    def __init__(
        self,
        robot_id: str,
        transport,
        *,
        transport_name: str,
        enable_imu: bool = False,
        enable_odin_odom: bool = False,
    ) -> None:
        self.robot_id = robot_id
        self.transport = transport
        self.transport_name = transport_name
        self.contract = load_contract()
        self.prefix_tpl = self.contract.get("key_prefix_template", "xgc2/{robot_id}")
        self.started_monotonic = time.monotonic()
        self.enabled = {
            "odom": True,
            "joint_states": True,
            "imu": enable_imu,
            "power_summary": True,
            "driver_status": True,
            "odin_odom": enable_odin_odom,
            "forwarder_hb": True,
        }
        self.gates: Dict[str, RateGate] = {}
        self.stats: Dict[str, ChannelStats] = {}
        for name, spec in self.contract["channels"]["up"].items():
            if name not in self.enabled:
                continue
            self.gates[name] = RateGate(float(spec.get("default_max_hz", 10)))
            self.stats[name] = ChannelStats()

    def key(self, relative: str) -> str:
        return full_key(self.robot_id, relative, self.prefix_tpl)

    def publish_json(self, name: str, payload: Dict[str, Any]) -> bool:
        if not self.enabled.get(name, False):
            return False
        stats = self.stats[name]
        stats.received()
        gate = self.gates[name]
        if not gate.allow():
            return False
        spec = self.contract["channels"]["up"][name]
        validate_payload(str(spec["schema"]), payload)
        self.transport.put(self.key(str(spec["key"])), pack_json(payload))
        stats.transmitted()
        return True

    def publish_odom(self, message: Any) -> bool:
        return self.publish_json("odom", odom_json(message))

    def publish_joint_states(self, message: Any) -> bool:
        return self.publish_json("joint_states", joint_states_json(message))

    def publish_imu(self, message: Any) -> bool:
        return self.publish_json("imu", imu_json(message))

    def publish_driver_status(self, message: Any) -> bool:
        return self.publish_json("driver_status", driver_status_json(message))

    def publish_power_summary(self, fields: Dict[str, Any]) -> bool:
        return self.publish_json("power_summary", power_summary_from_low_state_fields(**fields))

    def publish_hb(self) -> bool:
        data_names = [name for name, enabled in self.enabled.items() if enabled and name != "forwarder_hb"]
        body = forwarder_heartbeat(
            robot_id=self.robot_id,
            domain=0,
            transport=self.transport_name,
            uptime_ms=int((time.monotonic() - self.started_monotonic) * 1000),
            channels=data_names,
            stats={name: self.stats[name].snapshot() for name in data_names},
        )
        return self.publish_json("forwarder_hb", body)


def run_ros_node(args: argparse.Namespace) -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "0":
        print("ROS_DOMAIN_ID must be 0 for b2_forwarder_d0", file=sys.stderr)
        return 2
    rclpy, Node, qos_profile_sensor_data = _try_import_rclpy()
    if rclpy is None:
        print("rclpy not available; install ROS 2 or use --dry-run-no-ros", file=sys.stderr)
        return 2

    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState, Imu, JointState

    try:
        from b2_ros2_driver.msg import LowState  # type: ignore
    except Exception:
        LowState = None

    transport = open_transport(
        kind=args.transport,
        zenoh_mode=args.zenoh_mode,
        zenoh_listen=args.zenoh_listen or None,
        zenoh_connect=args.zenoh_connect or None,
        tcp_host=args.tcp_host,
        tcp_port=args.tcp_port,
        tcp_role=args.tcp_role,
    )

    class ForwarderNode(Node):
        def __init__(self) -> None:
            super().__init__("xgc2_b2_forwarder_d0")
            self.core = ForwarderCore(
                args.robot_id,
                transport,
                transport_name=args.transport,
                enable_imu=args.enable_imu,
                enable_odin_odom=args.enable_odin_odom,
            )
            self.create_subscription(Odometry, "/b2/odom", self.core.publish_odom, qos_profile_sensor_data)
            self.create_subscription(
                JointState, "/b2/joint_states", self.core.publish_joint_states, qos_profile_sensor_data
            )
            self.create_subscription(
                BatteryState, "/b2/battery", self._on_battery, qos_profile_sensor_data
            )
            if LowState is not None:
                self.create_subscription(
                    LowState, "/b2/low_state", self._on_low_state, qos_profile_sensor_data
                )
            else:
                self.get_logger().info(
                    "optional b2_ros2_driver/LowState unavailable; using /b2/battery"
                )
            self.create_subscription(
                DiagnosticArray,
                "/b2/driver_status",
                self.core.publish_driver_status,
                qos_profile_sensor_data,
            )
            if args.enable_imu:
                self.create_subscription(Imu, "/b2/imu", self.core.publish_imu, qos_profile_sensor_data)
            if args.enable_odin_odom:
                self.create_subscription(
                    Odometry, "/odin1/odometry", self._on_odin_odom, qos_profile_sensor_data
                )
            self.create_timer(1.0, self.core.publish_hb)
            self.get_logger().info(
                f"forwarder d0 robot_id={args.robot_id} transport={args.transport} domain=0"
            )

        def _on_odin_odom(self, message: Any) -> None:
            self.core.publish_json("odin_odom", odom_json(message))

        def _on_low_state(self, message: Any) -> None:
            bms_state = getattr(message, "bms_state", None)
            self.core.publish_power_summary(
                {
                    "soc": getattr(bms_state, "soc", None),
                    "power_v": getattr(message, "power_v", None),
                    "power_a": getattr(message, "power_a", None),
                    "temperature_ntc1": getattr(message, "temperature_ntc1", None),
                    "temperature_ntc2": getattr(message, "temperature_ntc2", None),
                }
            )

        def _on_battery(self, message: Any) -> None:
            self.core.publish_json(
                "power_summary", power_summary_from_battery_message(message)
            )

    rclpy.init()
    node = ForwarderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        transport.close()
        rclpy.shutdown()
    return 0


def run_dry(args: argparse.Namespace) -> int:
    from types import SimpleNamespace

    transport = open_transport(
        kind=args.transport,
        zenoh_mode=args.zenoh_mode,
        zenoh_listen=args.zenoh_listen or None,
        zenoh_connect=args.zenoh_connect or None,
        tcp_host=args.tcp_host,
        tcp_port=args.tcp_port,
        tcp_role=args.tcp_role,
    )
    core = ForwarderCore(
        args.robot_id,
        transport,
        transport_name=args.transport,
        enable_imu=args.enable_imu,
    )
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="odom")
    vector = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    quat = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    odom = SimpleNamespace(
        header=header,
        child_frame_id="b2_description",
        pose=SimpleNamespace(pose=SimpleNamespace(position=vector, orientation=quat)),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=vector, angular=vector)),
    )
    joints = SimpleNamespace(
        header=header,
        name=["FR_hip_joint"],
        position=[0.0],
        velocity=[0.0],
        effort=[0.0],
    )
    diagnostic = SimpleNamespace(header=header, status=[])
    print(f"dry-run forwarder robot_id={args.robot_id} transport={args.transport}", flush=True)
    for index in range(5):
        core.publish_odom(odom)
        core.publish_joint_states(joints)
        core.publish_power_summary(
            {
                "soc": 80 - index,
                "power_v": 48.0,
                "power_a": -0.5,
                "temperature_ntc1": None,
                "temperature_ntc2": None,
            }
        )
        core.publish_driver_status(diagnostic)
        core.publish_hb()
        time.sleep(0.2)
    time.sleep(0.5)
    transport.close()
    print("dry-run done", flush=True)
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.dry_run_no_ros:
        return run_dry(args)
    return run_ros_node(args)


if __name__ == "__main__":
    sys.exit(main())
