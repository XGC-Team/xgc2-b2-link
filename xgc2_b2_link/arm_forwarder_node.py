"""G3 domain-17 R5 status forwarder with an independent transport session."""

from __future__ import annotations

import argparse
import os
import sys
import time
from types import SimpleNamespace
from typing import Any, Dict

from xgc2_b2_link.codec import (
    arm_joint_states_json,
    forwarder_heartbeat,
    pack_json,
    validate_payload,
)
from xgc2_b2_link.contract import full_key, load_contract
from xgc2_b2_link.forwarder_node import ChannelStats
from xgc2_b2_link.rate import RateGate
from xgc2_b2_link.runtime_config import (
    installed_runtime_config,
    load_runtime_defaults,
    require_identity,
)
from xgc2_b2_link.sim_models import ARM_URDF_JOINTS
from xgc2_b2_link.transport import open_transport


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="XGC2 R5 domain-17 status forwarder (G3)")
    parser.add_argument("--config")
    parser.add_argument("--robot-id")
    parser.add_argument("--transport", choices=["zenoh", "tcp"])
    parser.add_argument("--source-topic", default="/arm_slave_l_status")
    parser.add_argument("--expected-rmw", default="rmw_fastrtps_cpp")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=7448)
    parser.add_argument("--tcp-role", default="client", choices=["client", "server"])
    parser.add_argument("--zenoh-mode", default="peer", choices=["peer", "client"])
    parser.add_argument("--zenoh-listen", action="append")
    parser.add_argument("--zenoh-connect", action="append")
    parser.add_argument("--dry-run-no-ros", action="store_true")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config")
    known, _ = pre.parse_known_args(raw_argv)
    parser = build_arg_parser()
    repeatable_defaults = {}
    try:
        config_path = known.config or installed_runtime_config("forwarder_d17.yaml")
        defaults = load_runtime_defaults(config_path, "d17")
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


class ArmForwarderCore:
    def __init__(self, robot_id: str, transport, *, transport_name: str) -> None:
        self.robot_id = robot_id
        self.transport = transport
        self.transport_name = transport_name
        self.contract = load_contract()
        self.prefix_tpl = self.contract.get("key_prefix_template", "xgc2/{robot_id}")
        up = self.contract["channels"]["up"]
        self.arm_gate = RateGate(float(up["arm_slave_status"]["default_max_hz"]))
        self.hb_gate = RateGate(float(up["arm_forwarder_hb"]["default_max_hz"]))
        self.arm_stats = ChannelStats()
        self.hb_stats = ChannelStats()
        self.started_monotonic = time.monotonic()

    def _key(self, channel: str) -> str:
        spec = self.contract["channels"]["up"][channel]
        return full_key(self.robot_id, str(spec["key"]), self.prefix_tpl)

    def publish_arm(self, message: Any) -> bool:
        self.arm_stats.received()
        if not self.arm_gate.allow():
            return False
        payload = arm_joint_states_json(message, ARM_URDF_JOINTS)
        validate_payload("arm_joint_states_v1", payload)
        self.transport.put(self._key("arm_slave_status"), pack_json(payload))
        self.arm_stats.transmitted()
        return True

    def publish_hb(self) -> bool:
        self.hb_stats.received()
        if not self.hb_gate.allow():
            return False
        payload = forwarder_heartbeat(
            robot_id=self.robot_id,
            domain=17,
            transport=self.transport_name,
            uptime_ms=int((time.monotonic() - self.started_monotonic) * 1000),
            channels=["arm_slave_status"],
            stats={"arm_slave_status": self.arm_stats.snapshot()},
        )
        validate_payload("forwarder_hb_v1", payload)
        self.transport.put(self._key("arm_forwarder_hb"), pack_json(payload))
        self.hb_stats.transmitted()
        return True


def _validate_source_environment(args: argparse.Namespace) -> None:
    if "cmd" in args.source_topic.lower() or "status" not in args.source_topic.lower():
        raise ValueError("domain-17 source must be a read-only status topic")
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain != "17":
        raise ValueError(f"ROS_DOMAIN_ID must be 17 for b2_forwarder_d17, got {domain!r}")
    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw != args.expected_rmw:
        raise ValueError(f"RMW_IMPLEMENTATION must be {args.expected_rmw}, got {rmw!r}")


def _open(args: argparse.Namespace):
    return open_transport(
        kind=args.transport,
        zenoh_mode=args.zenoh_mode,
        zenoh_listen=args.zenoh_listen or None,
        zenoh_connect=args.zenoh_connect or None,
        tcp_host=args.tcp_host,
        tcp_port=args.tcp_port,
        tcp_role=args.tcp_role,
    )


def run_ros_node(args: argparse.Namespace) -> int:
    try:
        _validate_source_environment(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    try:
        import rclpy
        from arx5_arm_msg.msg import RobotStatus  # type: ignore
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
    except ImportError as exc:
        print(f"required ROS 2 dependency unavailable: {exc}", file=sys.stderr)
        return 2

    transport = _open(args)

    class ArmForwarderNode(Node):
        def __init__(self) -> None:
            super().__init__("xgc2_b2_forwarder_d17")
            self.core = ArmForwarderCore(
                args.robot_id, transport, transport_name=args.transport
            )
            self.create_subscription(
                RobotStatus, args.source_topic, self.core.publish_arm, qos_profile_sensor_data
            )
            self.create_timer(1.0, self.core.publish_hb)
            self.get_logger().info(
                f"forwarder d17 robot_id={args.robot_id} topic={args.source_topic} "
                f"transport={args.transport} domain=17 rmw={args.expected_rmw}"
            )

    rclpy.init()
    node = ArmForwarderNode()
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
    transport = _open(args)
    core = ArmForwarderCore(args.robot_id, transport, transport_name=args.transport)
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="b2_description"
        ),
        joint_pos=[0.0] * 7,
        joint_vel=[0.0] * 7,
        joint_cur=[0.0] * 7,
    )
    for _ in range(5):
        core.publish_arm(message)
        core.publish_hb()
        time.sleep(0.2)
    time.sleep(0.5)
    transport.close()
    print("dry-run d17 done", flush=True)
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.dry_run_no_ros:
        return run_dry(args)
    return run_ros_node(args)


if __name__ == "__main__":
    sys.exit(main())
