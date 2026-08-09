"""G4 contract/dry-run peer for validating the ground-side B2 wire."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict

from xgc2_b2_link.codec import now_ms, unpack_json, validate_payload
from xgc2_b2_link.contract import full_key, load_contract
from xgc2_b2_link.runtime_config import load_runtime_defaults, require_identity
from xgc2_b2_link.transport import open_transport


FRESHNESS_MS = {
    "odom": 1000,
    "joint_states": 1000,
    "power_summary": 2000,
    "driver_status": 3000,
    "forwarder_hb": 3000,
    "arm_slave_status": 1000,
    "arm_forwarder_hb": 3000,
}
BASE_REQUIRED = ("odom", "joint_states", "power_summary", "driver_status", "forwarder_hb")
ARM_REQUIRED = ("arm_slave_status", "arm_forwarder_hb")


@dataclass
class LatestStore:
    lock: threading.Lock = field(default_factory=threading.Lock)
    by_channel: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    def set(self, channel: str, meta: Dict[str, Any]) -> None:
        with self.lock:
            self.by_channel[channel] = meta

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {name: dict(value) for name, value in self.by_channel.items()}


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="XGC2 B2 ground contract peer (G4)")
    parser.add_argument("--config")
    parser.add_argument("--robot-id")
    parser.add_argument("--transport", choices=["zenoh", "tcp"])
    parser.add_argument("--tcp-host", default="0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=7448)
    parser.add_argument("--tcp-role", default="server", choices=["client", "server"])
    parser.add_argument("--zenoh-mode", default="peer", choices=["peer", "client"])
    parser.add_argument("--zenoh-listen", action="append")
    parser.add_argument("--zenoh-connect", action="append")
    parser.add_argument("--print-hz", type=float, default=0.5)
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config")
    known, _ = pre.parse_known_args(raw_argv)
    parser = build_arg_parser()
    repeatable_defaults = {}
    if known.config:
        defaults = load_runtime_defaults(known.config, "ground")
        for name in ("zenoh_listen", "zenoh_connect"):
            repeatable_defaults[name] = defaults.pop(name, [])
        parser.set_defaults(**defaults)
    args = parser.parse_args(raw_argv)
    for name in ("zenoh_listen", "zenoh_connect"):
        if getattr(args, name) is None:
            setattr(args, name, repeatable_defaults.get(name, []))
    try:
        require_identity(args)
    except ValueError as exc:
        parser.error(str(exc))
    return args


class GroundPeer:
    def __init__(self, robot_id: str, transport) -> None:
        self.robot_id = robot_id
        self.transport = transport
        self.contract = load_contract()
        self.prefix_tpl = self.contract.get("key_prefix_template", "xgc2/{robot_id}")
        self.store = LatestStore()
        self.decode_errors = 0
        self.drop_count = 0
        expression = full_key(robot_id, "up/**", self.prefix_tpl)
        self.transport.subscribe(expression, self._on_up)
        print(f"G4 contract peer listening robot_id={robot_id} expr={expression}", flush=True)

    def _channel_from_key(self, key: str) -> str:
        parts = key.split("/")
        try:
            up_index = parts.index("up")
            return "/".join(parts[up_index + 1 :])
        except ValueError:
            return parts[-1]

    def _on_up(self, key: str, payload: bytes) -> None:
        channel = self._channel_from_key(key)
        spec = self.contract["channels"]["up"].get(channel)
        if spec is None or spec.get("encode") != "json_utf8":
            self.drop_count += 1
            return
        received_monotonic_ms = int(time.monotonic() * 1000)
        try:
            body = unpack_json(payload)
            validate_payload(str(spec["schema"]), body)
        except Exception as exc:
            self.decode_errors += 1
            self.store.set(
                channel,
                {
                    "key": key,
                    "received_monotonic_ms": received_monotonic_ms,
                    "bytes": len(payload),
                    "error": str(exc),
                },
            )
            return
        meta: Dict[str, Any] = {
            "key": key,
            "t_ms": now_ms(),
            "received_monotonic_ms": received_monotonic_ms,
            "bytes": len(payload),
            "schema": spec["schema"],
            "json": body,
        }
        self.store.set(channel, meta)
    def health_snapshot(self) -> Dict[str, Any]:
        now_monotonic_ms = int(time.monotonic() * 1000)
        channels = self.store.snapshot()

        def fresh(name: str) -> bool:
            value = channels.get(name) or {}
            received = int(value.get("received_monotonic_ms", 0))
            return "error" not in value and received > 0 and (
                now_monotonic_ms - received <= FRESHNESS_MS[name]
            )

        freshness = {name: fresh(name) for name in FRESHNESS_MS}
        driver = (channels.get("driver_status") or {}).get("json") or {}
        base_online = all(freshness[name] for name in BASE_REQUIRED) and int(
            driver.get("level", 3)
        ) < 2
        arm_seen = any(name in channels for name in ARM_REQUIRED)
        arm_ready = arm_seen and all(freshness[name] for name in ARM_REQUIRED)
        return {
            "t_ms": now_ms(),
            "base_online": base_online,
            "arm_ready": arm_ready,
            "arm_seen": arm_seen,
            "fresh": freshness,
            "decode_errors": self.decode_errors,
            "drop_count": self.drop_count,
            "channels": sorted(channels),
        }

    def spin(self, print_hz: float = 0.5) -> None:
        period = 1.0 / print_hz if print_hz and print_hz > 0 else 1.0
        try:
            while True:
                time.sleep(period)
                if print_hz and print_hz > 0:
                    print(json.dumps(self.health_snapshot(), ensure_ascii=False), flush=True)
        except KeyboardInterrupt:
            pass


def main(argv=None) -> int:
    args = parse_args(argv)
    transport = open_transport(
        kind=args.transport,
        zenoh_mode=args.zenoh_mode,
        zenoh_listen=args.zenoh_listen or None,
        zenoh_connect=args.zenoh_connect or None,
        tcp_host=args.tcp_host,
        tcp_port=args.tcp_port,
        tcp_role=args.tcp_role,
    )
    peer = GroundPeer(args.robot_id, transport)
    try:
        peer.spin(print_hz=args.print_hz)
    finally:
        transport.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
