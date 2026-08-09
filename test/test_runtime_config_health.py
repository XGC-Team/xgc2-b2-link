import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import xgc2_b2_link.arm_forwarder_node as arm_forwarder_node
import xgc2_b2_link.forwarder_node as forwarder_node

from xgc2_b2_link.arm_forwarder_node import (
    _validate_source_environment,
    parse_args as parse_d17,
)
from xgc2_b2_link.forwarder_node import parse_args as parse_d0, run_ros_node as run_d0
from xgc2_b2_link.ground_peer import FRESHNESS_MS, GroundPeer, parse_args as parse_ground
from xgc2_b2_link.transport import open_transport


ROOT = Path(__file__).resolve().parents[1]


class FakeTransport:
    def subscribe(self, expression, callback):
        self.expression = expression
        self.callback = callback


def _peer_with_fresh_channels():
    peer = GroundPeer("b2-test", FakeTransport())
    now = int(time.monotonic() * 1000)
    for name in ("odom", "joint_states", "power_summary", "driver_status", "forwarder_hb"):
        body = {"received_monotonic_ms": now, "json": {}}
        if name == "driver_status":
            body["json"] = {"level": 0}
        peer.store.set(name, body)
    for name in ("arm_slave_status", "arm_forwarder_hb"):
        peer.store.set(name, {"received_monotonic_ms": now, "json": {}})
    return peer


def test_deploy_overlays_are_consumed():
    d0 = parse_d0(["--config", str(ROOT / "config/forwarder_d0.yaml")])
    d17 = parse_d17(["--config", str(ROOT / "config/forwarder_d17.yaml")])
    ground = parse_ground(["--config", str(ROOT / "config/ground_peer.yaml")])
    assert (d0.robot_id, d0.transport, d0.zenoh_connect) == (
        "b2-01", "zenoh", ["tcp/core:7447"]
    )
    assert (d0.zenoh_mode, d17.zenoh_mode, ground.zenoh_mode) == (
        "client", "client", "peer"
    )
    assert (d17.expected_rmw, d17.source_topic) == (
        "rmw_fastrtps_cpp", "/arm_slave_l_status"
    )
    assert (ground.tcp_role, ground.zenoh_listen) == (
        "server", ["tcp/0.0.0.0:7447"]
    )


def test_formal_forwarders_resolve_packaged_defaults_without_install_prefix(monkeypatch):
    monkeypatch.setattr(
        forwarder_node,
        "installed_runtime_config",
        lambda filename: str(ROOT / "config" / filename),
    )
    monkeypatch.setattr(
        arm_forwarder_node,
        "installed_runtime_config",
        lambda filename: str(ROOT / "config" / filename),
    )
    d0 = parse_d0([])
    d17 = parse_d17([])
    assert d0.config == str(ROOT / "config/forwarder_d0.yaml")
    assert d17.config == str(ROOT / "config/forwarder_d17.yaml")


def test_cli_endpoint_replaces_deploy_overlay_instead_of_joining_two_fabrics():
    d0 = parse_d0(
        [
            "--config",
            str(ROOT / "config/forwarder_d0.yaml"),
            "--zenoh-connect",
            "tcp/192.168.30.1:7447",
        ]
    )
    assert d0.zenoh_connect == ["tcp/192.168.30.1:7447"]
    assert d0.zenoh_listen == []


def test_arm_stale_does_not_take_base_offline():
    peer = _peer_with_fresh_channels()
    old = int(time.monotonic() * 1000) - FRESHNESS_MS["arm_slave_status"] - 1
    peer.store.set("arm_slave_status", {"received_monotonic_ms": old, "json": {}})
    health = peer.health_snapshot()
    assert health["base_online"] is True
    assert health["arm_seen"] is True
    assert health["arm_ready"] is False


def test_stale_base_source_takes_only_base_offline():
    peer = _peer_with_fresh_channels()
    old = int(time.monotonic() * 1000) - FRESHNESS_MS["odom"] - 1
    peer.store.set("odom", {"received_monotonic_ms": old, "json": {}})
    health = peer.health_snapshot()
    assert health["base_online"] is False
    assert health["arm_ready"] is True


def test_domain_zero_forwarder_fails_before_ros_import_on_wrong_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "17")
    assert run_d0(SimpleNamespace()) == 2


def test_domain_seventeen_requires_status_topic_domain_and_rmw(monkeypatch):
    args = SimpleNamespace(
        source_topic="/arm_slave_l_status", expected_rmw="rmw_fastrtps_cpp"
    )
    monkeypatch.setenv("ROS_DOMAIN_ID", "17")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    _validate_source_environment(args)
    args.source_topic = "/arm_cmd"
    with pytest.raises(ValueError, match="read-only status topic"):
        _validate_source_environment(args)


def test_zenoh_requires_an_explicit_endpoint_before_importing_backend():
    with pytest.raises(ValueError, match="explicit listen or connect endpoint"):
        open_transport(kind="zenoh", tcp_role="client")


def test_zenoh_applies_only_the_explicit_connect_endpoint(monkeypatch):
    calls = []

    class FakeConfig:
        def insert_json5(self, key, value):
            calls.append((key, value))

    class FakeSession:
        def close(self):
            calls.append(("close", ""))

    monkeypatch.setitem(
        sys.modules,
        "zenoh",
        SimpleNamespace(Config=FakeConfig, open=lambda _config: FakeSession()),
    )
    transport = open_transport(
        kind="zenoh",
        zenoh_mode="peer",
        zenoh_connect=["tcp/core:7447"],
        tcp_role="client",
    )
    transport.close()
    assert calls[:2] == [
        ("mode", '"peer"'),
        ("connect/endpoints", '["tcp/core:7447"]'),
    ]
    assert not any(key == "listen/endpoints" for key, _value in calls)


def test_zenoh_config_api_failure_is_not_silently_ignored(monkeypatch):
    class BrokenConfig:
        def insert_json5(self, _key, _value):
            raise RuntimeError("unsupported config API")

    monkeypatch.setitem(
        sys.modules,
        "zenoh",
        SimpleNamespace(Config=BrokenConfig, open=lambda _config: object()),
    )
    with pytest.raises(RuntimeError, match="cannot apply the frozen endpoint config"):
        open_transport(
            kind="zenoh",
            zenoh_connect=["tcp/core:7447"],
            tcp_role="client",
        )
