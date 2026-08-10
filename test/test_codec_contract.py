import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from xgc2_b2_link.codec import (
    arm_joint_states_json,
    driver_status_json,
    forwarder_heartbeat,
    joint_states_json,
    odom_json,
    pack_json,
    power_summary_from_low_state_fields,
    power_summary_from_battery_message,
    unpack_json,
    validate_payload,
)
from xgc2_b2_link import contract as contract_module
from xgc2_b2_link.contract import contract_path, full_key, load_contract
from xgc2_b2_link.forwarder_node import ForwarderCore
from xgc2_b2_link.rate import RateGate
from xgc2_b2_link.sim_models import ARM_URDF_JOINTS, BASE_FRAME


def header():
    return SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2_000_000), frame_id="odom")


def test_contract_is_json_only_and_command_is_reserved():
    contract = load_contract()
    assert contract["version"] == 1
    assert all(spec["encode"] == "json_utf8" for spec in contract["channels"]["up"].values())
    assert contract["channels"]["down"]["cmd"]["authorized"] is False
    assert contract["channels"]["up"]["arm_forwarder_hb"]["key"] == "up/arm_forwarder_hb"
    zenoh = contract["transport"]["zenoh"]
    assert zenoh["forwarder"] == {
        "mode": "client",
        "connect": ["tcp/core:7447"],
        "listen": [],
    }
    assert zenoh["ground_adapter"] == {
        "mode": "peer",
        "connect": [],
        "listen": ["tcp/0.0.0.0:7447"],
    }


def test_installed_contract_resolves_through_ros_package_share(monkeypatch, tmp_path):
    installed_module = tmp_path / "lib" / "python3.12" / "site-packages" / "xgc2_b2_link" / "contract.py"
    installed_module.parent.mkdir(parents=True)
    installed_module.touch()
    package_share = tmp_path / "share" / "xgc2_b2_link"
    expected = package_share / "contract" / "zenoh_v1.yaml"
    expected.parent.mkdir(parents=True)
    expected.write_text("version: 1\n", encoding="utf-8")

    monkeypatch.delenv("XGC2_B2_LINK_CONTRACT", raising=False)
    monkeypatch.setattr(contract_module, "__file__", str(installed_module))
    monkeypatch.setattr(
        contract_module,
        "get_package_share_directory",
        lambda package: str(package_share) if package == "xgc2_b2_link" else "",
    )

    assert contract_path() == expected
    assert load_contract()["version"] == 1


def test_keys():
    assert full_key("b2-01", "up/odom") == "xgc2/b2-01/up/odom"


def test_odom_json_contract():
    vector = SimpleNamespace(x=1.0, y=2.0, z=3.0)
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    message = SimpleNamespace(
        header=header(),
        child_frame_id="b2_description",
        pose=SimpleNamespace(pose=SimpleNamespace(position=vector, orientation=quaternion)),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=vector, angular=vector)),
    )
    payload = odom_json(message)
    validate_payload("odom_v1", payload)
    assert payload["t_ms"] == 1002
    assert payload["position"]["z"] == 3.0


def test_joint_json_pads_arrays_and_validates_equal_lengths():
    message = SimpleNamespace(
        header=header(), name=["a", "b"], position=[1.0], velocity=[], effort=[]
    )
    payload = joint_states_json(message)
    validate_payload("joint_states_v1", payload)
    assert payload["positions"] == [1.0, 0.0]
    payload["efforts"] = []
    with pytest.raises(ValueError, match="equal lengths"):
        validate_payload("joint_states_v1", payload)


def test_forwarder_normalizes_empty_ros_joint_frame_to_frozen_b2_root():
    class CaptureTransport:
        def __init__(self):
            self.frames = []

        def put(self, key, payload):
            self.frames.append((key, unpack_json(payload)))

    transport = CaptureTransport()
    forwarder = ForwarderCore("b2-01", transport, transport_name="tcp")
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id=""),
        name=["FR_hip_joint"],
        position=[0.0],
        velocity=[0.0],
        effort=[0.0],
    )

    assert forwarder.publish_joint_states(message) is True
    assert transport.frames[0][0] == "xgc2/b2-01/up/joint_states"
    assert transport.frames[0][1]["frame_id"] == BASE_FRAME


def test_arm_status_expands_one_gripper_value_to_two_urdf_joints():
    message = SimpleNamespace(
        header=header(), joint_pos=list(range(7)), joint_vel=[0.0] * 7, joint_cur=[0.0] * 7
    )
    payload = arm_joint_states_json(message, ARM_URDF_JOINTS)
    validate_payload("arm_joint_states_v1", payload)
    assert len(payload["names"]) == 8
    assert payload["positions"][-2:] == [6.0, 6.0]


def test_power_summary_json():
    payload = power_summary_from_low_state_fields(soc=42, power_v=47.5, power_a=-1.0, t_ms=1)
    payload["temperature_ntc1"] = None
    payload["temperature_ntc2"] = None
    validate_payload("power_summary_v1", payload)
    assert unpack_json(pack_json(payload))["soc"] == 42


def test_standard_battery_state_normalizes_without_field_driver_messages():
    message = SimpleNamespace(
        header=header(), percentage=0.73, voltage=47.5, current=-1.0, temperature=35.0
    )
    payload = power_summary_from_battery_message(message)
    validate_payload("power_summary_v1", payload)
    assert payload["soc"] == 73
    assert payload["power_v"] == 47.5


def test_driver_status_accepts_jazzy_uint8_byte_constants():
    message = SimpleNamespace(
        header=header(),
        status=[SimpleNamespace(level=b"\x00", message="ready", name="driver", values=[])],
    )
    payload = driver_status_json(message)
    validate_payload("driver_status_v1", payload)
    assert payload["level"] == 0
    assert payload["summary"] == "ready"


def test_heartbeat_has_bounded_channel_stats():
    payload = forwarder_heartbeat(
        robot_id="b2-01",
        domain=17,
        transport="tcp",
        uptime_ms=123,
        channels=["arm_slave_status"],
        stats={"arm_slave_status": {"rx_count": 1, "tx_count": 1}},
        t_ms=9,
    )
    validate_payload("forwarder_hb_v1", payload)
    assert payload["domain"] == 17
    assert payload["transport"] == "tcp"


def test_rate_gate():
    gate = RateGate(10.0)
    assert gate.allow(0.0) is True
    assert gate.allow(0.01) is False
    assert gate.allow(0.11) is True
