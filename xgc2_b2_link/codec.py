"""Portable JSON codecs shared by the G3 forwarders and G4 adapter."""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence


def now_ms() -> int:
    return int(time.time() * 1000)


def pack_json(obj: Mapping[str, Any]) -> bytes:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def unpack_json(blob: bytes) -> Dict[str, Any]:
    data = json.loads(blob.decode("utf-8"))
    if not isinstance(data, dict):
        raise ValueError("json payload must be object")
    return data


def _stamp_ms(message: Any) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return now_ms()
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    value = sec * 1000 + nanosec // 1_000_000
    return value if value > 0 else now_ms()


def _frame_id(message: Any) -> str:
    return str(getattr(getattr(message, "header", None), "frame_id", "") or "")


def _vector3(value: Any) -> Dict[str, float]:
    return {
        "x": float(getattr(value, "x", 0.0)),
        "y": float(getattr(value, "y", 0.0)),
        "z": float(getattr(value, "z", 0.0)),
    }


def _quaternion(value: Any) -> Dict[str, float]:
    return {
        "x": float(getattr(value, "x", 0.0)),
        "y": float(getattr(value, "y", 0.0)),
        "z": float(getattr(value, "z", 0.0)),
        "w": float(getattr(value, "w", 1.0)),
    }


def odom_json(message: Any) -> Dict[str, Any]:
    pose = getattr(getattr(message, "pose", None), "pose", None)
    twist = getattr(getattr(message, "twist", None), "twist", None)
    return {
        "v": 1,
        "t_ms": _stamp_ms(message),
        "frame_id": _frame_id(message),
        "child_frame_id": str(getattr(message, "child_frame_id", "") or ""),
        "position": _vector3(getattr(pose, "position", None)),
        "orientation": _quaternion(getattr(pose, "orientation", None)),
        "linear": _vector3(getattr(twist, "linear", None)),
        "angular": _vector3(getattr(twist, "angular", None)),
    }


def joint_states_json(
    message: Any,
    *,
    names: Optional[Sequence[str]] = None,
    frame_id: Optional[str] = None,
) -> Dict[str, Any]:
    resolved_names = [str(value) for value in (names if names is not None else message.name)]
    count = len(resolved_names)

    def bounded(values: Optional[Iterable[Any]]) -> List[float]:
        result = [float(value) for value in (values or [])][:count]
        result.extend([0.0] * (count - len(result)))
        return result

    return {
        "v": 1,
        "t_ms": _stamp_ms(message),
        "frame_id": _frame_id(message) if frame_id is None else frame_id,
        "names": resolved_names,
        "positions": bounded(getattr(message, "position", None)),
        "velocities": bounded(getattr(message, "velocity", None)),
        "efforts": bounded(getattr(message, "effort", None)),
    }


def arm_joint_states_json(message: Any, names: Sequence[str]) -> Dict[str, Any]:
    """Convert the seven-value R5 status into six arm + two gripper joints."""

    positions = [float(value) for value in getattr(message, "joint_pos", [])]
    velocities = [float(value) for value in getattr(message, "joint_vel", [])]
    efforts = [float(value) for value in getattr(message, "joint_cur", [])]

    def expand(values: List[float]) -> List[float]:
        values = values[:7]
        values.extend([0.0] * (7 - len(values)))
        return values[:6] + [values[6], values[6]]

    return {
        "v": 1,
        "t_ms": _stamp_ms(message),
        "frame_id": _frame_id(message),
        "names": [str(name) for name in names],
        "positions": expand(positions),
        "velocities": expand(velocities),
        "efforts": expand(efforts),
    }


def imu_json(message: Any) -> Dict[str, Any]:
    return {
        "v": 1,
        "t_ms": _stamp_ms(message),
        "frame_id": _frame_id(message),
        "orientation": _quaternion(getattr(message, "orientation", None)),
        "angular_velocity": _vector3(getattr(message, "angular_velocity", None)),
        "linear_acceleration": _vector3(getattr(message, "linear_acceleration", None)),
    }


def driver_status_json(message: Any) -> Dict[str, Any]:
    statuses = list(getattr(message, "status", []) or [])

    def status_level(status: Any) -> int:
        value = getattr(status, "level", 0)
        if isinstance(value, (bytes, bytearray, memoryview)):
            if len(value) != 1:
                raise ValueError("diagnostic status level must contain exactly one byte")
            return value[0]
        return int(value)

    level = max((status_level(status) for status in statuses), default=0)
    summaries = [str(getattr(status, "message", "") or "") for status in statuses]
    summaries = [summary for summary in summaries if summary]
    values: Dict[str, str] = {}
    faults: List[str] = []
    for status in statuses:
        if status_level(status) >= 2:
            name = str(getattr(status, "name", "driver") or "driver")
            message_text = str(getattr(status, "message", "") or "")
            faults.append(f"{name}: {message_text}".rstrip())
        for item in getattr(status, "values", []) or []:
            values[str(getattr(item, "key", ""))] = str(getattr(item, "value", ""))

    def bool_value(name: str, default: bool) -> bool:
        raw = values.get(name)
        if raw is None:
            return default
        return raw.strip().lower() in {"1", "true", "yes", "on"}

    return {
        "v": 1,
        "t_ms": _stamp_ms(message),
        "level": level,
        "summary": "; ".join(summaries) or ("OK" if level == 0 else "driver status"),
        "motion_enabled": bool_value("motion_enabled", False),
        "command_stale": bool_value("command_stale", True),
        "faults": faults,
    }


def power_summary_from_low_state_fields(
    *,
    soc: Any = None,
    power_v: Any = None,
    power_a: Any = None,
    temperature_ntc1: Any = None,
    temperature_ntc2: Any = None,
    t_ms: Optional[int] = None,
) -> Dict[str, Any]:
    return {
        "v": 1,
        "t_ms": t_ms if t_ms is not None else now_ms(),
        "soc": soc,
        "power_v": power_v,
        "power_a": power_a,
        "temperature_ntc1": temperature_ntc1,
        "temperature_ntc2": temperature_ntc2,
    }


def power_summary_from_battery_message(message: Any) -> Dict[str, Any]:
    """Normalize the portable ROS BatteryState LAB/standard source."""

    def finite_or_none(value: Any) -> Optional[float]:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if math.isfinite(parsed) else None

    percentage = finite_or_none(getattr(message, "percentage", None))
    soc = None
    if percentage is not None and percentage >= 0.0:
        soc = int(round(max(0.0, min(1.0, percentage)) * 100.0))
    temperature = finite_or_none(getattr(message, "temperature", None))
    return power_summary_from_low_state_fields(
        soc=soc,
        power_v=finite_or_none(getattr(message, "voltage", None)),
        power_a=finite_or_none(getattr(message, "current", None)),
        temperature_ntc1=temperature,
        temperature_ntc2=None,
        t_ms=_stamp_ms(message),
    )


def forwarder_heartbeat(
    *,
    robot_id: str,
    domain: int,
    transport: str,
    uptime_ms: int,
    channels: Sequence[str],
    stats: Mapping[str, Mapping[str, Any]],
    t_ms: Optional[int] = None,
) -> Dict[str, Any]:
    return {
        "v": 1,
        "t_ms": t_ms if t_ms is not None else now_ms(),
        "robot_id": robot_id,
        "domain": int(domain),
        "channels": list(channels),
        "transport": transport,
        "uptime_ms": max(0, int(uptime_ms)),
        "stats": {name: dict(value) for name, value in stats.items()},
    }


def validate_payload(schema: str, payload: Mapping[str, Any]) -> None:
    required = {
        "odom_v1": {
            "v",
            "t_ms",
            "frame_id",
            "child_frame_id",
            "position",
            "orientation",
            "linear",
            "angular",
        },
        "joint_states_v1": {
            "v",
            "t_ms",
            "frame_id",
            "names",
            "positions",
            "velocities",
            "efforts",
        },
        "imu_v1": {
            "v",
            "t_ms",
            "frame_id",
            "orientation",
            "angular_velocity",
            "linear_acceleration",
        },
        "arm_joint_states_v1": {
            "v",
            "t_ms",
            "frame_id",
            "names",
            "positions",
            "velocities",
            "efforts",
        },
        "power_summary_v1": {
            "v",
            "t_ms",
            "soc",
            "power_v",
            "power_a",
            "temperature_ntc1",
            "temperature_ntc2",
        },
        "driver_status_v1": {
            "v",
            "t_ms",
            "level",
            "summary",
            "motion_enabled",
            "command_stale",
            "faults",
        },
        "forwarder_hb_v1": {
            "v",
            "t_ms",
            "robot_id",
            "domain",
            "channels",
            "transport",
            "uptime_ms",
            "stats",
        },
    }.get(schema)
    if required is None:
        raise ValueError(f"unsupported JSON schema {schema}")
    missing = sorted(required.difference(payload))
    if missing:
        raise ValueError(f"{schema} missing fields: {', '.join(missing)}")
    if schema in {"joint_states_v1", "arm_joint_states_v1"}:
        lengths = {
            len(payload.get("names", [])),
            len(payload.get("positions", [])),
            len(payload.get("velocities", [])),
            len(payload.get("efforts", [])),
        }
        if len(lengths) != 1:
            raise ValueError(f"{schema} joint arrays must have equal lengths")
