"""Small deploy-overlay loader shared by the G3/G4 executables."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml


def installed_runtime_config(filename: str) -> str:
    """Resolve a packaged config through the ROS 2 package index.

    Production callers never guess an install prefix or a deployment user's
    home. An explicit --config remains available for tests and operator-owned
    overrides, while the formal APT process uses this installed default.
    """

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise ValueError("ROS 2 package index is unavailable") from exc
    path = Path(get_package_share_directory("xgc2_b2_link")) / "config" / filename
    if not path.is_file():
        raise ValueError(f"installed runtime config is unavailable: {filename}")
    return str(path)


def load_runtime_defaults(path: str, role: str) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"runtime config must be a mapping: {path}")

    defaults: Dict[str, Any] = {}
    for name in ("robot_id", "transport", "source_topic", "expected_rmw"):
        if name in raw:
            defaults[name] = raw[name]
    if "rmw_implementation" in raw:
        defaults["expected_rmw"] = raw["rmw_implementation"]

    tcp = raw.get("tcp") or {}
    zenoh = raw.get("zenoh") or {}
    if not isinstance(tcp, dict) or not isinstance(zenoh, dict):
        raise ValueError("tcp and zenoh config entries must be mappings")
    if "host" in tcp:
        defaults["tcp_host"] = tcp["host"]
    if "port" in tcp:
        defaults["tcp_port"] = int(tcp["port"])
    if "role" in tcp:
        defaults["tcp_role"] = tcp["role"]
    if "listen" in zenoh:
        defaults["zenoh_listen"] = list(zenoh["listen"] or [])
    if "connect" in zenoh:
        defaults["zenoh_connect"] = list(zenoh["connect"] or [])
    if "mode" in zenoh:
        defaults["zenoh_mode"] = str(zenoh["mode"])

    if role == "d0":
        defaults["enable_imu"] = bool(raw.get("enable_imu", False))
        defaults["enable_odin_odom"] = bool(raw.get("enable_odin_odom", False))
    return defaults


def require_identity(args: Any) -> None:
    missing = [name for name in ("robot_id", "transport") if not getattr(args, name, None)]
    if missing:
        raise ValueError("missing required setting(s): " + ", ".join(missing))
