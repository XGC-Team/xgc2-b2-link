"""G3 dry-run forwarder ↔ G4 ground peer over TCP framed transport."""

import subprocess
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from xgc2_b2_link.transport import TcpFramedTransport


def _available_tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _peer_count(transport):
    with transport._lock:
        return len(transport._peers)


def test_tcp_g3_g4_loop():
    py = sys.executable
    env = {**dict(**__import__("os").environ), "PYTHONPATH": str(ROOT)}
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        tcp_port = probe.getsockname()[1]
    ground = subprocess.Popen(
        [
            py,
            "-m",
            "xgc2_b2_link.ground_peer",
            "--config",
            str(ROOT / "config/ground_peer.yaml"),
            "--robot-id",
            "b2-test",
            "--transport",
            "tcp",
            "--tcp-host",
            "127.0.0.1",
            "--tcp-role",
            "server",
            "--tcp-port",
            str(tcp_port),
            "--print-hz",
            "2",
        ],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    time.sleep(0.4)
    fwd = subprocess.Popen(
        [
            py,
            "-m",
            "xgc2_b2_link.forwarder_node",
            "--config",
            str(ROOT / "config/forwarder_d0.yaml"),
            "--robot-id",
            "b2-test",
            "--transport",
            "tcp",
            "--tcp-host",
            "127.0.0.1",
            "--tcp-role",
            "client",
            "--tcp-port",
            str(tcp_port),
            "--dry-run-no-ros",
        ],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    arm = subprocess.Popen(
        [
            py,
            "-m",
            "xgc2_b2_link.arm_forwarder_node",
            "--config",
            str(ROOT / "config/forwarder_d17.yaml"),
            "--robot-id",
            "b2-test",
            "--transport",
            "tcp",
            "--tcp-host",
            "127.0.0.1",
            "--tcp-role",
            "client",
            "--tcp-port",
            str(tcp_port),
            "--dry-run-no-ros",
        ],
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        # wait for forwarder dry-run to finish publishing
        fwd_out, _ = fwd.communicate(timeout=10)
        arm_out, _ = arm.communicate(timeout=10)
        time.sleep(0.8)
        ground.terminate()
        g_out, _ = ground.communicate(timeout=5)
    finally:
        if fwd.poll() is None:
            fwd.kill()
        if arm.poll() is None:
            arm.kill()
        if ground.poll() is None:
            ground.kill()

    assert "dry-run done" in (fwd_out or "")
    assert "dry-run d17 done" in (arm_out or "")
    # ground should have printed channel summaries
    assert '"base_online": true' in (g_out or "")
    assert '"arm_ready": true' in (g_out or "")
    assert "power_summary" in (g_out or "") or "odom" in (g_out or "")
    assert "decode_errors\": 0" in (g_out or "")


def test_tcp_client_recovers_after_server_restart_without_new_instance():
    port = _available_tcp_port()
    received = []
    received_event = threading.Event()
    client = TcpFramedTransport(
        host="127.0.0.1",
        port=port,
        role="client",
        reconnect_initial_delay=0.02,
        reconnect_max_delay=0.05,
        connect_timeout=0.05,
    )
    server = TcpFramedTransport(host="127.0.0.1", port=port, role="server")

    def record(key, payload):
        received.append((key, payload))
        received_event.set()

    server.subscribe("xgc2/test/**", record)
    replacement = None
    try:
        assert _wait_until(lambda: _peer_count(client) == 1)
        client.put("xgc2/test/status", b"before-restart")
        assert received_event.wait(1.0)
        assert received[-1] == ("xgc2/test/status", b"before-restart")

        server.close()
        assert _wait_until(lambda: _peer_count(client) == 0)
        received_event.clear()
        received_before_outage = len(received)
        client.put("xgc2/test/status", b"during-outage")
        time.sleep(0.1)
        assert len(received) == received_before_outage

        replacement = TcpFramedTransport(host="127.0.0.1", port=port, role="server")
        replacement.subscribe("xgc2/test/**", record)
        assert _wait_until(lambda: _peer_count(client) == 1)
        client.put("xgc2/test/status", b"after-restart")
        assert received_event.wait(1.0)
        assert received[-1] == ("xgc2/test/status", b"after-restart")
    finally:
        client.close()
        server.close()
        if replacement is not None:
            replacement.close()


def test_tcp_client_close_stops_reconnect_loop():
    port = _available_tcp_port()
    server = TcpFramedTransport(host="127.0.0.1", port=port, role="server")
    client = TcpFramedTransport(
        host="127.0.0.1",
        port=port,
        role="client",
        reconnect_initial_delay=0.02,
        reconnect_max_delay=0.05,
        connect_timeout=0.05,
    )
    try:
        assert _wait_until(lambda: _peer_count(client) == 1)
        server.close()
        assert _wait_until(lambda: _peer_count(client) == 0)
        client.close()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", port))
            listener.listen(1)
            listener.settimeout(0.2)
            with pytest.raises(socket.timeout):
                listener.accept()
    finally:
        client.close()
        server.close()
