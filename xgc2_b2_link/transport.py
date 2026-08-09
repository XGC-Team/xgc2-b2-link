"""Explicit Zenoh or TCP framed transport for the frozen G3/G4 wire."""

from __future__ import annotations

import json
import socket
import struct
import threading
from abc import ABC, abstractmethod
from typing import Callable, Dict, List, Optional

OnMessage = Callable[[str, bytes], None]


class Transport(ABC):
    @abstractmethod
    def put(self, key: str, payload: bytes) -> None: ...

    @abstractmethod
    def subscribe(self, key_expr: str, handler: OnMessage) -> None: ...

    @abstractmethod
    def close(self) -> None: ...


def open_transport(
    *,
    kind: str,
    zenoh_mode: str = "peer",
    zenoh_listen: Optional[List[str]] = None,
    zenoh_connect: Optional[List[str]] = None,
    tcp_host: str = "127.0.0.1",
    tcp_port: int = 7448,
    tcp_role: str,
) -> Transport:
    """
    kind: zenoh | tcp
    tcp_role: server | client  (server=listen, client=connect)
    """
    kind = (kind or "").lower()
    if kind == "zenoh":
        listen = list(zenoh_listen or [])
        connect = list(zenoh_connect or [])
        if not listen and not connect:
            raise ValueError("zenoh transport requires an explicit listen or connect endpoint")
        return ZenohTransport(
            mode=zenoh_mode,
            listen=listen,
            connect=connect,
        )
    if kind == "tcp":
        return TcpFramedTransport(host=tcp_host, port=tcp_port, role=tcp_role)
    raise ValueError(f"unknown transport kind {kind}")


class ZenohTransport(Transport):
    def __init__(
        self,
        *,
        mode: str = "peer",
        listen: Optional[List[str]] = None,
        connect: Optional[List[str]] = None,
    ) -> None:
        import zenoh

        if mode not in {"peer", "client"}:
            raise ValueError(f"unsupported zenoh mode {mode}")
        for endpoint in [*(listen or []), *(connect or [])]:
            if not isinstance(endpoint, str) or not endpoint.startswith("tcp/"):
                raise ValueError(f"invalid explicit zenoh endpoint {endpoint!r}")
        conf = zenoh.Config()
        endpoints = {
            "mode": mode,
            "listen": {"endpoints": listen or []},
            "connect": {"endpoints": connect or []},
        }
        try:
            conf.insert_json5("mode", json.dumps(mode))
            if listen:
                conf.insert_json5("listen/endpoints", json.dumps(listen))
            if connect:
                conf.insert_json5("connect/endpoints", json.dumps(connect))
        except Exception as exc:
            raise RuntimeError("installed Zenoh cannot apply the frozen endpoint config") from exc
        self._zenoh = zenoh
        self._session = zenoh.open(conf)
        self._subs = []
        self._endpoints = endpoints

    def put(self, key: str, payload: bytes) -> None:
        self._session.put(key, payload)

    def subscribe(self, key_expr: str, handler: OnMessage) -> None:
        def _cb(sample) -> None:
            try:
                k = str(sample.key_expr)
                payload = bytes(sample.payload)
            except Exception:
                # older API
                k = str(getattr(sample, "key_expr", key_expr))
                raw = getattr(sample, "payload", b"")
                payload = bytes(raw) if not isinstance(raw, bytes) else raw
            handler(k, payload)

        sub = self._session.declare_subscriber(key_expr, _cb)
        self._subs.append(sub)

    def close(self) -> None:
        try:
            self._session.close()
        except Exception:
            pass


class TcpFramedTransport(Transport):
    """
    Simple multi-client hub:
    - server: listen, fan-out puts to all clients; deliver client frames to local handlers
    - client: connect to server
    Frame: u32be key_len | key_utf8 | u32be payload_len | payload
    """

    def __init__(
        self,
        *,
        host: str,
        port: int,
        role: str,
        reconnect_initial_delay: float = 0.1,
        reconnect_max_delay: float = 2.0,
        connect_timeout: float = 1.0,
    ) -> None:
        if reconnect_initial_delay <= 0:
            raise ValueError("reconnect_initial_delay must be positive")
        if reconnect_max_delay < reconnect_initial_delay:
            raise ValueError("reconnect_max_delay must not be smaller than the initial delay")
        if connect_timeout <= 0:
            raise ValueError("connect_timeout must be positive")
        self._host = host
        self._port = port
        self._role = role
        self._reconnect_initial_delay = reconnect_initial_delay
        self._reconnect_max_delay = reconnect_max_delay
        self._connect_timeout = connect_timeout
        self._handlers: List[tuple] = []
        self._lock = threading.Lock()
        self._peers: List[socket.socket] = []
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._accept_thread: Optional[threading.Thread] = None
        self._client_thread: Optional[threading.Thread] = None
        if role == "server":
            self._start_server()
        elif role == "client":
            self._start_client()
        else:
            raise ValueError("tcp role must be server or client")

    def _start_server(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self._host, self._port))
        s.listen(8)
        s.settimeout(0.5)
        self._server_sock = s
        self._accept_thread = threading.Thread(
            target=self._accept_loop, args=(s,), name="b2-tcp-accept", daemon=True
        )
        self._accept_thread.start()

    def _accept_loop(self, server_sock: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(0.5)
            with self._lock:
                self._peers.append(conn)
            threading.Thread(
                target=self._read_loop, args=(conn,), name="b2-tcp-read", daemon=True
            ).start()

    def _start_client(self) -> None:
        self._client_thread = threading.Thread(
            target=self._client_loop, name="b2-tcp-client", daemon=True
        )
        self._client_thread.start()
        # Preserve the old ready-server startup behavior without making a
        # temporarily unavailable server fatal. Later attempts stay in the
        # bounded-backoff client loop.
        self._connected.wait(self._connect_timeout + self._reconnect_initial_delay)

    def _client_loop(self) -> None:
        delay = self._reconnect_initial_delay
        while not self._stop.is_set():
            try:
                conn = socket.create_connection(
                    (self._host, self._port), timeout=self._connect_timeout
                )
                conn.settimeout(0.5)
            except OSError:
                if self._stop.wait(delay):
                    break
                delay = min(self._reconnect_max_delay, delay * 2)
                continue

            with self._lock:
                if self._stop.is_set():
                    self._close_socket(conn)
                    break
                self._client_sock = conn
                self._peers.append(conn)
                self._connected.set()

            delay = self._reconnect_initial_delay
            self._read_loop(conn)
            if self._stop.wait(delay):
                break

    def _read_loop(self, conn: socket.socket) -> None:
        try:
            while not self._stop.is_set():
                try:
                    header = self._recvexact(conn, 4)
                    if not header:
                        break
                    (klen,) = struct.unpack(">I", header)
                    key_raw = self._recvexact(conn, klen)
                    if len(key_raw) != klen:
                        break
                    key = key_raw.decode("utf-8")
                    payload_header = self._recvexact(conn, 4)
                    if len(payload_header) != 4:
                        break
                    (plen,) = struct.unpack(">I", payload_header)
                    payload = self._recvexact(conn, plen)
                    if len(payload) != plen:
                        break
                except socket.timeout:
                    continue
                except (OSError, struct.error, UnicodeDecodeError):
                    break
                self._dispatch(key, payload)
        finally:
            with self._lock:
                if conn in self._peers:
                    self._peers.remove(conn)
                if self._client_sock is conn:
                    self._client_sock = None
                    self._connected.clear()
            self._close_socket(conn)

    def _recvexact(self, conn: socket.socket, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return b""
            buf += chunk
        return buf

    def _dispatch(self, key: str, payload: bytes) -> None:
        with self._lock:
            handlers = list(self._handlers)
        for expr, handler in handlers:
            if _key_match(expr, key):
                try:
                    handler(key, payload)
                except Exception:
                    pass

    def put(self, key: str, payload: bytes) -> None:
        if self._stop.is_set():
            return
        frame = struct.pack(">I", len(key.encode("utf-8"))) + key.encode("utf-8")
        frame += struct.pack(">I", len(payload)) + payload
        dead = []
        with self._lock:
            peers = list(self._peers)
        for p in peers:
            try:
                p.sendall(frame)
            except OSError:
                dead.append(p)
        if dead:
            with self._lock:
                for p in dead:
                    if p in self._peers:
                        self._peers.remove(p)
                    if self._client_sock is p:
                        self._client_sock = None
                        self._connected.clear()
                    self._close_socket(p)

    def subscribe(self, key_expr: str, handler: OnMessage) -> None:
        with self._lock:
            self._handlers.append((key_expr, handler))

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            peers = list(self._peers)
            self._peers.clear()
            self._client_sock = None
            self._connected.clear()
            server_sock = self._server_sock
            self._server_sock = None
        for p in peers:
            self._close_socket(p)
        if server_sock:
            self._close_socket(server_sock)
        for thread in (self._client_thread, self._accept_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=self._connect_timeout + 1.0)

    @staticmethod
    def _close_socket(sock: socket.socket) -> None:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass


def _key_match(expr: str, key: str) -> bool:
    # minimal: exact, or prefix*  (zenoh-like trailing /**)
    if expr == key:
        return True
    if expr.endswith("/**"):
        return key.startswith(expr[:-3])
    if expr.endswith("/*"):
        prefix = expr[:-1]
        if not key.startswith(prefix):
            return False
        rest = key[len(prefix) :]
        return "/" not in rest.rstrip("/") or rest.count("/") == 0
    if "**" in expr or "*" in expr:
        # crude: prefix before first *
        head = expr.split("*", 1)[0]
        return key.startswith(head)
    return False
