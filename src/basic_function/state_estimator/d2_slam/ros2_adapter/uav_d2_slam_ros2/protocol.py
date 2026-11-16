"""Length-delimited local IPC shared by the ROS 1 proxy and ROS 2 adapter."""
from __future__ import annotations

import json
import socket
import struct


_PREFIX = struct.Struct("!II")
MAX_HEADER_BYTES = 1 << 20
MAX_PAYLOAD_BYTES = 64 << 20


def _receive_exact(stream: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.recv(size - len(chunks))
        if not chunk:
            raise EOFError("D2SLAM bridge disconnected")
        chunks.extend(chunk)
    return bytes(chunks)


def send_packet(stream: socket.socket, header: dict, payload: bytes = b"") -> None:
    encoded = json.dumps(header, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(encoded) > MAX_HEADER_BYTES or len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("D2SLAM bridge packet exceeds configured limit")
    stream.sendall(_PREFIX.pack(len(encoded), len(payload)) + encoded + payload)


def receive_packet(stream: socket.socket) -> tuple[dict, bytes]:
    header_size, payload_size = _PREFIX.unpack(_receive_exact(stream, _PREFIX.size))
    if header_size > MAX_HEADER_BYTES or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError("invalid D2SLAM bridge packet length")
    header = json.loads(_receive_exact(stream, header_size).decode("utf-8"))
    if not isinstance(header, dict) or not isinstance(header.get("kind"), str):
        raise ValueError("invalid D2SLAM bridge header")
    return header, _receive_exact(stream, payload_size)
