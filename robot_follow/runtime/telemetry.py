"""Low-priority, latest-only telemetry publisher for the local web service."""
from __future__ import annotations

import json
import os
import queue
import socket
import threading


class LiveTelemetryPublisher:
    """Queue snapshots without serializing or doing socket I/O in the servo loop."""

    def __init__(self, port: int, token: str, period: float = 0.05):
        if not 0 < port < 65536 or not token or period <= 0:
            raise ValueError('invalid telemetry destination')
        self._address = ('127.0.0.1', port)
        self._token = token
        self._period = period
        self._next_send = 0.0
        self._queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='live-telemetry', daemon=True)

    @classmethod
    def from_environment(cls):
        port = os.environ.get('REBOT_TELEMETRY_PORT')
        token = os.environ.get('REBOT_TELEMETRY_TOKEN')
        if not port or not token:
            return None
        return cls(int(port), token)

    def start(self):
        self._thread.start()
        return self

    def due(self, now: float) -> bool:
        if now < self._next_send:
            return False
        self._next_send = now + self._period
        return True

    def offer(self, row: dict) -> None:
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            # Keep the newest sample without ever waiting for the consumer.
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(row)
            except queue.Full:
                pass

    def _run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.setblocking(False)
                while not self._stop.is_set():
                    try:
                        row = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    try:
                        payload = json.dumps({'token': self._token,
                                              'row': {k: str(v) for k, v in row.items()}},
                                             ensure_ascii=False).encode('utf-8')
                        sock.sendto(payload, self._address)
                    except (OSError, TypeError, ValueError):
                        # Telemetry is advisory and must never stop the controller.
                        continue
        except OSError:
            return

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
