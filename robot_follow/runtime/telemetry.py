"""Low-priority, latest-only telemetry publisher for the local web service."""
from __future__ import annotations

import json
import os
import queue
import socket
import threading
import time
import math


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


class PhaseTelemetry:
    """Publish command angles during HOME/return without extra CAN reads."""

    def __init__(self, robot, publisher, period=0.05):
        self.robot, self.publisher, self.period = robot, publisher, period
        self._phase = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='phase-telemetry', daemon=True)

    def start(self):
        self._thread.start()
        return self

    def set_phase(self, phase):
        if phase not in (None, 'home', 'return'):
            raise ValueError('invalid movement phase')
        with self._lock:
            self._phase = phase

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                phase = self._phase
            if phase:
                try:
                    joints = [float(q) for q in self.robot.get_command_pos()]
                    if len(joints) == 6 and all(math.isfinite(q) for q in joints):
                        row = {'host_time': time.monotonic(), 'phase': phase,
                               'q_cmd_valid': 1, 'fb_valid': 0,
                               'following': 0, 'visible': 0, 'hold_latch': 0}
                        row.update({f'q_cmd_{i + 1}': q for i, q in enumerate(joints)})
                        # A phase transition may occur while reading angles.
                        # Never publish a stale HOME sample after servo starts.
                        with self._lock:
                            if self._phase == phase:
                                self.publisher.offer(row)
                except (AttributeError, TypeError, ValueError, RuntimeError):
                    pass
            self._stop.wait(self.period)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
