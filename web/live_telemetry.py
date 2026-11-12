"""Receive session-scoped telemetry from the managed controller."""
import json
import math
import secrets
import socket
import time


class LiveTelemetryReceiver:
    def __init__(self, token):
        self.token = token
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(('127.0.0.1', 0))
            self.socket.setblocking(False)
        except OSError:
            self.socket.close()
            raise
        self.port = self.socket.getsockname()[1]
        self.row = None
        self.sampled_at = None

    def latest(self):
        # Bounded work even if a local sender floods the port.
        for _ in range(128):
            try:
                payload, _ = self.socket.recvfrom(32768)
            except BlockingIOError:
                break
            except OSError:
                return None
            try:
                message = json.loads(payload.decode('utf-8'))
                token = message.get('token')
                row = message.get('row')
                if not isinstance(token, str) or not secrets.compare_digest(token, self.token):
                    continue
                if not isinstance(row, dict) or not all(isinstance(k, str) and isinstance(v, str)
                                                        for k, v in row.items()):
                    continue
                sampled_at = float(row['host_time'])
                age = time.monotonic() - sampled_at
                if not math.isfinite(age) or age < 0 or age > 3.0:
                    continue
                if self.sampled_at is not None and sampled_at <= self.sampled_at:
                    continue
            except (UnicodeError, ValueError, AttributeError, TypeError, KeyError):
                continue
            self.row = row
            self.sampled_at = sampled_at
        if self.row is None:
            return None
        age = max(0.0, time.monotonic() - self.sampled_at)
        if age > 3.0:
            return None
        return self.row, age

    def close(self):
        self.socket.close()
