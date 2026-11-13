"""Aggregate frame-level depth outcomes without recording images."""
from collections import Counter


class DepthDiagnostics:
    def __init__(self, now, interval_s=5.0):
        self.started = now
        self.interval_s = interval_s
        self.counts = Counter()

    def record(self, track, now, *, no_frame=False):
        if no_frame:
            reason = '相机无帧'
        elif track is None:
            reason = '未检测到手'
        else:
            reason = track.get('depth_status', '未知')
        self.counts[reason] += 1
        if now - self.started < self.interval_s:
            return None
        counts = ' '.join(f'{key}={value}' for key, value in sorted(self.counts.items()))
        message = f'[深度诊断] {now - self.started:.1f}秒 帧/尝试={sum(self.counts.values())} {counts}'
        self.started = now
        self.counts.clear()
        return message
