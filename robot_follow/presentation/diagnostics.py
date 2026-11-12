"""Console diagnostics derived from immutable frame snapshots."""
from dataclasses import dataclass
from typing import Any


@dataclass
class DebugFrame:
    fps: float
    track: dict | None
    following: bool
    visible: bool
    status: dict
    pinch_px: float | None
    gripper_deg: float
    grab_ms: float
    detect_ms: float
    processing_ms: float
    hardware_time: bool
    dropped_frames: int


def format_debug(frame: DebugFrame) -> str:
    track = frame.track
    det = 'None'
    if track is not None:
        frozen = '❄冻结' if track.get('z3d_frozen') and track['lost'] == 0 else ''
        bridged = '◦桥接' if track.get('bridged') else ''
        det = (f"u={track['u']:.0f} v={track['v']:.0f} "
               f"lost={track['lost']} z3d={track['z3d']:.3f}{frozen}{bridged}")
    pinch = (f'pinch={frame.pinch_px:.0f}px→{frame.gripper_deg:.1f}°'
             if frame.pinch_px is not None else 'pinch=None')
    status: dict[str, Any] = frame.status
    error = status.get('error', '')
    if frame.following and not status.get('ffm', True):
        error = f'{error}·静止' if error else '·静止'
    return (f'[DBG] fps={frame.fps:4.1f}  palm=({det})  visible={frame.visible} '
            f"following={frame.following}  joint_speed={status['joint_speed']:.3f}  "
            f"{pinch}  tcp={status.get('tcp')}  err={error}  "
            f'延迟ms(取帧/识别)={frame.grab_ms:.0f}/{frame.detect_ms:.0f} '
            f"观测={status.get('target_age_ms', 0.0):.0f}ms "
            f"lead={status.get('lead_ms', 0.0):.1f}/{status.get('lead_raw_ms', 0.0):.1f}ms "
            f"w={status.get('lead_weight', 0.0):.2f} kv={status.get('kf_speed', 0.0):.3f}m/s "
            f"e={status.get('lead_error', 0.0):.3f}m "
            f"latch={int(status.get('hold_latched', False))} "
            f"slow={int(status.get('slow_active', False))} "
            f"控制={status.get('ctrl_dt_ms', 0.0):.1f}/{status.get('ctrl_work_ms', 0.0):.1f}ms "
            f"超时={status.get('ctrl_overruns', 0)} seq={status.get('target_seq', 0)} "
            f'处理={frame.processing_ms:.1f}ms hw={int(frame.hardware_time)} '
            f'丢帧={frame.dropped_frames}')
