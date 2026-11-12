"""Coordinate camera recovery and reset dependent perception state."""
import time


def reconnect_camera(session, pipeline, timestamp_calibrator, observation, lock,
                     depth_filter_factory):
    """Replace a failed camera; discard its stream if reset cannot complete."""
    try:
        session.stop()
    except Exception:
        pass
    time.sleep(1.0)
    try:
        session.open()
        pipeline.reset(depth_filter_factory())
        timestamp_calibrator.reset()
        with lock:
            observation.visible = False
            observation.position_cam = None
        print('[相机] 重连成功')
        return True
    except Exception as exc:
        try:
            session.stop()
        except Exception:
            pass
        print(f'[相机] 重连失败（{exc}），2 秒后重试...')
        return False
