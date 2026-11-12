"""Load gripper mapping settings independently of the runtime."""

import math
import os
from dataclasses import dataclass

import yaml


def load_grasp_cfg(path):
    """读取捏合→夹爪映射配置，缺失字段给安全默认值。"""
    cfg = {"pinch": {"closed_px": 20.0, "open_px": 200.0,
                     "filter": "one_euro",
                     "oe_min_cutoff": 1.2, "oe_beta": 0.05,
                     "smoothing": 0.35, "deadband_deg": 2.0},
           "gripper": {"enabled": True, "closed_deg": -19.7, "open_deg": 293.6,
                       "hold_while_moving": True, "move_speed_threshold": 0.05}}
    if not os.path.exists(path):
        print(f"⚠️ 未找到 {path}，使用内置默认映射")
        return cfg
    with open(path, "r", encoding="utf-8") as f:
        user = yaml.safe_load(f) or {}
    for section, defaults in cfg.items():
        if isinstance(user.get(section), dict):
            defaults.update(user[section])
    # 兼容两套键名：filter_type / min_cutoff_hz / beta / dcutoff_hz
    p = cfg["pinch"]
    if "filter_type" in p:
        p["filter"] = p["filter_type"]
    if "min_cutoff_hz" in p:
        p["oe_min_cutoff"] = p["min_cutoff_hz"]
    if "beta" in p:
        p["oe_beta"] = p["beta"]
    return cfg




@dataclass(frozen=True)
class GripperSettings:
    closed_px: float
    open_px: float
    smoothing: float
    deadband_deg: float
    filter_mode: str
    closed_deg: float
    open_deg: float
    hold_while_moving: bool
    move_speed_threshold: float
    min_cutoff: float
    beta: float

    @classmethod
    def from_config(cls, config):
        pinch = config['pinch']
        gripper = config['gripper']
        settings = cls(
            closed_px=float(pinch['closed_px']), open_px=float(pinch['open_px']),
            smoothing=float(pinch['smoothing']), deadband_deg=float(pinch['deadband_deg']),
            filter_mode=str(pinch.get('filter', 'one_euro')).lower(),
            closed_deg=float(gripper['closed_deg']), open_deg=float(gripper['open_deg']),
            hold_while_moving=bool(gripper.get('hold_while_moving', True)),
            move_speed_threshold=float(gripper.get('move_speed_threshold', .05)),
            min_cutoff=float(pinch.get('oe_min_cutoff', 1.2)),
            beta=float(pinch.get('oe_beta', .05)))
        numbers = (settings.closed_px, settings.open_px, settings.smoothing,
                   settings.deadband_deg, settings.closed_deg, settings.open_deg,
                   settings.move_speed_threshold, settings.min_cutoff, settings.beta)
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError('grasp.yaml 配置错误: 数值必须有限')
        if settings.open_px <= settings.closed_px:
            raise ValueError('grasp.yaml 配置错误: pinch.open_px 必须大于 pinch.closed_px')
        if abs(settings.open_deg - settings.closed_deg) < 5.0:
            raise ValueError('grasp.yaml 配置错误: 夹爪开合区间过小，请重新标定')
        if not 0 <= settings.smoothing <= 1 or settings.deadband_deg < 0:
            raise ValueError('grasp.yaml 配置错误: 平滑系数或死区不合法')
        if settings.move_speed_threshold < 0 or settings.min_cutoff <= 0 or settings.beta < 0:
            raise ValueError('grasp.yaml 配置错误: 速度阈值或滤波参数不合法')
        return settings
