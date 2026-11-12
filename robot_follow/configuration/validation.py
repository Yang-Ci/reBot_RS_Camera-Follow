"""Validate follow settings before starting hardware."""

import numpy as np


def validate_follow_cfg(cfg):
    """在相机/机械臂初始化前拒绝非有限值和危险控制配置。"""
    def walk(value, path="follow"):
        if isinstance(value, dict):
            for key, item in value.items():
                walk(item, f"{path}.{key}")
        elif isinstance(value, (list, tuple)):
            for index, item in enumerate(value):
                walk(item, f"{path}[{index}]")
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            if not np.isfinite(value):
                raise ValueError(f"配置错误: {path} 必须是有限数值")

    walk(cfg)
    f = cfg["follow"]
    checks = [
        (0.18 <= f["distance"] <= 0.60, "distance 须在 0.18~0.60m"),
        (0.0 < f["gain"] <= 3.5, "gain 须在 (0, 3.5]"),
        (0.0 < f["gain_x"] <= 2.5, "gain_x 须在 (0, 2.5]"),
        (0.0 < f["max_vel"] <= 0.70, "max_vel 须在 (0, 0.70]m/s"),
        (0.0 < f["max_vel_joint"] <= 2.0, "max_vel_joint 须在 (0, 2.0]rad/s"),
        (f["position_deadband"] > 0.0, "position_deadband 必须为正"),
        (f["deadband_hysteresis"] >= 1.0, "deadband_hysteresis 不得小于 1"),
        (f["kf_ff_deadband_on"] > f["kf_ff_deadband_off"] >= 0.0,
         "kf_ff_deadband_on 必须大于 kf_ff_deadband_off"),
        (f["kf_ff_exit_frames"] >= 1, "kf_ff_exit_frames 至少为 1"),
        (f["kf_r"] > 0.0 and f["kf_r_xy"] > 0.0 and f["kf_r_z"] > 0.0,
         "KF 观测噪声方差必须为正"),
        (f["kf_slow_follow_dist"] > f["kf_slow_follow_release"] >= 0.0,
         "慢速跟随确认距离必须大于释放距离"),
        (f["kf_slow_follow_frames"] >= 1 and
         0.0 <= f["kf_slow_follow_dir_cos"] <= 1.0 and
         0.0 < f["kf_slow_follow_speed"] <= 0.06,
         "慢速跟随帧数/方向/速度参数不合法"),
        (0.05 <= f["camera_hw_max_transport_s"] <= 0.20,
         "硬件时间最大传输延迟须在 0.05~0.20s"),
        (f["diag_queue_size"] >= 128, "diag_queue_size 至少为 128"),
        (0.0 <= f["diag_actual_feedback_hz"] <= 20.0,
         "实际关节诊断频率须为 0（关闭）或不超过 20Hz"),
        (f["ff_taper_speed_ref"] > 0.0 and
         f["ff_taper_max_dist"] >= f["ff_taper_dist"] and
         f["ff_taper_max_dist"] <= 0.15,
         "自适应停靠距离/速度参考不合法"),
        (f["kf_axis_adapt_ref_xy"] > 0.0 and f["kf_axis_adapt_ref_z"] > 0.0 and
         1.0 <= f["kf_axis_adapt_max_xy"] <= 2.0 and
         1.0 <= f["kf_axis_adapt_max_z"] <= 1.5 and
         f["kf_axis_adapt_confirm"] >= 1 and
         f["kf_axis_adapt_attack_tau"] > 0.0 and
         f["kf_axis_adapt_release_tau"] > 0.0 and
         0.1 <= f["kf_static_q_scale"] <= 1.0,
         "分轴自适应 Q / 静止 Q 倍率参数不合法"),
        (0.0 < f["static_velocity_hold_speed"] <= 0.08 and
         0.0 <= f["static_velocity_hold_delay_s"] <= 0.10,
         "静止残余速度门槛/确认时长不合法"),
        (f["kf_lead_ms"] >= 0.0 and f["kf_lead_max_ms"] >= 0.0,
         "lead 时间不得为负"),
        (not f["kf_dynamic_lead"] or f["kf_lead_max_ms"] >= f["kf_lead_ms"],
         "动态 lead 上限不得小于基础 lead"),
        (f["kf_lead_full_dist"] > f["kf_lead_near_dist"] >= 0.0,
         "kf_lead_full_dist 必须大于 kf_lead_near_dist"),
        (0.01 <= f["kf_lead_dir_full_cos"] <= 0.99,
         "kf_lead_dir_full_cos 须在 0.01~0.99"),
        (f["kf_lead_attack_tau"] > 0.0 and f["kf_lead_release_tau"] > 0.0,
         "lead 渐变时间常数必须为正"),
        (0.0 < f["target_stale_s"] <= f["lost_hold_s"],
         "target_stale_s 须为正且不得大于 lost_hold_s"),
        (f["servo_direction"] in (-1.0, 1.0), "servo_direction 只能为 ±1"),
        (0.0 <= f["reversal_residual_scale"] <= 1.0,
         "reversal_residual_scale 须在 0~1"),
    ]
    for ok, message in checks:
        if not ok:
            raise ValueError(f"follow.yaml 配置错误: {message}")
    ws = cfg["workspace"]
    lower = np.asarray(ws["min"], dtype=np.float64)
    upper = np.asarray(ws["max"], dtype=np.float64)
    if lower.shape != (3,) or upper.shape != (3,) or np.any(lower >= upper):
        raise ValueError("follow.yaml 配置错误: workspace min/max 必须为递增的三维边界")
    if not 0.10 <= ws["max_radius"] <= 0.42:
        raise ValueError("follow.yaml 配置错误: workspace.max_radius 须在 0.10~0.42m，"
                         "水平径向工作范围不得超过 70% 臂展安全规则")
