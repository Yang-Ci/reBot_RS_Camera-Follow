"""Follow configuration defaults, loading, and runtime conversions.

Each instance owns its values. Loading and applying profiles therefore does not
change parameters in another runtime. The legacy entry point retains its own
instance for scripts that still import the original helper functions.
"""

import os

import numpy as np
import yaml


class FollowConfiguration:
    """Mutable builder for a single validated follow configuration."""

    def __init__(self):
        self.tool_offset = np.array([0.0, 0.0, 0.0])   # TCP 相对末关节的偏移（米）
        # 相机系分方向增益：[X=画面水平(左右), Y=竖直(上下), Z=深度]
        # 水平方向跟随主要靠底座 J1 偏航（惯量最大、限速最低），增益高了会
        # 饱和振荡（先反向、再晃回来）；竖直/深度用 J2/J3 俯仰，可以高些。
        self.lambda_gain_xyz = np.array([1.8, 2.5, 2.5])
        self.k_orientation = 1.6           # 姿态修正增益
        self.desired_depth = 0.30          # 期望相机到掌心距离 (m)，不要小于 0.20
        self.servo_direction = 1.0         # 若运动方向完全相反，改为 -1.0（标定反了的症状）
        self.position_deadband = 0.015     # 3D 误差死区 (m)，想贴更近可降到 0.010
        self.orientation_deadband = np.deg2rad(2.0)
        self.max_vel_cart = 0.30           # 最大末端线速度 (m/s)，首次调试建议 0.10
        self.max_vel_angular = 1.20        # 最大角速度 (rad/s)
        self.max_vel_joint = 1.20          # 最大关节速度 (rad/s)，适配层还有第二道限幅
        self.ctrl_rate = 0.01              # 控制周期 10ms (100Hz)：指令量化延迟减半
                                      # 注意：下面三个常量由 CTRL_RATE 推导（时间制），
                                      # 改频率时无需手改，安全时长不会随频率变化

        # 目标速度前馈：纯误差反馈的跟随必有滞后（滞后 ≈ 目标速度/增益），
        # 手移动越快手臂落后越多。估计掌心目标速度并前馈叠加。0 = 关闭。
        self.vel_feedforward = 0.7         # 前馈系数，推荐 0.5~0.8；出现超前摆动则调低
        self.ff_smooth = 0.3               # 目标速度估计的平滑系数（防检测跳变放大）
        self.ff_max_vel = 0.6              # 前馈速度限幅 (m/s)，防检测丢帧跳变
        self.ff_min_speed = 0.05           # 目标速度低于此值视为静止，前馈置零：
                                      # 否则深度噪声被 /CTRL_RATE 放大后持续注入，
                                      # 是「手不动臂却小幅振荡」的主因之一
        self.ff_kf_deadband_off = 0.06   # KF 速度死区下沿 (m/s)：速度估计低于此值判定手已停，
                                    # 前馈与 lead 外推整体归零——静止时 CV 模型会把 3D 观测
                                    # 噪声解释成 ~0.15m/s 的慢速漂移，直接注入前馈就是
                                    # 「臂伸展、手停了却来回抖」的主因（log33）；
                                    # 伸展态 MIT 杠杆最长、有效阻尼最低，扰动被放大成可见晃动
        self.ff_kf_deadband_on = 0.11    # 回差上沿：速度超过才恢复运动状态，防阈值边界往复抖动
        self.kf_ff_exit_frames = 5       # 跳出静止确认帧数：速度须连续 N 周期越过上沿才
                                    # 恢复运动（log38：死区状态在边界逐帧闪烁，每次跳出
                                    # 都解锁锁存+lead 打一拍目标=停手振荡；50ms 确认
                                    # 窗让噪声单帧毛刺无法解除锁存；进入静止仍即时）
        self.kf_hold_tol = 0.03          # 旧版相机系锁存迁移容差；仅保留配置兼容。
                                    # 静止目标已改为世界系 hold_world；旧“偏离容差
                                    # 就 0.06m/s 拉回”的兜底已删除，避免噪声漂移。
        self.ff_adaptive_taper_enable = True  # 快速接近时提前减前馈；输出 scale 不大于 v8
        self.ff_taper_speed_ref = 0.35        # 朝目标接近速度参考 (m/s)
        self.ff_taper_max_dist = 0.12         # 自适应停靠距离硬上限 (m)
        self.ff_taper_dist = 0.07          # 前馈近端渐弱距离 (m)：误差小于此值时前馈
                                      # 线性衰减到 0——手停下瞬间前馈惯性会把机械臂
                                      # 推过目标（到位后回环振荡的主因），巡航段
                                      # 不受影响，只在最后 ~7cm 停靠段退出

        # 卡尔曼目标滤波（恒速模型）：替代「帧间差分+EMA」的目标速度估计。
        # 差分把 2cm 位置噪声 /0.01s 放大成 1.7m/s 假速度，靠 EMA 硬压又引入
        # 群延迟；卡尔曼把速度作为内部状态联合估计，仿真（σ=2cm 观测噪声）：
        #   速度 RMSE 0.54→0.20 m/s（-63%），静止假速度 0.58→0.15 m/s（-74%）。
        # 第一版 lead=0 只取速度喂前馈，位置通道不接入（死区/增益行为不变）。
        self.kf_enable = True              # false = 退回旧差分路径（独立回退开关）
        self.kf_q = 2.0                    # 过程噪声：手的加速度能力 (m/s^2)^2
                                      # 大=跟踪快噪声多；小=平滑好滞后大。1~5 是合理区
        self.kf_r = 4e-3                   # 标量观测噪声方差（旧配置回退）
        self.kf_r_xy = 2.5e-3              # 横向适度信任观测；保守于直接大幅降低 R
        self.kf_r_z = 4e-3                 # 深度维持 v7.1 噪声强度，不放大 Z 抖动
        # 慢拖参数：在世界系掌心点阵上确认，确认后只限速迁移 hold_world。
        # 相机自运动不会进入这个通道；静止时前馈/lead 仍保持撤销。
        self.kf_slow_follow_enable = True
        self.kf_slow_follow_dist = 0.014
        self.kf_slow_follow_release = 0.008
        self.kf_slow_follow_frames = 3
        self.kf_slow_follow_dir_cos = 0.80
        self.kf_slow_follow_speed = 0.045
        self.camera_hw_timestamp_enable = False  # 未确认实机 SDK 前保持主机时刻基线
        self.camera_hw_max_transport_s = 0.20
        self.diag_enable = True
        self.diag_queue_size = 8192
        self.diag_actual_feedback_hz = 0.0  # 同步 CAN 诊断默认关闭；开启时只允许 0<hz<=2
        self.kf_axis_adapt_enable = True    # 连续同向创新才增强 XY；Z 默认不放大
        self.kf_axis_adapt_ref_xy = 0.060
        self.kf_axis_adapt_ref_z = 0.080
        self.kf_axis_adapt_max_xy = 1.60
        self.kf_axis_adapt_max_z = 1.00
        self.kf_axis_adapt_confirm = 2
        self.kf_axis_adapt_attack_tau = 0.040
        self.kf_axis_adapt_release_tau = 0.120
        self.kf_axis_adapt_static_guard = True  # 静止观测先用基准 Q，运动确认后再增强
        self.kf_static_q_scale = 1.0            # 静止时额外压低 Q 的倍率（1=关闭）
                                           # follow.yaml 默认 0.75：静止速度估计更平滑，
                                           # 运动确认后立即恢复 1.0 + 分轴增强
        self.static_vel_hold_enable = True      # 只清除静止死区内的低速残余指令状态
        self.static_vel_hold_speed = 0.025      # 高于此速度仍走原 EMA + 加速度限制
        self.static_vel_hold_delay_s = 0.03     # 连续确认时长；帧数由 CTRL_RATE 推导
        self.static_vel_hold_frames = max(1, round(self.static_vel_hold_delay_s / self.ctrl_rate))
        self.kf_resync_frames = 3          # 连续跳变剔除帧数上限：盲期后手真的移了位时，
        self.kf_adapt_enable = True        # 自适应过程噪声：创新（观测-预测偏差）大说明
                                      # 手在加速，临时放大 Q 让滤波器更信观测、跟手更快；
                                      # 平稳段回到基准 Q，保持静止/匀速段的平滑度
        self.kf_adapt_ref = 0.05           # 创新参考值 (m)：倍率 = clip(创新/REF, 1, MAX)
        self.kf_adapt_max = 4.0            # Q 最大放大倍率（≈响应提速一档，再大噪声压不住）
        self.kf_adapt_decay = 0.90         # 无新观测周期的倍率衰减（毛刺过后 ~0.1s 回到 1×）
                                      # 每帧都被 6cm 守卫拒收 → 滤波器永久锁死、前馈
                                      # 持续注入过期速度（log26「不跟手」的根因）。
                                      # 连续剔除 ≥3 帧认定是真实位移：重同步——
                                      # 位置采纳新观测、速度清零（不注入假速度）
        self.kf_lead_ms = 0.0              # 卡尔曼外推（第二版）：伺服目标 = 滤波位置 +
                                      # 速度×lead，主动补偿「帧等待+识别+滤波」段延迟。
                                      # 0 = 关闭（= 第一版行为，代码默认）。
                                      # 安全设计：仅手掌可见帧启用（丢失帧仍走 coast）；
                                      # 跳变剔除连续帧期间 lead 清零（换向时恒速模型
                                      # 不可信）；重同步后速度为 0，外推自然归零。
                                      # 推荐 10~15ms 起步；>20ms 换向过冲风险增大
        self.kf_dynamic_lead = True        # 基础 lead + 本帧视觉处理耗时；重复帧不累计 lead
        self.kf_lead_max_ms = 20.0         # 动态 lead 硬上限，换向/剔除/冻结仍单独归零
        self.kf_lead_adaptive = True       # 速度/近目标/方向三重门控；false 仅用于旧逻辑对照
        self.kf_lead_near_dist = 0.022     # 误差小于此值不再额外 lead（与位置死区一致）
        self.kf_lead_full_dist = 0.045     # 实机稳定后提前进入全量 lead，减少中速跟随拖后
        self.kf_lead_dir_full_cos = 0.50   # 速度与剩余误差方向足够一致才恢复全量 lead
        self.kf_lead_attack_tau = 0.025    # 运动确认后更快恢复；静止/反向撤权规则不变
        self.kf_lead_release_tau = 0.015   # 撤回提前量时间常数：更快，优先停稳
        self.kf_fast_start_enable = False  # 实机 A/B 稳定基线；原速度确认通道仍保留
        self.kf_fast_start_frames = 2      # 只统计新三维观测，不按100Hz重复周期计数
        self.kf_fast_start_dist = 0.030    # 累计位移门槛 (m)，须高于静止三维噪声
        self.kf_fast_start_dir_cos = 0.80  # 相邻位移方向一致性，换向时重新确认
        self.deadband_hysteresis = 1.5     # 死区迟滞：进入死区后，误差要超过 1.5×死区
                                      # 才重新启动——防止误差在死区边界反复穿入穿出
        self.depth_oe_min_cutoff = 1.0     # 深度通道 One Euro 参数：静止时压低 z3d 抖动
        self.depth_oe_beta = 0.02          # （手快速靠近/远离时自动放开滤波）
        self.z3d_max_rate = 0.05           # 深度限速 (m/帧)：参照无效时的最后防线
        self.z3d_max_jump = 0.08           # 深度跳变剔除阈值 (m)：单帧 |Δz| 超过此值
                                      # 视为误检帧丢弃（手 16ms 内不可能移动 8cm）
        self.z_freeze_max = 0.20           # 深度冻结最长时长 (s)：2D 新鲜但深度无效时
                                      # （快速挥手的深度空洞，log28 主诉），用最后
                                      # 通过四层保护链验证的深度冻结 z 分量、横向
                                      # 继续跟踪；不采纳任何新深度（保护链不拆）。
                                      # 超时回退原有丢失流程（coast → 保持）。
                                      # 0.20s × 手纵向速度 0.5m/s ≈ 最坏 10cm 深度
                                      # 保持误差，有界且无尖峰，可安全接受

        # 伸展度增益调度：手臂前伸越多，J1 杠杆臂越长、整臂刚度越低，
        # 同样增益在伸展位姿会超出稳定裕度（前伸后停靠大振荡的根源）。
        # TCP 水平半径 ≤ REACH_SOFT_START 时参数原样，到 REACH_SOFT_END
        # 线性衰减到 REACH_MIN_SCALE，超过后保持 REACH_MIN_SCALE。
        self.reach_soft_start = 0.25       # 开始衰减的水平伸展半径 (m)
        self.reach_soft_end = 0.40         # 衰减到底的半径 (m)
        self.reach_min_scale = 0.60        # 最深伸展时停靠段指令的保留比例
        self.reach_dock_dist = 0.20        # 停靠段半径 (m)：误差小于此值才开始衰减，
        self.reach_att_tau = 0.15          # 伸展衰减乘子 EMA 平滑时间常数 (s)（log36 修复）
                                      # 巡航段（误差大）不衰减、全速跟手

        self.camera_fps = 30               # 相机帧率；Gemini2 支持 640x480@60，
                                      # 高速跟随建议 --fps 60（感知延迟砍半）

        # 阻尼和平滑参数
        self.damping_base = 0.010
        self.damping_adaptive = True
        self.manipulability_threshold = 0.004
        self.min_manipulability_speed_scale = 0.35
        self.smooth_alpha = 0.38
        # 方向反转时清除速度平滑的「惯性」：手突然反向快移时，若不清除，
        # 平滑后的指令仍维持原方向 → 手臂先反向冲一下再拉回
        self.reversal_fast_response = True
        self.max_accel = 2.0               # 末端速度指令加速度上限 (m/s^2)，启停梯形化
        self.reversal_accel_boost = 1.8    # 方向反转时的加速度额度倍数
        self.reversal_residual_scale = 0.35  # 反转时保留的旧方向速度比例
                                        # 0=原逻辑（先完全停住，易顿挫）
                                        # 0.35=保留少量惯性并加速反向，减少换向卡顿

        # 工作空间虚拟墙（基座系，米）——按实际桌面环境设置
        self.workspace_min = np.array([-0.40, -0.10, 0.03])
        self.workspace_max = np.array([ 0.40,  0.40, 0.60])
        self.workspace_radius = 0.42       # 70% 臂展：水平径向 hypot(X,Y) 上限；不限制 Z

        self.lost_hold_frames = round(0.30 / self.ctrl_rate)   # 连续丢手 0.30s → 保持不动
        self.coast_frames = round(0.12 / self.ctrl_rate)       # 丢失外推窗口 0.12s
        self.coast_decay = 0.90 ** (self.ctrl_rate / 0.02)     # 每 20ms 等效衰减 0.90（防爆冲）
        self.coast_max_vel = 0.35          # 外推速度限幅 (m/s)：防前馈污染带跑
        self.target_stale_s = 0.12         # 控制侧观测看门狗：视觉线程卡住也不能永远沿用旧目标

    def load_follow_cfg(self, path):
        """读取跟随伺服参数配置，缺失字段给代码内置默认值。

        优先级：命令行参数 > 本 yaml > 代码内置默认值。
        """
        cfg = {"follow": {
                   "distance": self.desired_depth,
                   "gain": float(self.lambda_gain_xyz[1]),
                   "gain_x": float(self.lambda_gain_xyz[0]),
                   "max_vel": self.max_vel_cart,
                   "max_vel_joint": self.max_vel_joint,
                   "max_vel_angular": self.max_vel_angular,
                   "position_deadband": self.position_deadband,
                   "deadband_hysteresis": self.deadband_hysteresis,
                   "orientation_deadband_deg": float(np.rad2deg(self.orientation_deadband)),
                   "vel_feedforward": self.vel_feedforward,
                   "ff_smooth": self.ff_smooth,
                   "ff_max_vel": self.ff_max_vel,
                   "ff_min_speed": self.ff_min_speed,
                   "ff_taper_dist": self.ff_taper_dist,
                   "ff_adaptive_taper_enable": self.ff_adaptive_taper_enable,
                   "ff_taper_speed_ref": self.ff_taper_speed_ref,
                   "ff_taper_max_dist": self.ff_taper_max_dist,
                   "kf_enable": self.kf_enable,
                   "kf_q": self.kf_q,
                   "kf_r": self.kf_r,
                   "kf_r_xy": self.kf_r_xy,
                   "kf_r_z": self.kf_r_z,
                   "kf_slow_follow_enable": self.kf_slow_follow_enable,
                   "kf_slow_follow_dist": self.kf_slow_follow_dist,
                   "kf_slow_follow_release": self.kf_slow_follow_release,
                   "kf_slow_follow_frames": self.kf_slow_follow_frames,
                   "kf_slow_follow_dir_cos": self.kf_slow_follow_dir_cos,
                   "kf_slow_follow_speed": self.kf_slow_follow_speed,
                   "camera_hw_timestamp_enable": self.camera_hw_timestamp_enable,
                   "camera_hw_max_transport_s": self.camera_hw_max_transport_s,
                   "diag_enable": self.diag_enable,
                   "diag_queue_size": self.diag_queue_size,
                   "diag_actual_feedback_hz": self.diag_actual_feedback_hz,
                   "kf_axis_adapt_enable": self.kf_axis_adapt_enable,
                   "kf_axis_adapt_ref_xy": self.kf_axis_adapt_ref_xy,
                   "kf_axis_adapt_ref_z": self.kf_axis_adapt_ref_z,
                   "kf_axis_adapt_max_xy": self.kf_axis_adapt_max_xy,
                   "kf_axis_adapt_max_z": self.kf_axis_adapt_max_z,
                   "kf_axis_adapt_confirm": self.kf_axis_adapt_confirm,
                   "kf_axis_adapt_attack_tau": self.kf_axis_adapt_attack_tau,
                   "kf_axis_adapt_release_tau": self.kf_axis_adapt_release_tau,
                   "kf_axis_adapt_static_guard": self.kf_axis_adapt_static_guard,
                   "kf_static_q_scale": 1.0,
                   "static_velocity_hold_enable": self.static_vel_hold_enable,
                   "static_velocity_hold_speed": self.static_vel_hold_speed,
                   "static_velocity_hold_delay_s": self.static_vel_hold_delay_s,
                   "stationary_joint_hold": False,
                   "kf_lead_ms": self.kf_lead_ms,
                   "kf_dynamic_lead": self.kf_dynamic_lead,
                   "kf_lead_max_ms": self.kf_lead_max_ms,
                   "kf_lead_adaptive": self.kf_lead_adaptive,
                   "kf_lead_near_dist": self.kf_lead_near_dist,
                   "kf_lead_full_dist": self.kf_lead_full_dist,
                   "kf_lead_dir_full_cos": self.kf_lead_dir_full_cos,
                   "kf_lead_attack_tau": self.kf_lead_attack_tau,
                   "kf_lead_release_tau": self.kf_lead_release_tau,
                   "kf_fast_start_enable": self.kf_fast_start_enable,
                   "kf_fast_start_frames": self.kf_fast_start_frames,
                   "kf_fast_start_dist": self.kf_fast_start_dist,
                   "kf_fast_start_dir_cos": self.kf_fast_start_dir_cos,
                   "kf_ff_deadband_off": self.ff_kf_deadband_off,
                   "kf_ff_deadband_on": self.ff_kf_deadband_on,
                   "kf_hold_tol": self.kf_hold_tol,
                   "kf_ff_exit_frames": self.kf_ff_exit_frames,
                   "smooth_alpha": self.smooth_alpha,
                   "reversal_fast_response": self.reversal_fast_response,
                   "reversal_residual_scale": self.reversal_residual_scale,
                   "depth_oe_min_cutoff": self.depth_oe_min_cutoff,
                   "depth_oe_beta": self.depth_oe_beta,
                   "k_orientation": self.k_orientation,
                   "fps": self.camera_fps,
                   # ---- 以下为进阶参数（原硬编码，全部开放 yaml 可调）----
                   "servo_direction": self.servo_direction,
                   "kf_resync_frames": self.kf_resync_frames,
                   "kf_adapt_enable": self.kf_adapt_enable,
                   "kf_adapt_ref": self.kf_adapt_ref,
                   "kf_adapt_max": self.kf_adapt_max,
                   "kf_adapt_decay": self.kf_adapt_decay,
                   "z3d_max_rate": self.z3d_max_rate,
                   "z_freeze_max": self.z_freeze_max,
                   "lost_hold_s": self.lost_hold_frames * self.ctrl_rate,
                   "coast_window_s": self.coast_frames * self.ctrl_rate,
                   "coast_decay_per20ms": 0.90,
                   "coast_max_vel": self.coast_max_vel,
                   "target_stale_s": self.target_stale_s,
                   "reach_att_tau": self.reach_att_tau,
                   "damping_base": self.damping_base,
                   "damping_adaptive": self.damping_adaptive,
                   "manipulability_threshold": self.manipulability_threshold,
                   "min_manip_speed_scale": self.min_manipulability_speed_scale,
               },
               "workspace": {"min": self.workspace_min.tolist(),
                             "max": self.workspace_max.tolist(),
                             "max_radius": self.workspace_radius}}
        if not os.path.exists(path):
            print(f"[配置] 未找到 {path}，使用内置默认跟随参数")
            return cfg
        with open(path, "r", encoding="utf-8") as f:
            user = yaml.safe_load(f) or {}
        for section, defaults in cfg.items():
            if isinstance(user.get(section), dict):
                defaults.update(user[section])
        return cfg


    def apply_follow_cfg(self, cfg):
        """Convert loaded YAML values into settings for this runtime instance."""
        fcfg = cfg["follow"]
        self.position_deadband = float(fcfg["position_deadband"])
        self.deadband_hysteresis = float(fcfg["deadband_hysteresis"])
        self.orientation_deadband = np.deg2rad(float(fcfg["orientation_deadband_deg"]))
        self.vel_feedforward = float(fcfg["vel_feedforward"])
        self.ff_smooth = float(fcfg["ff_smooth"])
        self.ff_max_vel = float(fcfg["ff_max_vel"])
        self.ff_min_speed = float(fcfg["ff_min_speed"])
        self.ff_taper_dist = float(fcfg["ff_taper_dist"])
        self.ff_adaptive_taper_enable = bool(fcfg.get(
            "ff_adaptive_taper_enable", self.ff_adaptive_taper_enable))
        self.ff_taper_speed_ref = float(fcfg.get("ff_taper_speed_ref", self.ff_taper_speed_ref))
        self.ff_taper_max_dist = float(fcfg.get("ff_taper_max_dist", self.ff_taper_max_dist))
        self.kf_enable = bool(fcfg.get("kf_enable", self.kf_enable))
        self.kf_q = float(fcfg.get("kf_q", self.kf_q))
        self.kf_r = float(fcfg.get("kf_r", self.kf_r))
        self.kf_r_xy = float(fcfg.get("kf_r_xy", self.kf_r))
        self.kf_r_z = float(fcfg.get("kf_r_z", self.kf_r))
        self.kf_slow_follow_enable = bool(fcfg.get(
            "kf_slow_follow_enable", self.kf_slow_follow_enable))
        self.kf_slow_follow_dist = float(fcfg.get(
            "kf_slow_follow_dist", self.kf_slow_follow_dist))
        self.kf_slow_follow_release = float(fcfg.get(
            "kf_slow_follow_release", self.kf_slow_follow_release))
        self.kf_slow_follow_frames = int(fcfg.get(
            "kf_slow_follow_frames", self.kf_slow_follow_frames))
        self.kf_slow_follow_dir_cos = float(fcfg.get(
            "kf_slow_follow_dir_cos", self.kf_slow_follow_dir_cos))
        self.kf_slow_follow_speed = float(fcfg.get(
            "kf_slow_follow_speed", self.kf_slow_follow_speed))
        self.camera_hw_timestamp_enable = bool(fcfg.get(
            "camera_hw_timestamp_enable", self.camera_hw_timestamp_enable))
        self.camera_hw_max_transport_s = float(fcfg.get(
            "camera_hw_max_transport_s", self.camera_hw_max_transport_s))
        self.diag_enable = bool(fcfg.get("diag_enable", self.diag_enable))
        self.diag_queue_size = int(fcfg.get("diag_queue_size", self.diag_queue_size))
        self.diag_actual_feedback_hz = float(fcfg.get(
            "diag_actual_feedback_hz", self.diag_actual_feedback_hz))
        self.kf_axis_adapt_enable = bool(fcfg.get(
            "kf_axis_adapt_enable", self.kf_axis_adapt_enable))
        self.kf_axis_adapt_ref_xy = float(fcfg.get(
            "kf_axis_adapt_ref_xy", self.kf_axis_adapt_ref_xy))
        self.kf_axis_adapt_ref_z = float(fcfg.get(
            "kf_axis_adapt_ref_z", self.kf_axis_adapt_ref_z))
        self.kf_axis_adapt_max_xy = float(fcfg.get(
            "kf_axis_adapt_max_xy", self.kf_axis_adapt_max_xy))
        self.kf_axis_adapt_max_z = float(fcfg.get(
            "kf_axis_adapt_max_z", self.kf_axis_adapt_max_z))
        self.kf_axis_adapt_confirm = int(fcfg.get(
            "kf_axis_adapt_confirm", self.kf_axis_adapt_confirm))
        self.kf_axis_adapt_attack_tau = float(fcfg.get(
            "kf_axis_adapt_attack_tau", self.kf_axis_adapt_attack_tau))
        self.kf_axis_adapt_release_tau = float(fcfg.get(
            "kf_axis_adapt_release_tau", self.kf_axis_adapt_release_tau))
        self.kf_axis_adapt_static_guard = bool(fcfg.get(
            "kf_axis_adapt_static_guard", self.kf_axis_adapt_static_guard))
        self.kf_static_q_scale = float(fcfg.get(
            "kf_static_q_scale", self.kf_static_q_scale))
        self.static_vel_hold_enable = bool(fcfg.get(
            "static_velocity_hold_enable", self.static_vel_hold_enable))
        self.static_vel_hold_speed = float(fcfg.get(
            "static_velocity_hold_speed", self.static_vel_hold_speed))
        self.static_vel_hold_delay_s = float(fcfg.get(
            "static_velocity_hold_delay_s", self.static_vel_hold_delay_s))
        self.static_vel_hold_frames = max(1, round(
            self.static_vel_hold_delay_s / self.ctrl_rate))
        self.kf_lead_ms = float(fcfg.get("kf_lead_ms", self.kf_lead_ms))
        self.kf_dynamic_lead = bool(fcfg.get("kf_dynamic_lead", self.kf_dynamic_lead))
        self.kf_lead_max_ms = float(fcfg.get("kf_lead_max_ms", self.kf_lead_max_ms))
        self.kf_lead_adaptive = bool(fcfg.get("kf_lead_adaptive", self.kf_lead_adaptive))
        self.kf_lead_near_dist = float(fcfg.get("kf_lead_near_dist", self.kf_lead_near_dist))
        self.kf_lead_full_dist = float(fcfg.get("kf_lead_full_dist", self.kf_lead_full_dist))
        self.kf_lead_dir_full_cos = float(fcfg.get(
            "kf_lead_dir_full_cos", self.kf_lead_dir_full_cos))
        self.kf_lead_attack_tau = float(fcfg.get("kf_lead_attack_tau", self.kf_lead_attack_tau))
        self.kf_lead_release_tau = float(fcfg.get("kf_lead_release_tau", self.kf_lead_release_tau))
        self.kf_fast_start_enable = bool(fcfg.get("kf_fast_start_enable", self.kf_fast_start_enable))
        self.kf_fast_start_frames = int(fcfg.get("kf_fast_start_frames", self.kf_fast_start_frames))
        self.kf_fast_start_dist = float(fcfg.get("kf_fast_start_dist", self.kf_fast_start_dist))
        self.kf_fast_start_dir_cos = float(fcfg.get("kf_fast_start_dir_cos", self.kf_fast_start_dir_cos))
        self.ff_kf_deadband_off = float(fcfg.get("kf_ff_deadband_off", self.ff_kf_deadband_off))
        self.ff_kf_deadband_on = float(fcfg.get("kf_ff_deadband_on", self.ff_kf_deadband_on))
        self.kf_hold_tol = float(fcfg.get("kf_hold_tol", self.kf_hold_tol))
        self.kf_ff_exit_frames = int(fcfg.get("kf_ff_exit_frames", self.kf_ff_exit_frames))
        self.smooth_alpha = float(fcfg["smooth_alpha"])
        self.reversal_fast_response = bool(fcfg["reversal_fast_response"])
        self.max_accel = float(fcfg.get("max_accel", 2.0))
        self.reversal_accel_boost = float(fcfg.get("reversal_accel_boost", 1.8))
        self.reversal_residual_scale = float(fcfg.get(
            "reversal_residual_scale", self.reversal_residual_scale))
        self.depth_oe_min_cutoff = float(fcfg["depth_oe_min_cutoff"])
        self.depth_oe_beta = float(fcfg["depth_oe_beta"])
        self.z3d_max_jump = float(fcfg.get("z3d_max_jump", self.z3d_max_jump))
        self.k_orientation = float(fcfg["k_orientation"])
        self.max_vel_joint = float(fcfg["max_vel_joint"])
        self.max_vel_angular = float(fcfg["max_vel_angular"])
        # 伸展度增益调度（旧 yaml 没有这四个键时用代码默认值）
        self.reach_soft_start = float(fcfg.get("reach_soft_start", self.reach_soft_start))
        self.reach_soft_end = float(fcfg.get("reach_soft_end", self.reach_soft_end))
        self.reach_min_scale = float(fcfg.get("reach_min_scale", self.reach_min_scale))
        self.reach_dock_dist = float(fcfg.get("reach_dock_dist", self.reach_dock_dist))
        ws = cfg["workspace"]
        self.workspace_min = np.array(ws["min"], dtype=np.float64)
        self.workspace_max = np.array(ws["max"], dtype=np.float64)
        self.workspace_radius = float(ws.get("max_radius", self.workspace_radius))
        # ---- 进阶参数（缺失时用代码内置默认值，帧计数常量由 CTRL_RATE 推导）----
        self.servo_direction = float(fcfg.get("servo_direction", self.servo_direction))
        self.kf_resync_frames = int(fcfg.get("kf_resync_frames", self.kf_resync_frames))
        self.kf_adapt_enable = bool(fcfg.get("kf_adapt_enable", self.kf_adapt_enable))
        self.kf_adapt_ref = float(fcfg.get("kf_adapt_ref", self.kf_adapt_ref))
        self.kf_adapt_max = float(fcfg.get("kf_adapt_max", self.kf_adapt_max))
        self.kf_adapt_decay = float(fcfg.get("kf_adapt_decay", self.kf_adapt_decay))
        self.z3d_max_rate = float(fcfg.get("z3d_max_rate", self.z3d_max_rate))
        self.z_freeze_max = float(fcfg.get("z_freeze_max", self.z_freeze_max))
        self.lost_hold_frames = round(float(fcfg.get(
            "lost_hold_s", self.lost_hold_frames * self.ctrl_rate)) / self.ctrl_rate)
        self.coast_frames = round(float(fcfg.get(
            "coast_window_s", self.coast_frames * self.ctrl_rate)) / self.ctrl_rate)
        self.coast_decay = float(fcfg.get("coast_decay_per20ms", 0.90)) ** (self.ctrl_rate / 0.02)
        self.coast_max_vel = float(fcfg.get("coast_max_vel", self.coast_max_vel))
        self.target_stale_s = float(fcfg.get("target_stale_s", self.target_stale_s))
        self.reach_att_tau = float(fcfg.get("reach_att_tau", self.reach_att_tau))
        self.damping_base = float(fcfg.get("damping_base", self.damping_base))
        self.damping_adaptive = bool(fcfg.get("damping_adaptive", self.damping_adaptive))
        self.manipulability_threshold = float(fcfg.get(
            "manipulability_threshold", self.manipulability_threshold))
        self.min_manipulability_speed_scale = float(fcfg.get(
            "min_manip_speed_scale", self.min_manipulability_speed_scale))
