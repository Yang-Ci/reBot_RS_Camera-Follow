# reBot Arm B601-RS 手掌跟随 + 捏合抓取 一体项目

## 致谢与来源

感谢 [TonyWeyyy / Rebot_B601_RS_CV_Control](https://github.com/TonyWeyyy/Rebot_B601_RS_CV_Control) 的原作者及贡献者，感谢 [Yang-Ci / reBot_RS_Camera-Follow-](https://github.com/Yang-Ci/reBot_RS_Camera-Follow-) 的维护与贡献。本版本在现有 B601-RS 手掌跟随项目基础上继续开发，加入 Gemini 336 支架模型、网页数字孪生与真机控制等功能。

机械臂控制依赖 [LAN-GER / rebot_control](https://github.com/LAN-GER/rebot_control)；相机和支架模型来源与各自许可证见 [模型说明](models/wrist_camera/README.md)。

把原来的两个独立项目合并为一个：

| 原项目 | 功能 | 合并后 |
| --- | --- | --- |
| `rebot_gripper_control` | MediaPipe 捏合距离 → 夹爪开合 | `palm_follow_grasp.py` 的抓取通道 |
| `rebot_palm_follow` | 掌心 3D → 机械臂固定姿态跟随 | `palm_follow_grasp.py` 的跟随通道 |

**一台 Gemini 2 相机、一次 MediaPipe 检测、一条 CAN 总线**，同时完成：
手掌移动 → 机械臂跟随（保持固定距离与姿态）；拇指食指捏合 → 夹爪开合。

> 日常调参只改 `config/follow.yaml`（跟随）和 `config/grasp.yaml`（夹爪）两个文件；
> 命令行参数临时覆盖。完整的技术原理与踩坑记录见 [技术手册](docs/技术手册.md)。

## 技术路线

系统采用腕部深度相机和视觉伺服：一次手部检测同时驱动掌心跟随与捏合抓取，机械臂和夹爪通过同一控制器下发指令。

```text
RGB / 深度相机 → 图像与深度对齐 → MediaPipe 手部关键点
                                      ├→ 掌心深度采样 → 三维目标 → 滤波与预测
                                      │                          ↓
                                      │    坐标变换 → 位置/姿态控制 → 雅可比逆解
                                      │                          ↓
                                      │       工作空间与关节约束 → CAN → 机械臂
                                      └→ 拇指、食指距离 → 滤波与映射 → 夹爪

本地网页 → 管理控制程序、预览相机、显示关节状态和三维模型
```

1. **感知**：Gemini 2、Gemini 336 或 RealSense D435i 提供图像与对齐深度；MediaPipe 提取手部关键点，掌心深度用于恢复三维位置，捏合距离用于控制夹爪。
2. **标定与坐标变换**：使用相机内参、手眼关系和机械臂运动学，将相机观测转换到控制所需的坐标系；不同设备与安装应使用匹配的标定结果。
3. **跟随控制**：视觉线程更新目标，伺服线程按设定的 10 ms 周期执行滤波、速度前馈、位置和姿态控制、阻尼雅可比逆解及关节指令下发。实际频率和延迟以诊断数据为准。
4. **稳定与保护**：深度有效性和跳变检查、目标丢失保持、工作空间约束、奇异位形阻尼以及速度和加速度限制共同约束运动；夹爪使用独立滤波和状态门控。
5. **监控与验证**：本地网页管理控制子进程，通过本机实时通道接收状态并显示相机与机械臂模型；通道不可用时可回退日志。相机重连时会核对可用的设备序列号、图像流和内参，拒绝不匹配的设备。离线工具用于标定、诊断和日志分析。网页模型可能使用低频反馈、指令或估算值，具体来源以页面标记为准。

原项目曾进行实机测试；本次结构重构已完成离线回归，尚需在实际相机与机械臂上重新验证。测试结论依赖具体硬件、安装、标定和配置。更换这些条件后应重新验证。工作空间约束不等同于整机碰撞检测，网页退出也不能替代硬件急停。

## 阅读导航

- 首次运行：下方的相机章节和[快速开始](#快速开始)。
- 日常调参：[配置文件与参数优先级](#配置文件与参数优先级)及跟随、夹爪调参章节。
- 网页操作：[网页说明](web/README.md)；相机安装模型：[模型说明](models/wrist_camera/README.md)。
- 原理和排障：[技术手册](docs/技术手册.md)；历史基准：[定稿技术文档](docs/B601手掌跟随系统_定稿技术文档.md)。历史文档中的参数应以当前配置和 README 为准。
## 只有 D435i：先运行纯视觉模式

已增加 Intel RealSense D435i 的 RGB + 对齐深度支持，**不需要 Gemini 2 或机械臂**。
RealSense 的 IMU 暂未使用。默认以 640×480、30 fps 启动；有 RGB 流时使用自动曝光并将深度对齐到彩色画面。
当设备没有 RGB 流（例如当前 SDK 识别的 D430I），自动使用左红外画面并对齐深度；
红外转为三通道输入检测模型，识别效果取决于实际画面，需现场验证。连接 USB 3 后：

```bash
# 在项目根目录执行
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-d435i.txt
.venv/bin/python palm_follow_grasp.py --camera d435i --no-arm --detector-backend cpu
```

窗口中观察手部关键点、掌心和深度，按 `q` 退出。无 RGB 的红外模式在纯视觉和
机械臂模式中均使用较低的检测阈值、中值滤波和夹爪区域过滤。纯视觉模式默认关闭
红外投影，退出时恢复原设置；机械臂模式保留投影以获得深度。降低阈值可能增加
误检，关闭投影可能导致深度空洞。可加 `--ir-emitter on` 让纯视觉模式保留投影
进行对照；关闭投影仅允许 `--no-arm`。
想试 60 fps 可加 `--fps 60`，若设备或 USB
链路不支持则使用默认的 30 fps。手动彩色曝光可加 `--color-exposure 6`（毫秒）；
默认自动曝光；红外模式不支持此参数。模型文件已包含在 `config/hand_landmarker.task`。如果找不到相机，
先检查 USB 连接及设备权限，并确保没有其他程序占用；安装 librealsense 的 udev
规则需要系统管理员权限。

本项目原有参数、零位、HOME 和手眼标定来自另一台机械臂和 Gemini 2，
**不能直接用于新的相机安装或机械臂**。准备接入机械臂时，先安装其余机械臂
依赖并核对 `rebot_adapter.py` 的路径、CAN、关节零位、HOME、工作空间和夹爪
设置；首次实机低速调试。为避免误用旧手眼结果，D435i 跟随只读取
`config/hand_eye_calibration_d435i.json`，没有此文件将拒绝启动跟随。

D435i 标定需要机械臂及固定 ArUco 标记；以下命令会驱动机械臂，先核对标记尺寸、
安装和工作空间，并准备急停。D435i 的示教点、采样和结果单独保存，勿复用旧样本：

```bash
.venv-arm/bin/python tools/hand_eye_calibration_auto.py --camera=d435i
.venv-arm/bin/python tools/handeye_joint_calib.py --camera=d435i
```

标定完成后应先检查标定质量和方向，才考虑用 `--camera d435i --no-gripper
--gain 1.5 --max-vel 0.10` 进行低速跟随验证。仓库默认的 Gemini 2 路径和参数
仍可用；D435i 参数尚未经过实机调优。相机流模式或安装改变后需要重新标定。

## 从当前姿态自动手眼标定（D435i / D430i）

新程序 `tools/auto_calibrate_d435i.py` 不需要逐关节键盘示教：它以当前机械臂
姿态为起点，自动规划并采集 24 个小幅扰动姿态，然后求解手眼矩阵。它不寻找
标记，也不自动调整初始姿态：启动前必须让固定在末端的相机看见桌面上固定的
ArUco 4X4_50 ID 0 标记（黑色外边长 100 mm；合格采样至少 95 px，尽量更大）。

```bash
# 持续预览标记，显示绿色 ID 0 OK 后按 q 关闭（不连接机械臂）
.venv-arm/bin/python tools/auto_calibrate_d435i.py --preview

# 仅检查标记，不连接机械臂
.venv-arm/bin/python tools/auto_calibrate_d435i.py --check

# 已检查机器人、环境、标记及回零路径后，启动全自动采样
.venv-arm/bin/python tools/auto_calibrate_d435i.py --run
```

`--run` 会使能机械臂，采样关节速度上限为 4°/s，默认关节偏移上限为
[2,2,2,3,3,4] 度；运行过程中 `q` / `Esc` 或 Ctrl+C 请求停止，随后程序调用
控制器回零并失能。准备硬件急停；退出及故障时的回零路径也必须无障碍。
程序检查关节限位和法兰位置的轨迹采样，但 **不检查整条机械臂、夹爪、相机支架
与环境的碰撞**；传感器误差、总线故障、温度保护或驱动器行为也可能改变实际轨迹。
请先核对 URDF、关节零位修正与实物，以及项目原有的工作空间边界；不能以软件检查
代替现场观察。当前默认检查的法兰位置范围为 x=[0.21,0.36]、y=[-0.06,0.24]、
z=[0.24,0.39] 米（基座坐标系），起点不在其中会拒绝自动采样。

采样过程中标记不可移动；尺寸不足或画面不可用的采样会被跳过，轨迹检查失败、
未到位或最终质量不达标时不会写入结果。只有通过重投影、旋转多样性和固定点一致性检查的结果才写入
`config/hand_eye_calibration_d435i.json`，已有文件不会被覆盖。结果记录相机序列号、
图像流和当时的关节零位修正，跟随程序会拒绝使用不匹配的新格式结果。该程序只求手眼
变换，不重新标定机械臂关节零位；需先确认已有零位修正适用于当前机械臂。质量检查
无法证明绝对精度或安全，实机跟随仍要先低速验证。

离线测试：`.venv-arm/bin/python tools/test_auto_calibrate_d435i.py`。

## Gemini 336

Gemini 336 采用 `--camera gemini336`，纯视觉预览：

```bash
.venv-arm/bin/python palm_follow_grasp.py --camera gemini336 --no-arm
```

`models/wrist_camera/gemini336.urdf` 用于本地网页显示，相机本体来自 Orbbec 官方 335/336 模型。V1 支架与旧支架的 CAD 原点及相机安装角度不同；已按底座两孔重合、同侧安装面贴合及相机安装面的 CAD 关系计算名义装配。该相机位姿尚未经过实机验证，跟随前必须检查安装、光学方向、机械臂零位及工作空间，并低速验证。

默认使用 URDF 名义位姿；也可显式指定 `--gemini336-pose calibration` 使用与当前安装匹配的独立实测手眼结果；程序将验证 `config/hand_eye_calibration_gemini336.json` 的设备信息。相机内参仍由相机 SDK 提供。默认叠加 `config/follow_gemini336.yaml`，原有速度、工作空间和夹爪配置不变。

本地监视与控制页面：

```bash
python3 web/server.py
# 打开 http://127.0.0.1:8765
```

网页可以启动本机真机程序、开始或暂停跟随，并在退出时请求回零失能；空格、Q 和 Esc 快捷键可用。页面优先显示低频实际反馈，缺失时标记并显示关节指令。启动、退出的实际机械臂行为及低速选项见 [web/README.md](web/README.md)。

## 接近 J2 软限位的示教 HOME

手动选择的候选 HOME 保存在 `config/home_pose_candidate_20261110_105700_272184.json`，
J2 约 4.5°，低于原软件下限 10°。默认 HOME 没有被覆盖。该姿态接近运动学奇异位置，
尚未进行实机跟随验证。显式传入 `--home-file` 和 `--allow-home-limit-recovery` 才能
使用这个特殊姿态：只允许当前 J2 处于目标以下时向内移动到 HOME，随后跟随控制只能
向原软限位内恢复；达到原限位后继续遵循原限位。HOME 接近速度上限为 4°/s，跟随阶段
关节速度上限为 4°/s，软限位外恢复上限为 2°/s。实际反馈缺失、过期或异常时暂停跟随。
这些软件保护不能代替现场检查；启动及退出可能使机械臂移动，退出会回零并失能。

可选参数 `--home-j2-command-offset 1.2` 给 J2 指令增加 1.2°，用于尝试补偿本次
实测约 1° 的静态偏差；实际到位仍按示教 HOME 检查，J2 容差为 0.5°，J3、J4
容差为 3°。跟随时使用低频实际反馈修正关节位姿估计，并在反馈或跟踪误差异常时
暂停。补偿量是基于本次测试的估计，负载或姿态改变时不一定适用；该模式尚待实机验证。

## 本机机械臂标定环境

本机标定及机械臂模式使用 `.venv-arm`（Python 3.10，复用本机 rebotarm 环境中的
Pinocchio 和 motorbridge），视觉预览仍可使用 `.venv`。机械臂控制库位于
`vendor/rebot_control`，URDF 默认使用 `/home/robot/reBotArm_control_py/urdf/RS/urdf/ReBot_Arm_RS.urdf`；
可分别通过环境变量 `REBOT_CONTROL_PATH` 和 `REBOT_URDF_PATH` 覆盖。

`vendor/` 不随本仓库上传。新机器可单独获取本版本使用的控制库：

```bash
git clone https://github.com/LAN-GER/rebot_control.git vendor/rebot_control
git -C vendor/rebot_control checkout a901ec109ad99d7140c9b2110c618f54a9107ae9
```

网页机械臂模型及部分浏览器资源另依赖 [Yang-Ci / ReBot_Arm_DigitalTwin_RS](https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS)，机械臂 URDF 另依赖 [Seeed-Projects / reBotArm_control_py](https://github.com/Seeed-Projects/reBotArm_control_py)。迁移路径设置见 [网页说明](web/README.md)。

连接前请检查实际机械臂的关节零位、HOME、关节限位、工作空间与夹爪配置；
仓库附带配置是另一台设备的测量值。CAN 需设置 1 Mbps：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

关闭相机预览后，以 `.venv-arm/bin/python tools/hand_eye_calibration_auto.py --camera=d435i`
运行自动标定。程序会使能机械臂；示教保存后检查预检和安全范围，再确认开始自动采样。
相机固定于末端、标记固定在桌面且不可移动。`Q` / `Esc` 可能不能立即打断等待中的运动，
请准备硬件急停；退出会触发机械臂回零。标定质量合格并确认零位、HOME 和检测可靠后才做实机跟随。

## v10.0 控制策略（CV-only）

该策略来自原项目的实机验收版本，只保留恒速 CV 卡尔曼模型，不再包含 Singer。本次重构后的实机行为仍需重新验证。

### 最终控制行为

- **CV-only KF**：位置/速度联合估计，观测年龄外推 + 自适应 lead；
- **eye-in-hand 自运动补偿只用于静止判定**，不再把补偿速度送进前馈/lead；
- **静止后保持当前机械臂位姿**：`ff_moving=false` 后线速度指令归零，不做世界系慢速对中；
- **慢拖目标迁移关闭**：`kf_slow_follow_enable: false`，避免静止目标被噪声推动；
- **换向平滑**：反转时保留 `reversal_residual_scale=0.45` 的旧方向速度，
  避免先完全停住再反向的顿挫；
- **`--no-gripper` 主动打开夹爪**：启动后通过 J7 通道设为 `gripper.open_deg`，
  不再保持当前角度，减少对腕部相机的视野遮挡；
- **单一 CAN 主机**：夹爪只走 `rebot_adapter.set_gripper_deg()`，不另开 motorbridge；
- **退出安全**：`return_to_zero=True, wait=True`，严禁非零位失能。

### 最终参数摘要

| 参数 | 最终值 |
| --- | --- |
| `distance` | `0.35m` |
| `gain / gain_x` | `2.35 / 2.4` |
| `max_vel` | `0.70m/s` |
| `max_vel_joint` | `2.00rad/s`（适配层 `MAX_JOINT_SPEED` 同值） |
| `ff_max_vel` | `0.90m/s` |
| `max_accel` | `3.2m/s²` |
| `kf_lead_ms` | `10.0ms`（动态上限 20ms） |
| `kf_axis_adapt_ref_xy / max_xy / confirm` | `0.025 / 1.85 / 1` |
| `kf_static_q_scale` | `0.65` |
| `position_deadband / deadband_hysteresis` | `0.024m / 1.8` |
| `static_velocity_hold_speed` | `0.060m/s` |
| `smooth_alpha` | `0.48` |
| `reversal_fast_response` | `true` |
| `reversal_accel_boost` | `2.0` |
| `reversal_residual_scale` | `0.45` |
| `kf_slow_follow_enable` | `false` |
| `fps / color_exposure_ms` | `60 / 6.0` |

完整参数以 `config/follow.yaml` 为准。

### 已知边界

- 手掌原始速度 p95 约 `0.85m/s`，最高可达 `1.3m/s`；
- 当前安全包络为末端 `0.70m/s`、关节 `2.0rad/s`，超过部分物理上跟不满；
- 继续提高速度需要重新做电机温度、整臂稳定性和工作空间安全评估，
  最终版不再盲目上提；
- 静止时机械臂保持当前位姿，不会自动把偏离中心的掌心拉回画面中心，
  这是为了保证静止稳定性做的明确取舍。

### 离线验证

```bash
python3 tools/test_servo_upgrades.py    # 57 项通过
python3 tools/offline_smoke_test.py     # 正常/故障、--no-gripper 打开、eye-in-hand 静止
python3 tools/analyze_servo_log.py logs/servo_时间.csv
```

以上不是实机安全认证，首次运行仍须低速、清空工作区、按 README 的安全 checklist 验收。

## 历史版本 v9.0：以实机认可的 v8.0 为稳定基准线

本版没有重写控制架构。v8.0 的慢速跟随、分轴 KF 观测噪声、自适应 lead、静止锁存、
真实 dt、深度四层保护、100Hz MIT 控制、水平 `hypot(X,Y)≤0.42m` 软墙、单 CAN 主机和
“回零到位后才失能”全部保留。新增三项可独立回退的能力：

1. **实际关节反馈与跟踪误差诊断**：复用现有 `RebotAdapter`，独立低频线程读取实际六轴，
   并立即记录同一采样点的内存指令六轴。默认 `diag_actual_feedback_hz: 0.0` 完全关闭；
   普通配置默认关闭，Gemini 336 配置目标 20Hz、上限 20Hz；同步读取超过 10ms 时自动回退至 1Hz。同步 CAN 读取不在 100Hz 热循环执行，也不参与控制；
   异常、未来时间戳或过期样本只记为空，连续 3 次故障永久停用诊断。退出顺序为
   “停控制线程 → 停反馈线程 → 刷新日志 → 保持并安全回零”。
2. **加速阶段分轴自适应 Q**：单轴创新超过参考值且连续 2 次新观测同向，才平滑提高
   XY 过程噪声倍率，默认最高 1.60；Z 默认倍率上限 1.00，不把深度噪声解释为加速度。
   静止、丢手、深度冻结、跳变剔除、重同步、回 HOME 和停止跟随时立即复位。
   旧标量 `kf_adapt_*` 通道保留且默认关闭，可独立对照。
3. **速度感知停靠制动**：按目标朝向期望点的接近速度，在快速接近时把前馈渐弱距离
   从 0.07m 平滑增至最多 0.12m。不超过 0.10m/s 的慢速接近与远离目标均完整保持
   v8.0 行为；新前馈比例永远不大于 v8.0，不增加 lead、增益或速度上限。

离线验证：**45 项单元测试全部通过**；完整假硬件主程序分别运行反馈正常与连续故障
两种场景，覆盖默认配置、v8/v7.1 回退、危险参数拒绝、静止、慢拖、移动、回中心停手、
有效/异常硬件时间、反馈字段、故障留空及安全生命周期。两种场景均有非零跟随运动，
停手末段连续 15 个控制周期关节指令为零，反馈线程先于统一安全停止关闭。
以上不是实机安全认证，首次仍须低速、清空工作区验收。

### 一键回退到实机认可的 v8.0

```bash
python3 palm_follow_grasp.py --follow-cfg config/follow_v8_fallback.yaml
```

该配置只关闭分轴自适应 Q、速度感知停靠和实际反馈采样，旧标量自适应 Q 也保持关闭；
其余 v8.0 控制与安全参数不变。若需回到更早的 v7.1 行为：

```bash
python3 palm_follow_grasp.py --follow-cfg config/follow_v7.1_fallback.yaml
```

不要为排查晃动关闭深度保护、软墙或安全回零。

### 开启实际反馈诊断（默认无需开启）

把 `config/follow.yaml` 中 `diag_actual_feedback_hz` 改为 `1.0`，同时保持
`diag_enable: true`。分析器按 `fb_time` 去重，输出独立反馈样本数、同步读取耗时 p95、
采样点六轴最大跟踪误差 median/p95/max。该低频数据用于观察跟踪误差趋势，
**不能据此精确测量执行延迟**。

### 离线验证与日志分析

```bash
python3 tools/test_servo_upgrades.py
python3 tools/offline_smoke_test.py
python3 tools/analyze_servo_log.py logs/servo_时间.csv
python3 tools/analyze_servo_log.py logs/servo_时间.csv --json
```

分析器将每次停手后的前 0.30s 停靠过程排除在稳态静止指标之外，
输出运动误差、稳态静止关节指令、停手稳定时间、观测/处理/控制耗时、超时比例及
实际反馈跟踪误差，便于同动作 A/B。

## 合并时的关键设计

1. **夹爪改走机械臂控制器的第 7 通道**（最重要）
   旧 gripper 项目自己开一个 motorbridge 连接直连电机 7；若与跟随程序同时跑，
   两个 CAN 主机会在 can0 上争抢电机 7。合并后夹爪目标角由
   `rebot_adapter.set_gripper_deg()` 设置，随 rebot_control 的 100Hz 关节指令
   一同下发，单一 CAN 主机，从根上避免总线冲突。
   - 夹爪角度单位统一为**度**（rebot_control 约定），不再做弧度换算。
   - 夹爪最大速度由适配层统一设为 500°/s，角度软限幅仍为 -100° ~ +400°。

2. **一次检测，两处复用**
   掌心（9 号关键点）用于跟随，拇指尖(4)—食指尖(8) 距离用于夹爪，
   取自同一个 HandLandmarker 结果，不增加检测开销。

3. **安全约束全部保留并叠加**
   跟随侧：虚拟软墙/硬墙、可操作度奇异保护、丢手保持、Esc 请求退出、
   退出时「缓慢回零→到位→失能」（防止自由下落）；
   夹爪侧：启动时以电机当前角为平滑起点（不猛动）、映射两端截断、
   One Euro 滤波防抖 + 死区、适配层角度硬限幅（-100° ~ +400°）兜底。

## 项目结构

```
rebot_palm_follow_grasp/
├── palm_follow_grasp.py           ★ 启动入口与设备装配
├── palm_follow_fixed_orientation.py   仅跟随版（原跟踪主程序，保留作对照调试）
│
├── gemini2_camera.py              Orbbec 相机接口
├── realsense_camera.py            RealSense 相机接口
├── robot_follow/                  新的分层代码包（逐步迁移中）
│   ├── configuration/           跟随配置、夹爪配置和参数校验
│   ├── control/                 伺服循环、目标滤波、时序、夹爪与保护
│   ├── hardware/                相机会话、设备接口和硬件运动学
│   ├── perception/              手部检测与帧处理、深度保护和相机校验
│   ├── presentation/           本地画面和控制诊断显示
│   └── runtime/                 视觉循环、HOME、诊断、资源生命周期和实时状态
├── tests/                         独立模块回归测试
├── kalman_target.py               目标滤波旧导入路径（兼容入口）
├── urdf_camera_pose.py            URDF 相机位姿解析
├── palm_detection.py              手部检测旧导入路径（兼容入口）
├── rebot_adapter.py               机械臂适配层（速度伺服 + FK/雅可比 + J7 夹爪通道）
├── servo_timing.py                时序与控制辅助旧导入路径（兼容入口）
├── servo_diagnostics.py           诊断旧导入路径（兼容入口）
│
├── config/                        所有配置与标定产物
│   ├── follow.yaml                ★ 最终版 CV 跟随参数（日常调参改这里）
│   ├── follow_v9_fallback.yaml     v9 参数回退对照
│   ├── follow_v8_fallback.yaml     一键恢复实机认可的 v8.0 稳定行为
│   ├── follow_v7.1_fallback.yaml   一键恢复更早的 v7.1 稳定行为
│   ├── grasp.yaml                 捏合→夹爪映射与开合标定值
│   ├── hand_eye_calibration.json  手眼标定结果（本项目实测，跟随必需）
│   ├── joint_offset.json          关节零位修正（本项目实测，适配层自动加载）
│   ├── home_pose.json             示教 HOME 位姿（本项目实测，启动优先加载）
│   ├── viewpoint.json             联合标定示教点（重新标定时免重复示教）
│   ├── handeye_joint_samples.npz  联合标定原始样本（重求解免重新采样）
│   └── hand_landmarker.task       MediaPipe 手掌模型
│
├── tools/                         标定与诊断工具（按需运行，日常不用碰）
│   ├── gripper_j7_calibrate.py    夹爪开合标定（键盘点动 J7，结果写回 grasp.yaml）
│   ├── teach_home.py              HOME 位姿示教
│   ├── hand_eye_calibration.py    手眼标定（手动棋盘格 JOG 采样）
│   ├── hand_eye_calibration_auto.py   手眼标定（ArUco + 关节空间自动扰动）
│   ├── handeye_joint_calib.py     手眼+关节零位联合标定（精度最高，推荐）
│   ├── hand_eye_solver.py         手眼标定求解器（OpenCV 5.0 移除了 calibrateHandEye）
│   ├── calib_diagnose.py          标定体检（不标定，只定位问题）
│   ├── camera_intrinsic_check.py  相机内参核对
│   ├── test_servo_upgrades.py     升级逻辑离线单元测试
│   ├── analyze_servo_log.py       高频日志离线 A/B 指标分析
│   └── offline_smoke_test.py      假相机/假机械臂完整主程序冒烟测试
│
├── requirements.txt
├── web/                           本地网页、控制进程与遥测解析
├── models/                        相机与支架模型
├── docs/                          技术手册与历史定稿文档
├── requirements-d435i.txt         D435i 纯视觉依赖
└── README.md
```

核心离线回归：先安装 `python -m pip install -r requirements-test.txt`，再运行 `python tools/run_regression.py`。它在独立进程中依次运行模块、伺服和假硬件流程，避免并发测试互相使用诊断日志；`--list` 可查看项目；`--extended` 额外运行假设备自动标定和浏览器手势逻辑测试，需要 SciPy 和 Node.js。网页进程和信号相关的三项检查只在 Linux 运行，其他平台会标为跳过。ROS 等专项工具仍需按各自环境单独运行。仓库的 Linux GitHub Actions 使用同一扩展回归入口；其结果以实际运行记录为准。以上检查不连接真实机械臂。

## 快速开始

### 1. 环境

```bash
cd rebot_palm_follow_grasp
pip install -r requirements.txt
git clone https://github.com/LAN-GER/rebot_control.git        # 运动下发（源码仓库）
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git  # URDF
```

确认 `rebot_adapter.py` 顶部的 `REBOT_CONTROL_PATH` 与 `URDF_PATH` 指向本机实际路径。

### 2. CAN 总线（每次插拔 USB-CAN 后都要执行）

```bash
#1. 套件里是 PCAN-USB，通常应该直接出现 can0 或 can1
sudo modprobe peak_usb
ip -br link

#2. 如果出现 can0，再设置 bitrate
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

#3. 把 ROS 路径从环境变量里剔除
export PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v /opt/ros | paste -sd:)
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v /opt/ros | paste -sd:)
#4. 验证加载的是 conda 环境里的 pinocchio
python3 -c "import pinocchio as pin; print(pin.__version__, pin.__file__)"
```

### 3. 分步验证（强烈建议按此顺序）

```bash
# 第 0 步：纯视觉演练——不碰硬件，验证掌心检测 + 捏合映射
python3 palm_follow_grasp.py --no-arm

# 第 1 步：仅夹爪——手臂保持不动，捏合验证夹爪开合方向与范围
python3 palm_follow_grasp.py --no-follow

# 第 2 步：仅跟随——夹爪保持打开，低速假目标测试（先别上真人手）
python3 palm_follow_grasp.py --no-gripper --gain 1.5 --max-vel 0.10

# 第 3 步：完整模式
python3 palm_follow_grasp.py

# 如若机械臂故障，先关闭电源重启，等电源响声停止后，清除电机故障指令
motorbridge run --vendor robstride --channel can0 --motor-id 1 --feedback-id 0xFD --mode clear-error
```

常用启动参数（两个主程序都支持）：

```bash
python3 palm_follow_grasp.py --distance 0.35   # 手掌与相机的跟随距离（米），
                                              # 建议 0.20~0.50，默认 0.30
python3 palm_follow_grasp.py --fps 60          # 相机 60fps，感知延迟砍半
python3 palm_follow_grasp.py --gain-x 1.4      # 水平跟随增益（左右振荡时调低）
```

操作：`g`/空格 启停跟随，`q` 退出，`Esc` 急停回零。

**最终版首次实机验收**：先用上面的 `--no-gripper --gain 1.5 --max-vel 0.10`。
清空工作区并准备随时按 q；依次做静止 30 秒、慢速同向移动、匀速移动、
急停、换向、短时丢手和贴墙返回。确认低速全部正常后再用默认参数。
静止时 DBG 的 `lead` 应回到 0；若仍晃，直接采用上面的稳定回退组合，
不要继续提高增益或速度。任何改参一次只改一项并重新做静止与急停测试。

### 4. 标定（环境变动后才需要重做）

```bash
python3 tools/gripper_j7_calibrate.py     # 夹爪开合标定 → config/grasp.yaml
python3 tools/teach_home.py               # 示教顺手的工作 HOME → config/home_pose.json
python3 tools/handeye_joint_calib.py      # 手眼+零位联合标定（需先有 config/viewpoint.json，
                                         #   由 tools/hand_eye_calibration_auto.py 示教生成）
```

随附的 `config/hand_eye_calibration.json` / `config/joint_offset.json` /
`config/home_pose.json` 是本机实测结果；**换了相机安装位置或重设过电机零点后必须重标定**。

## 配置文件与参数优先级

项目有两份配置文件，**日常调参只改这两个文件，不用动代码**：

| 文件 | 管什么 |
| --- | --- |
| `config/follow.yaml` | 跟随伺服全部参数：距离、增益、速度上限、死区/迟滞、前馈、滤波、虚拟墙 |
| `config/grasp.yaml` | 捏合→夹爪映射：像素阈值、滤波、开合标定角度、冻结开关 |

**参数优先级：命令行参数 > yaml 配置 > 代码内置默认值。**

```bash
python3 palm_follow_grasp.py                    # 全部用 follow.yaml 的值
python3 palm_follow_grasp.py --distance 0.35    # 仅距离被覆盖，其余仍走 yaml
python3 palm_follow_grasp.py --follow-cfg my.yaml   # 换一份配置文件
```

## 调参：跟随（config/follow.yaml）

每个参数在 yaml 里都有行内注释，这里给"按症状调"的速查（新架构）：

| 症状 | 改哪个 | 方向 |
| --- | --- | --- |
| 停手后振荡（含伸展位） | 先确认 `kf_lead_adaptive: true`；仍晃用稳定回退 `kf_lead_ms: 0` | 再查 DBG 的 `w/kv/e/latch`，不要先提增益 |
| 停手后上下小幅振荡 | 最终版静止后保持当前位姿；先查 `kf_slow_follow_enable: false`，再调 `position_deadband` | 0.022→0.024→0.025 |
| 死区状态边界闪烁（停手瞬间一抽一抽） | `kf_ff_exit_frames` | 5→7（起步略钝可降回） |
| 跟随偏肉 | `gain` → `smooth_alpha` → `kf_q` | 2.2→2.35 ／ 0.45→0.48 ／ 1.5→1.8 |
| 跟随发冲、关节速度频繁冲 2~3 | `kf_q` → `smooth_alpha` | 2.0→1.5 ／ 0.52→0.45 |
| 静止时仍有微抖 | 先确认已静止保持位姿；再调 `depth_oe_min_cutoff` / `position_deadband` | 0.75→0.6→0.5 ／ 0.024→0.025 |
| 丢手偏多 | 先补光、确认 `color_exposure_ms: 6.0`；手别挥出画面 | 硬件手段优先于参数 |
| 老架构（`kf_enable: false`）回摆/振荡 | `ff_taper_dist` → `vel_feedforward` → `smooth_alpha` | 0.07→0.10 ／ 0.7→0.5 ／ 0.38→0.55 |

注意：`max_vel`（末端线速度）和适配层 `MAX_JOINT_SPEED`（关节限速）是两道串联的墙，
只提末端限速而关节限速不动，左右跟随不会有任何变化——先提 J1。

## 调参：夹爪（config/grasp.yaml）

启动前会校验配置：数值必须有限，`open_px` 须大于 `closed_px`，夹爪开合角差至少 5°；平滑系数须在 0～1，死区、运动阈值和滤波 beta 不得为负，滤波截止频率必须为正。校验失败时不会打开相机或使能机械臂。

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `pinch.closed_px` / `open_px` | 捏合/张开时两指尖像素距离 | 手离相机远则整体调小，按实测改 |
| `pinch.filter_type` | 夹爪目标滤波：`one_euro`（默认，推荐）/ `ema` | one_euro = 静止强滤波、快动自动放开，两全其美 |
| `pinch.min_cutoff_hz` / `beta` | One Euro 参数 | 跟随慢/肉：beta 0.010→0.015~0.020；静止仍抖：min_cutoff 2.0→1.5 |
| `pinch.smoothing` | 仅 `filter_type=ema` 时的平滑系数 | 0.35 是速度与平滑的甜点；更大更跟手但更抖 |
| `pinch.deadband_deg` | 夹爪目标死区 | 指尖是噪声最大的关键点，建议 ≥2；夹爪仍抖就加到 3 |
| `gripper.hold_while_moving` | 手臂运动时冻结夹爪 | **默认 true，跟随稳定性关键**，见下节 |
| `gripper.move_speed_threshold` | 「手臂在运动」判定阈值 (rad/s) | 一般不动；冻结解除太迟钝可调小到 0.03 |
| `gripper.closed_deg` / `open_deg` | 夹爪闭合/张开角度 | **用标定工具实测，勿手猜** |

### 关于跟随稳定性（为什么默认冻结夹爪）

跟随链路与旧单独项目逐字节一致，但合并后多了一个**物理干扰源**：
相机装在手腕上（eye-in-hand），而夹爪电机 J7 就在相机旁边。
跟随过程中手指的微动会让夹爪持续小动作，振动直接抖动画面的掌心
3D 估计——这是合并后跟随「不如以前稳」的最可能原因。

默认开启 `hold_while_moving`：手臂运动时夹爪冻结（HUD 显示
`[FROZEN]`），手停下后夹爪才跟随捏合——符合「移动到位 → 再抓取」的
自然节奏。若你确认需要「边跟随边捏合」，把它设为 `false` 即可。

自查方法：跑 `python3 palm_follow_grasp.py --no-gripper`，
此时夹爪主动打开并保持，不做捏合控制；既减少 J7 对相机的视野遮挡，
也能排除夹爪动作通过相机支架耦合回来的干扰。若稳定性恢复，
即可确认是夹爪干扰而非跟随链路问题。

## 提升跟随速度与精度

左右跟随的物理上限 ≈ J1 偏航限速 × 臂展，高速滞后来自感知+控制延迟。
按见效程度排序的调优手段：

| 手段 | 怎么用 | 预期效果 |
| --- | --- | --- |
| 相机 60fps | `python3 palm_follow_grasp.py --fps 60` 或 yaml `follow.fps: 60` | 感知延迟砍半，高速跟随立竿见影（需 CPU 跑得动 MediaPipe@60fps） |
| 目标速度前馈 | 已内置（`follow.vel_feedforward: 0.7`） | 消除「手快臂慢」的高速滞后；若出现超前摆动调到 0.5 |
| 提 J1 限速 | `rebot_adapter.py` 的 `MAX_JOINT_SPEED` 当前 `2.0` | 最终版不再上提；再快需单独评估电机温度和整臂稳定性 |
| 提水平增益 | `--gain-x 2.2`（配前馈后裕量更大） | 响应更快，出现振荡就退回 |
| 提末端限速 | `--max-vel 0.70` 或 yaml `follow.max_vel` | 与 `MAX_JOINT_SPEED=2.0` 串联；最终版已到安全包络 |
| 提高定位精度 | `follow.position_deadband` 0.024 → 0.020 | 静止时贴得更近；太小会围绕目标微抖 |
| 检测滤波 | `palm_detection.py` 掌心检测已改用 One Euro Filter（`PALM_FILTER="one_euro"`，min_cutoff=1.0, beta=0.02） | 静止更稳、快速移动不拖尾；一般不用动 |

### 旧 gripper 项目调速建议在本项目的对应情况

| 旧建议 | 本项目状态 |
| --- | --- |
| smoothing 0.15→0.35 | 已应用（`pinch.smoothing: 0.35`，ema 模式生效） |
| One Euro Filter | **已应用**：掌心检测（palm_detection.py）+ 捏合目标（默认 `filter_type: one_euro`）两处都换上了 |
| 相机 60fps / MJPG | 已支持：`--fps 60`；Gemini 2 用 pyorbbecsdk 取流，帧率由 SDK 控制，无 CAP_PROP_BUFFERSIZE 概念 |
| `CAP_PROP_BUFFERSIZE=1` | 不适用（非 cv2.VideoCapture），SDK 内部已是最新帧语义 |
| MediaPipe `model_complexity=0` | 不适用：新版 HandLandmarker tasks API 无此参数，模型已是最快的手部模型 |
| MIT 模式 kp/kd + 速度前馈 | kp/kd 在 rebot_control 驱动侧配置；视觉侧已内置目标速度前馈（`VEL_FEEDFORWARD=0.7`） |
| CAN 1Mbps / 48V 供电 | **仍然有效**，属硬件前提，见下方安全 checklist |

注意：`--max-vel` / `follow.max_vel`（末端线速度）和 `MAX_JOINT_SPEED`（关节限速）是两道串联的墙，
只提末端限速而关节限速不动，左右跟随不会有任何变化——先提 J1。

跟随侧全部参数都在 `config/follow.yaml`（增益/死区/前馈/滤波/虚拟墙），
命令行可用 `--distance / --gain / --gain-x / --max-vel / --fps` 临时覆盖，
或 `--follow-cfg` 换整份配置。

## 故障排查

| 症状 | 处理 |
| --- | --- |
| **到位后振荡几下才停稳**（手已停，臂冲过目标再拉回反复几次） | 回环振荡：手停瞬间前馈估计带惯性，把臂推过目标。已内置**前馈近端渐弱**（`follow.ff_taper_dist: 0.07`，误差 <7cm 时前馈线性退出）。调参顺序：① `ff_taper_dist` 加大到 0.10（更早开始退出前馈）② `vel_feedforward` 0.7→0.5（巡航提速变弱但更稳）③ `smooth_alpha` 0.38→0.55（减小速度平滑滞后，响应更"脆"，过冲更小） |
| **手掌不动时机械臂持续小幅振荡** | 最终版在 `ff_moving=false` 后直接保持当前位姿，不做慢速对中；确认 `kf_slow_follow_enable: false`、`kf_enable: true`。其余滤波/死区：`depth_oe_min_cutoff`、`deadband_hysteresis`、`position_deadband: 0.024`。若仍抖，先查 J7/相机支架振动（`--no-gripper` 夹爪已打开），再微调死区。 |
| 左右快速移动时机械臂振荡、先反向后拉回 | 水平跟随靠底座 J1 偏航（惯量大、限速低），增益过高会饱和振荡。调低水平增益 `--gain-x 1.2`，或减小 `--max-vel 0.20`。程序已内置分方向增益 + tanh 软饱和 + 反转清惯性三重缓解 |
| 夹爪不动 | `--no-follow` 单独验证；确认 grasp.yaml 中 `gripper.enabled: true`；按 `tools/gripper_j7_calibrate.py` 重标定 |
| 夹爪开合方向反了 | 把 `closed_deg` / `open_deg` 对调，或检查是否重设过零点 |
| 夹爪响应慢/肉 | 适配层 `GRIPPER_SPEED_DEG_S` 适当加大；确认 rebot_control 的 J7 MIT 增益 |
| 机械臂朝反方向跑 | 停止跟随，检查相机安装朝向、URDF 光学坐标系及关节零位；确认后低速复验 |
| 跟随一直偏一个固定量 | 重做联合标定的不动点验证（分散度 < 2cm 合格） |
| 深度经常为 0 | 手离相机 < 15cm（Gemini2 近距盲区），保持 d ≥ 0.25m |
| 电机不响应 | CAN 没起来/波特率不对：重跑 `ip link` 命令，确认 1 Mbps |

## 安全 checklist（每次运行前）

- [ ] 底座固定、相机支架拧紧、线缆有余量
- [ ] CAN 已配置（1 Mbps）、电机温度正常
- [ ] 三轴虚拟墙已按实际桌面修改；`workspace.max_radius` 不超过 0.42m
- [ ] 首次/改参后：`--no-arm` → `--no-follow` → `--no-gripper` 低速 → 完整模式
- [ ] Esc 请求退出及回零流程有效；硬件急停可用
- [ ] 工作半径内无人；**眼睛看机械臂，不看屏幕**
- [ ] ⚠️ 任何退出路径都必须走完「保持→缓慢回零→到位→失能」，
      严禁在非零位直接失能电机（屈肘姿态失能 = 自由下落，有真实事故教训）

## 致谢与许可

- 跟随控制数学移植自 [HighTorque-Robotics/Panthera_HT_SDK_Extensions](https://github.com/HighTorque-Robotics/Panthera_HT_SDK_Extensions)
- 夹爪手势映射思路移植自 [reBotArm_develop_hjx](https://github.com/HJX-exoskeleton/reBotArm_develop_hjx)（MIT License）
- 电机控制依赖 [motorbridge](https://github.com/tianrking/MotorBridge) 与 [rebot_control](https://github.com/LAN-GER/rebot_control)
- 硬件参数参考 [Seeed reBot Arm B601-RS 官方 Wiki](https://wiki.seeedstudio.com/rebot_arm_b601_rs_mit_control/)

MIT License.

## 新/老伺服架构切换

`config/follow.yaml` 中 `kf_enable` 一个键即可切换，重启脚本生效：

- `kf_enable: true` —— 新架构（卡尔曼恒速滤波）：速度估计更干净，可叠加
  lead 外推（`kf_lead_ms`）与 eye-in-hand 自运动补偿，跟随更跟手；
- `kf_enable: false` —— 老架构（帧间差分+EMA）：无 lead、无自运动补偿，
  运动风格更温润，`ff_smooth` / `ff_min_speed` / `ff_taper_dist` 生效。

两套架构共用全部安全机制（软墙 / 伸展衰减 / 加速度限制 / 丢失外推 /
深度保护链），切换不影响安全性。全部可调参数均在 follow.yaml 内；
yaml 末尾【老架构配套参数组】注释里保留了一套实测稳定的老架构参数。

## 历史版本 v9 技术要点（已被 v10 最终版取代）

按实机日志逐轮迭代出的新架构完整技术栈：

1. **卡尔曼恒速滤波 + 自适应 lead 外推**（`kf_q/kf_r/kf_lead_*`）
   目标是滤波位置 + 速度×观测年龄 + 速度×额外lead×权重。速度、距离和方向
   三重门控让巡航保留延迟补偿，停手、近目标和反向纠偏时收回额外提前量。

2. **静止判定三件套**（修停手振荡）
   - 速度死区带回差：`kf_ff_deadband_off/on`（0.06/0.13 m/s），
     残余假速度低于下沿才进静止，超过上沿才回运动；
   - 跳出确认帧：`kf_ff_exit_frames=7`，须连续 7 周期越过上沿才
     恢复运动，杜绝死区边界状态闪烁引起的停手一抽一抽；
   - 静止状态标记：`hold_latch` 只表示“已判定静止”。v10 最终版已改为
     `ff_moving=false` 后直接保持当前机械臂位姿，不再用实时目标或世界目标
     做慢速对中；`kf_slow_follow_enable` 最终为 `false`。

3. **伸展衰减 EMA 平滑**（`reach_att_tau`）
   伸展衰减增益不再逐帧跳变（曾是伸展位极限环振荡的根因），
   按 τ≈0.15s 平滑过渡。

4. **深度冻结帧跳过 KF 更新**
   深度无效、z 用占位值的帧只做 predict 不做 update，
   防止占位深度把滤波器状态拉飞。

5. **重获目标强化**
   连续丢手 ≥30 帧后重获目标须 5 帧一致（原 3 帧），且深度超过
   `distance + 0.25m` 的一律判为背景拒绝（修背景深度被当手、
   手臂冲向墙壁的问题）。

6. **检测+光流混合跟踪**（`hybrid_track_*`）
   MediaPipe 单帧漏检时，以上一帧掌心为种子做 LK 光流桥接
   （3×3 点阵，≤15 帧，单帧位移 ≤90px 防飞点），桥接帧自动冻结
   夹爪（landmarks 缺失），短丢手不再触发保持-重启的顿挫。

7. **相机帧率结论**
   `tools/enumerate_camera_modes.py` 枚举确认 Gemini 2 彩色/深度
   均 60fps 硬顶（彩色 1280x720@60、深度 640x400@60），
   提帧率这条路已封死，延迟优化只能走 lead 外推。

8. **自适应过程噪声 Q**（`kf_adapt_*`，**本机默认关闭**）
   按创新（观测-预测偏差）动态放大 Q：加速段更信观测、平稳段回基准。
   实测在暗曝光高噪声环境下测量噪声会误触发，反而变冲——
   低噪声环境（补光充足/自动曝光）可开 `kf_adapt_enable: true` 试，
   参考 `kf_adapt_ref 0.10 / max 2.5 / decay 0.85`。

9. **曝光进 yaml**（`color_exposure_ms`，命令行 `--color-exposure` 可覆盖）
   手动曝光压运动模糊：6ms 配合补光是实测最佳点；
   4ms 更暗、MediaPipe 丢失明显增多，不建议。

**v10 最终参数（定版）**：控制基准来自实机多轮日志迭代（60fps BS 深度 + 6ms 手动曝光 +
补光），并已完成低速/高速/静止/换向验收；以下为最终交付参数：

```
distance 0.35 | gain 2.35 / gain_x 2.4 | max_vel 0.70 | max_vel_joint 2.00
kf_enable true | kf_q 1.5 | kf_r_xy 0.0025 | kf_r_z 0.004 | kf_lead_ms 10.0
kf_lead_adaptive true | lead near/full 0.022/0.045 | attack/release 0.025/0.015
kf_fast_start_enable true | workspace.max_radius 0.42（仅水平X/Y）
死区 off/on 0.06/0.13 | exit_frames 7
position_deadband 0.024 | deadband_hysteresis 1.8 | smooth_alpha 0.48 | color_exposure_ms 6.0
kf_axis_adapt_enable true | XY ref/max 0.025/1.85 | Z max 1.00 | confirm 1
kf_axis_adapt_static_guard true | kf_static_q_scale 0.65
kf_slow_follow_enable false | static_velocity_hold_enable true
static_hold speed/delay 0.060m/s / 0.03s
ff_adaptive_taper_enable true | base/max taper 0.07/0.12 | speed_ref 0.35
ff_max_vel 0.90 | max_accel 3.2 | reversal residual 0.45
kf_adapt_enable false | diag_actual_feedback_hz 0.0 | hybrid_track_enable true
MAX_JOINT_SPEED 2.0（rebot_adapter.py） | 夹爪 J7 走单一 CAN 通道
```
