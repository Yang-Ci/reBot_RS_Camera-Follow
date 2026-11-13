# reBot RS Camera Follow

基于腕部深度相机的 B601-RS 机械臂手掌跟随与夹爪控制项目。相机识别手掌位置，程序结合深度和机械臂运动学计算跟随指令；本地网页提供相机预览、关节状态、速度设置和真机控制。

## 致谢

本项目在手掌跟随、视觉伺服和夹爪控制方面借鉴了 [TonyWeyyy / Rebot_B601_RS_CV_Control](https://github.com/TonyWeyyy/Rebot_B601_RS_CV_Control) 的思路，感谢作者及贡献者。Gemini 2 V1 平视版支架由本项目作者制作。

机械臂控制使用 [LAN-GER / rebot_control](https://github.com/LAN-GER/rebot_control)；网页模型使用 [Yang-Ci / ReBot_Arm_DigitalTwin_RS](https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS) 的数字孪生资源。相机支架来源及许可见[支架说明](models/wrist_camera/gemini2-v1-level/README.txt)和[模型说明](models/wrist_camera/README.md)。

## 功能

- 支持 Gemini 2、Gemini 336 和 RealSense D435i 的手掌视觉跟随；相机和机械臂的适配要求见下方说明。
- 通过手部关键点和深度图获取三维掌心位置，经过滤波、坐标变换和运动学计算生成机械臂指令。
- 支持拇指、食指捏合控制夹爪，也可以选择保持夹爪当前位置。
- 本地网页可预览相机与深度图、显示关节状态和三维模型，并调整跟随增益及速度。
- HOME 到位后可按 `R`、点击网页按钮，或持续张开手掌 3 秒开始跟随。

## Gemini 2 V1 平视支架

[支架文件夹](models/wrist_camera/gemini2-v1-level/)包含四个文件：

| 文件 | 用途 |
| --- | --- |
| [Gemini2_V1_Level.step](models/wrist_camera/gemini2-v1-level/Gemini2_V1_Level.step) | 可编辑 CAD 模型，单位 mm |
| [Gemini2_V1_Level.stl](models/wrist_camera/gemini2-v1-level/Gemini2_V1_Level.stl) | 可切片的打印模型，单位 mm |
| [Gemini2_V1_Level_preview.png](models/wrist_camera/gemini2-v1-level/Gemini2_V1_Level_preview.png) | 修改前后及夹爪视场示意 |
| [README.txt](models/wrist_camera/gemini2-v1-level/README.txt) | 尺寸、几何校核和来源说明 |

本项目作者基于 Seeed 的 V1 模型制作了这个平视版支架，将相机固定面从约 15° 下倾调整为名义 0°，保留机械臂侧和相机侧安装接口。视场结果来自 CAD 几何校核，**尚未完成实物试装和相机画面验证**；更换支架后应核对相机安装、手眼标定及实际视野。该模型针对 Gemini 2 的尺寸与视场设计，不能直接视为已验证适配 Gemini 336。

平视支架是独立提供的打印模型；当前网页及控制使用的 Gemini 336 URDF 未因添加该文件而自动更换，详见[腕部模型说明](models/wrist_camera/README.md)。

## 环境与安装

项目面向 Linux 实机环境。真机运行需要已配置的机械臂、CAN 接口、深度相机及其 SDK，以及运动学和视觉依赖。不同机器的 Python、系统库和设备权限可能不同。

```bash
git clone https://github.com/Yang-Ci/reBot_RS_Camera-Follow.git
cd reBot_RS_Camera-Follow
```

安装离线测试依赖：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
```

真机程序使用 `.venv-arm`，参考环境为 Python 3.10，并需要 `requirements.txt` 中的依赖以及可用的相机 SDK、Pinocchio 和 motorbridge。可在满足系统依赖的机器上创建环境：

```bash
python3.10 -m venv .venv-arm
.venv-arm/bin/python -m pip install -r requirements.txt
```

控制库、URDF 和网页数字孪生资源不包含在 `vendor/` 中，需单独准备：

```bash
git clone https://github.com/LAN-GER/rebot_control.git vendor/rebot_control
git -C vendor/rebot_control checkout a901ec109ad99d7140c9b2110c618f54a9107ae9
```

还需获取 [reBotArm_control_py](https://github.com/Seeed-Projects/reBotArm_control_py) 的 RS URDF，以及网页使用的 [ReBot_Arm_DigitalTwin_RS](https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS)。可通过 `REBOT_CONTROL_PATH`、`REBOT_URDF_PATH` 和 `REBOT_TWIN_PATH` 指定本机位置；网页资源配置及电脑摄像头模拟准备见[网页说明](web/README.md)。

仓库内的 HOME、零位和标定数据对应原设备，**不能直接假定适用于另一台机械臂或新的相机安装**。连接真机前，应检查 CAN、关节零位、HOME、手眼标定、夹爪配置和工作空间。

## 启动网页

```bash
python3 web/server.py
```

打开 <http://127.0.0.1:8765>。网页本身可以在不连接机械臂时打开。选择 Gemini 336 并打开相机后，可先检查画面，再选择速度、夹爪模式和参数。

点击“初始化真机”会使能机械臂并移动到 HOME；到位后按 `R`、点击“开始跟随”，或持续张开手掌 3 秒开始。空格暂停；`Q` / `Esc` 请求退出并回零。初始化、退出和回零都可能产生真实运动，紧急情况使用硬件急停。

网页预设中，低速验证使用增益 1.5 / 1.5、末端速度上限 0.10 m/s；中速验证使用增益 1.9 / 1.9、速度上限 0.25 m/s。参数可在初始化前调整，具体设置和操作见[网页说明](web/README.md)。

仅进行 Gemini 336 视觉预览、不连接机械臂：

```bash
.venv-arm/bin/python palm_follow_grasp.py --camera gemini336 --no-arm
```

D435i 使用、手眼标定和更详细的控制说明见[技术手册](docs/技术手册.md)与[定稿技术文档](docs/B601手掌跟随系统_定稿技术文档.md)。历史文档中的参数应以当前配置和代码为准。

## 跟随与安全

短时手掌丢失按原跟随逻辑做有限外推，较长时间丢失时保持；重新识别后可能自动继续。深度丢失使用原有深度冻结与超时处理。Gemini 336 桌面跟随保留关节反馈和桌面高度软件保护，详细条件见[网页说明](web/README.md)。

这些保护依赖模型、标定和反馈，不能代替碰撞检测或硬件急停，也不能保证 HOME、回零、夹持物和整条机械臂的安全。首次运行或更换硬件后，应清空工作区、确认急停可用，并从低速开始验证。

## 测试

不连接真实机械臂的核心回归：

```bash
.venv/bin/python tools/run_regression.py
```

包含 Node.js 浏览器策略测试的扩展回归：

```bash
.venv/bin/python tools/run_regression.py --extended
```

这些测试使用假设备，不能替代现场安装、相机标定和低速实机验证。日志保存在本地 `logs/`，默认不提交到仓库。

## 目录

```text
config/                         跟随、夹爪、HOME 与标定配置
robot_follow/                   视觉、控制、硬件与运行时模块
web/                            本地网页与控制服务
models/wrist_camera/            相机模型与支架文件
tests/  tools/                  离线测试、诊断和标定工具
docs/                           技术与历史文档
```
