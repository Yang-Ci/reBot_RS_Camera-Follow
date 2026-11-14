"""Command-line interface for the local robot runtime."""

import argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="手掌跟随 + 捏合抓取 一体程序")
    p.add_argument("--distance", type=float, default=None,
                   help="期望相机到掌心距离（米），建议 0.20~0.50；"
                        "不传则用 config/follow.yaml 的 follow.distance")
    p.add_argument("--gain", type=float, default=None,
                   help="竖直/深度方向位置增益（Y/Z）；"
                        "不传则用 follow.yaml 的 follow.gain")
    p.add_argument("--gain-x", type=float, default=None,
                   help="水平方向（左右跟随）位置增益，左右晃动就调低它；"
                        "不传则用 follow.yaml 的 follow.gain_x")
    p.add_argument("--max-vel", type=float, default=None,
                   help="最大末端线速度 m/s；不传则用 follow.yaml 的 follow.max_vel")
    p.add_argument("--camera", choices=["gemini2", "gemini305", "gemini336", "d435i"], default="gemini2",
                   help="相机型号：gemini2（默认）、gemini305、gemini336 或 d435i")
    p.add_argument("--gemini2-urdf-pose", action="store_true",
                   help="兼容旧命令：Gemini 336 使用 URDF 名义相机位姿")
    p.add_argument("--gemini336-pose", choices=["urdf", "calibration"], default="urdf",
                   help="Gemini 336 相机位姿来源，默认 urdf；可选 calibration 使用独立手眼结果")
    p.add_argument("--home-j34-tolerance", type=float, default=2.0, metavar="DEG",
                   help="普通 HOME 的 J3/J4 实际到位容差，范围 2~4°")
    p.add_argument("--home-speed", type=float, default=15.0, metavar="DEG_S",
                   help="普通 HOME 速度上限，范围 0~15°/s")
    p.add_argument("--stationary-joint-hold", action=argparse.BooleanOptionalAction,
                   default=None, help="手掌静止时保持关节指令位置；不指定时使用相机配置")
    p.add_argument("--keep-gripper-position", action="store_true",
                   help="与 --no-gripper 一起使用：保持夹爪当前位置")
    p.add_argument("--ir-emitter", choices=["auto", "on", "off"], default="auto",
                   help="红外投影：auto=纯视觉时关闭；on=保留投影；off=关闭（仅 --no-arm）")
    p.add_argument("--fps", type=int, default=None,
                   help="相机帧率，默认取 config/follow.yaml 的 follow.fps；"
                        "Gemini2 支持 60，高速跟随建议 60")
    p.add_argument("--follow-cfg",
                   default=None,
                   help="自定义跟随参数；未指定时使用 follow.yaml 和相机对应参数")
    p.add_argument("--grasp-cfg",
                   default=str(PROJECT_ROOT / "config" / "grasp.yaml"),
                   help="捏合→夹爪映射配置文件路径")
    p.add_argument("--detector-backend", default="auto",
                   choices=["auto", "gpu", "cpu"],
                   help="手掌检测推理后端：auto=优先GPU失败回退CPU（默认）")
    p.add_argument("--color-exposure", type=float, default=None, metavar="MS",
                   help="彩色手动曝光（毫秒）：不填=用 follow.yaml 的"
                        " color_exposure_ms（默认 0=自动）。快速挥手时"
                        "MediaPipe 整帧掉检（palm=None）可试 5~8——压运动"
                        "模糊，代价是画面变暗；暗光环境不要用")
    p.add_argument("--no-follow", action="store_true",
                   help="仅捏合夹爪：手臂保持不动（此模式无需手眼标定）")
    p.add_argument("--no-gripper", action="store_true",
                   help="仅手掌跟随：夹爪保持打开（不做捏合控制，减少遮挡）")
    p.add_argument("--no-arm", action="store_true",
                   help="纯视觉演练：不连接机械臂，只验证检测与映射")
    p.add_argument("--home-file", default=None,
                   help="使用指定的 HOME JSON；默认使用 config/home_pose.json")
    p.add_argument("--allow-home-limit-recovery", action="store_true",
                   help="允许从软限位外向内移动到近限位 J2 HOME（需先确认路径安全）")
    p.add_argument("--home-j2-command-offset", type=float, default=0.0, metavar="DEG",
                   help="近限位 HOME 的 J2 指令补偿（0~2°）；实际到位仍按示教角检查")
    p.add_argument("--headless", action="store_true",
                   help="无窗口模式（SSH/远程场景；按 q 键或 Ctrl+C 退出）")
    p.add_argument("--web-control", action="store_true",
                   help="由本机网页进程接收跟随启停信号，键盘快捷键由网页处理")
    p.add_argument("--web-auto-follow", action="store_true",
                   help="已停用：HOME 到位后需要主动启动跟随")
    return p.parse_args(argv)
