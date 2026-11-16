"""Nominal TCP-to-camera transform from the digital twin's URDFs."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_GEMINI2_URDF = Path(__file__).resolve().parent / 'models/wrist_camera/gemini2_level.urdf'
DEFAULT_D435I_URDF = Path(__file__).resolve().parent / 'models/wrist_camera/d435i_level.urdf'
DEFAULT_GEMINI305_URDF = Path(__file__).resolve().parent / 'models/wrist_camera/gemini305_centered.urdf'
DEFAULT_GEMINI336_URDF = Path(__file__).resolve().parent / 'models/wrist_camera/gemini336_level.urdf'


def _origin_matrix(origin):
    xyz = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()])
    rpy = np.array([float(v) for v in origin.get('rpy', '0 0 0').split()])
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.all(np.isfinite(np.r_[xyz, rpy])):
        raise ValueError('URDF joint origin 必须包含有限的 xyz 和 rpy')
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]]
    result[:3, 3] = xyz
    return result


def _fixed_chain(root, start, end):
    by_child = {}
    for joint in root.findall('joint'):
        child, parent = joint.find('child'), joint.find('parent')
        if child is None or parent is None:
            continue
        name = child.get('link')
        if name in by_child:
            raise ValueError(f'URDF link 有多个父关节: {name}')
        by_child[name] = joint
    chain, current, visited = [], end, set()
    while current != start:
        if current in visited or current not in by_child:
            raise ValueError(f'URDF 中没有从 {start} 到 {end} 的关节链')
        visited.add(current)
        joint = by_child[current]
        if joint.get('type') != 'fixed':
            raise ValueError(f'相机链包含非固定关节: {joint.get("name")}')
        origin = joint.find('origin')
        chain.append(_origin_matrix(origin if origin is not None else ET.Element('origin')))
        current = joint.find('parent').get('link')
    result = np.eye(4)
    for transform in reversed(chain):
        result = result @ transform
    return result


def gemini2_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf=DEFAULT_GEMINI2_URDF, *, optical_frame="camera_color_optical_frame"):
    """joint6 TCP -> Gemini 2 color optical frame, assuming identical mounting."""
    arm = ET.parse(arm_urdf).getroot()
    camera = ET.parse(camera_urdf).getroot()
    joint6 = arm.find("joint[@name='joint6']")
    if joint6 is None or joint6.find('child') is None:
        raise ValueError('机械臂 URDF 缺少 joint6 的子 link')
    offset = np.asarray(tool_offset, dtype=float)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise ValueError('TCP 偏移无效')
    tcp_to_joint6 = np.eye(4)
    tcp_to_joint6[:3, 3] = -offset
    return (tcp_to_joint6 @ _fixed_chain(arm, joint6.find('child').get('link'), 'gripper_end')
            @ _fixed_chain(camera, 'gripper_end', optical_frame))


def gemini336_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf=DEFAULT_GEMINI336_URDF):
    """TCP -> color optical frame using the shared 336 assembly URDF."""
    return gemini2_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf)


def gemini305_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf=DEFAULT_GEMINI305_URDF):
    """Nominal centered, level mount; verify assembly before any movement."""
    return gemini2_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf)


def d435i_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf=DEFAULT_D435I_URDF, *, has_color=True):
    """Use the optical frame of the image stream used for detection."""
    frame = "camera_color_optical_frame" if has_color else "camera_infra1_optical_frame"
    return gemini2_nominal_tcp_camera(arm_urdf, tool_offset, camera_urdf, optical_frame=frame)
