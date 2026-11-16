#!/usr/bin/env python3
"""Validate and save a named HOME using the project robot model, without CAN."""
import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rebot_adapter import (B601_RS_JOINT_NAMES, JOINT_OFFSET, URDF_PATH,
                           load_soft_joint_limits)
from robot_follow.runtime.home_profiles import save_profile, validate_home_pose


def validate(pose):
    model = pin.buildModelFromUrdf(str(URDF_PATH))
    lower, upper = load_soft_joint_limits()
    with (ROOT / 'config/follow.yaml').open(encoding='utf-8') as stream:
        workspace = yaml.safe_load(stream)['workspace']
    def fk(value):
        data = model.createData()
        full = pin.neutral(model)
        for index, name in enumerate(B601_RS_JOINT_NAMES):
            full[model.joints[model.getJointId(name)].idx_q] = value[index] + JOINT_OFFSET[index]
        pin.forwardKinematics(model, data, full)
        pin.updateFramePlacements(model, data)
        return data.oMi[model.getJointId(B601_RS_JOINT_NAMES[-1])].translation.copy()
    return validate_home_pose(pose, lower, upper, fk, workspace)


def main():
    try:
        data = json.load(sys.stdin)
        if not isinstance(data, dict) or set(data) != {'name', 'degrees'}:
            raise ValueError('HOME 数据格式无效')
        degrees = data['degrees']
        if (not isinstance(degrees, list) or len(degrees) != 6
                or any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in degrees)):
            raise ValueError('请输入 6 个有效关节角度')
        pose, position = validate(np.deg2rad(degrees))
        if sys.argv[1:] == ['--check']:
            result = {'valid': True}
        elif not sys.argv[1:]:
            path = save_profile(ROOT / 'config/home_profiles', data['name'], pose, position)
            result = {'id': path.stem, 'name': data['name'].strip(), 'degrees': np.rad2deg(pose).tolist()}
        else:
            raise ValueError('无效操作')
        print('HOME_JSON=' + json.dumps(result, ensure_ascii=False), flush=True)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        print('HOME_ERROR=' + str(exc), file=sys.stderr, flush=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
