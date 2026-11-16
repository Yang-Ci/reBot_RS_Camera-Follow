"""Manage local HOME profiles; validation runs in the robot Python environment."""
import hashlib
import json
import math
import re
import subprocess
from pathlib import Path


class HomeManager:
    def __init__(self, project):
        self.project = Path(project)
        self.directory = self.project / 'config/home_profiles'

    def read(self, profile_id):
        if not isinstance(profile_id, str) or not re.fullmatch('[a-f0-9]{64}', profile_id):
            raise ValueError('HOME 选择无效')
        path = self.directory / (profile_id + '.json')
        if path.is_symlink():
            raise ValueError('HOME 文件无效')
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            name, pose = data['name'], data['home_joint_pos']
            if (not isinstance(name, str) or not name or len(name) > 40 or name.strip() != name
                    or hashlib.sha256(name.encode('utf-8')).hexdigest() != profile_id
                    or not isinstance(pose, list) or len(pose) != 6
                    or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in pose)):
                raise ValueError('HOME 文件内容无效')
            return {'id': profile_id, 'name': name,
                    'degrees': [math.degrees(x) for x in pose]}
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise ValueError('HOME 不存在或文件无效') from exc

    def list(self):
        entries = []
        for path in sorted(self.directory.glob('*.json')):
            try:
                entries.append(self.read(path.stem))
            except ValueError:
                continue
        return sorted(entries, key=lambda row: row['name'])

    def validate(self, name, degrees, *, save):
        try:
            proc = subprocess.run([str(self.project / '.venv-arm/bin/python'),
                                   str(self.project / 'tools/home_profile_cli.py')]
                                  + ([] if save else ['--check']),
                                  input=json.dumps({'name': name, 'degrees': degrees}, allow_nan=False),
                                  text=True, capture_output=True, timeout=10)
        except (OSError, subprocess.SubprocessError, TypeError, ValueError) as exc:
            raise ValueError('无法验证 HOME，请检查机械臂环境和输入') from exc
        if proc.returncode:
            detail = next((line[11:] for line in proc.stderr.splitlines()
                           if line.startswith('HOME_ERROR=')), '无法验证 HOME')
            raise ValueError(detail[:250])
        line = next((line[10:] for line in proc.stdout.splitlines() if line.startswith('HOME_JSON=')), None)
        if line is None:
            raise ValueError('HOME 验证没有返回结果')
        try:
            return json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError('HOME 验证结果无效') from exc

    def checked_path(self, profile_id):
        profile = self.read(profile_id)
        self.validate(profile['name'], profile['degrees'], save=False)
        return self.directory / (profile_id + '.json')
