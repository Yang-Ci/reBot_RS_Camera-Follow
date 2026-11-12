#!/usr/bin/env python3
"""Run the hardware-free core regression suite in isolated, sequential processes."""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Check:
    name: str
    arguments: tuple[str, ...]
    linux_only: bool = False
    executable: str = "python"


CHECKS = (
    Check('module tests', ('-m', 'unittest', 'discover', '-s', 'tests', '-v')),
    Check('servo algorithms', ('tools/test_servo_upgrades.py',)),
    Check('full fake-device smoke', ('tools/offline_smoke_test.py',)),
    Check('startup failures and camera recovery', ('tools/test_startup_cleanup.py',)),
    Check('gripper pipeline', ('tools/test_gripper_pipeline.py',)),
    Check('local display pipeline', ('tools/test_display_pipeline.py',)),
    Check('CLI failure status', ('tools/test_cli_exit.py',)),
    Check('live telemetry pipeline', ('tools/test_live_pipeline.py',)),
    Check('web command pipeline', ('tools/test_web_commands_pipeline.py',)),
    Check('web telemetry compatibility', ('tools/test_web_sync.py',), linux_only=True),
    Check('web process management', ('tools/test_robot_web_control.py',), linux_only=True),
    Check('web signal process', ('tools/test_robot_web_fake.py',), linux_only=True),
)


EXTENDED_CHECKS = (
    Check('automatic calibration (fake devices)', ('tools/test_auto_calibrate_d435i.py',)),
    Check('browser gesture policy', ('--test', 'tools/test_webcam_control.cjs'), executable='node'),
)


def run_check(check: Check, timeout: float = 120.0) -> tuple[bool, str]:
    env = os.environ.copy()
    env['PYTHONUTF8'] = '1'
    env['PYTHONIOENCODING'] = 'utf-8'
    executable = sys.executable if check.executable == 'python' else shutil.which(check.executable)
    if executable is None:
        return False, f'required executable not found: {check.executable}'
    started = time.monotonic()
    try:
        result = subprocess.run([executable, *check.arguments], cwd=ROOT, env=env,
                                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                errors='replace', timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        return False, f'timed out after {timeout:g}s: {exc}'
    except OSError as exc:
        return False, str(exc)
    elapsed = time.monotonic() - started
    if result.returncode:
        return False, f'exit {result.returncode} after {elapsed:.1f}s\n{result.stdout[-12000:]}'
    return True, f'{elapsed:.1f}s'


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='list checks without running them')
    parser.add_argument('--extended', action='store_true',
                        help='also run calibration and browser checks (requires SciPy and Node.js)')
    args = parser.parse_args(argv)
    failures = []
    for check in CHECKS + (EXTENDED_CHECKS if args.extended else ()):
        if args.list:
            suffix = ' (Linux only)' if check.linux_only else ''
            print(f'{check.name}{suffix}: {check.executable} {" ".join(check.arguments)}')
            continue
        if check.linux_only and not sys.platform.startswith('linux'):
            print(f'[SKIP] {check.name}: requires Linux', flush=True)
            continue
        ok, detail = run_check(check)
        print(f'[{"PASS" if ok else "FAIL"}] {check.name}: {detail}', flush=True)
        if not ok:
            failures.append(check.name)
    if failures:
        print('Failed checks: ' + ', '.join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
