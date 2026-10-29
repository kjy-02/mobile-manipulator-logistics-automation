#!/usr/bin/env python3
"""Run the existing DWA mission, then the existing YOLO/xArm launch unchanged."""
import argparse
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='DWA 목적지 도착 및 종료 후 기존 YOLO/xArm launch 실행')
    parser.add_argument('--run', action='store_true', help='기존 실차 DWA 주행 및 xArm 순차 실행')
    args, navigation_args = parser.parse_known_args()
    if not args.run:
        parser.print_help()
        return 0

    import rospkg
    local_source = Path(__file__).resolve().parent / 'local_ws' / 'src'
    packages = rospkg.RosPack()
    for name in ('scout_dwa', 'dwa_planner', 'use_yolo'):
        if Path(packages.get_path(name)).resolve() != (local_source / name).resolve():
            print('복사본 환경을 먼저 적용하세요: source ' +
                  str(local_source.parent / 'devel/setup.bash'), file=sys.stderr)
            return 2

    child = None
    interrupted = False

    def stop(signum, _frame):
        nonlocal interrupted
        interrupted = True
        if child is not None and child.poll() is None:
            child.send_signal(signum)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def run(command):
        nonlocal child
        if interrupted:
            return 130
        print('실행: ' + ' '.join(command), flush=True)
        child = subprocess.Popen(command)
        if interrupted:
            child.send_signal(signal.SIGINT)
        return child.wait()

    try:
        result = run(['rosrun', 'scout_dwa', 'run_goal_navigation.py', '--run'] + navigation_args)
        if interrupted:
            return 130
        if result != 0:
            print('DWA가 도착 성공으로 종료되지 않아 xArm을 실행하지 않습니다.', file=sys.stderr)
            return result if result > 0 else 128 - result
        print('DWA 주행 및 종료 완료. 기존 YOLO/xArm launch를 실행합니다.', flush=True)
        result = run(['roslaunch', 'use_yolo', 'vision_pick_drop.launch'])
        return 130 if interrupted else (result if result >= 0 else 128 - result)
    except OSError as exc:
        print('실행 실패: ' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
