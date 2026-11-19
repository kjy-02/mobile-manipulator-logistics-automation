#!/usr/bin/env python3
"""Start the ROS preparation environment; the user starts the mission separately."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


# Do not replace nodes launched by another terminal.
ENVIRONMENT_NODES = {
    '/scout_base_node', '/velodyne_nodelet_manager', '/dwa_localization_nodelet_manager',
    '/globalmap_server_nodelet', '/hdl_localization_nodelet', '/scout_legacy_sensor_tf',
    '/cmd_vel_guard', '/localization_monitor', '/scout_command_watchdog',
    '/scout_dwa_rviz', '/trajectory_display', '/cloud_obstacle_adapter',
    '/static_footprint_publisher', '/camera/realsense2_camera_manager',
    '/camera/realsense2_camera', '/move_group', '/xarm/uf_ros_controller',
    '/joint_state_publisher', '/robot_state_publisher', '/xarm/traj_controller_spawner',
    '/scout_dwa/dwa_planner', '/yolo_detector', '/yolo_detector_node',
    '/xarm_vision_pick_drop', '/xarm_vision_pick_drop_py',
}
REQUIRED_PUBLISHERS = {
    '/scout/odom', '/scout_status', '/velodyne_points', '/globalmap',
    '/camera/color/image_raw', '/camera/depth/image_rect_raw', '/xarm/joint_states',
    '/xarm/xarm_gripper/gripper_action/status',
}


def system_state():
    import rosgraph
    try:
        return rosgraph.Master('/scout_xarm_start_check').getSystemState()
    except (OSError, rosgraph.MasterError, rosgraph.MasterFailure):
        return None


def main():
    parser = argparse.ArgumentParser(description='CAN 및 ROS 준비 환경 실행; 주행·파지는 별도 명령으로 시작')
    parser.add_argument('--robot-ip', default='192.168.1.210')
    parser.add_argument('--map-pcd', default='/home/rilab/catkin_ws/map/m304.pcd')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    import rospkg
    packages = rospkg.RosPack()
    for name in ('scout_dwa', 'dwa_planner', 'use_yolo'):
        if Path(packages.get_path(name)).resolve() != root / 'local_ws' / 'src' / name:
            print('복사본 환경이 적용되지 않았습니다. start_all.sh로 실행하세요.', file=sys.stderr)
            return 2
    if not Path(args.map_pcd).is_file():
        parser.error('지도 파일이 없습니다: ' + args.map_pcd)

    state = system_state()
    if state is not None:
        nodes = {node for entries in state for _, names in entries for node in names}
        existing = sorted(nodes & ENVIRONMENT_NODES)
        if existing:
            print('중복 실행을 피하려면 기존 개별 실행을 종료하세요: ' + ', '.join(existing), file=sys.stderr)
            return 2
        commands = dict(state[0])
        if commands.get('/cmd_vel') or commands.get('/scout_dwa/vehicle_cmd'):
            print('기존 차량 명령 발행자가 있어 시작하지 않습니다.', file=sys.stderr)
            return 2

    link = subprocess.run(['ip', '-json', 'link', 'show', 'can0'], capture_output=True, text=True)
    if link.returncode != 0:
        print('can0를 찾을 수 없습니다: ' + link.stderr.strip(), file=sys.stderr)
        return 2
    if 'UP' not in json.loads(link.stdout)[0]['flags']:
        print('CAN 활성화: 필요하면 터미널에서 sudo 비밀번호를 입력하세요.', flush=True)
        subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can',
                        'bitrate', '500000'], check=True)

    stop = threading.Event()
    children = []
    pumps = []

    def interrupt(_signum, _frame):
        stop.set()
        for child in children:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def start(command, log_path):
        print('실행: ' + ' '.join(command), flush=True)
        log = open(log_path, 'w', buffering=1)
        try:
            child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        except BaseException:
            log.close()
            raise
        children.append(child)

        def copy_output():
            try:
                for line in child.stdout:
                    log.write(line)
                    print(line, end='', flush=True)
                    if 'Detected protocol: UNKNOWN' in line:
                        print('Scout CAN protocol을 확인할 수 없어 통합 실행을 중단합니다.', file=sys.stderr)
                        stop.set()
            finally:
                child.stdout.close()
                log.close()

        pump = threading.Thread(target=copy_output, daemon=True)
        pump.start()
        pumps.append(pump)
        return child

    try:
        if stop.is_set():
            return 130
        environment = start(['roslaunch', str(root / 'launch/system.launch'),
                             'robot_ip:=' + args.robot_ip, 'map_pcd_path:=' + args.map_pcd],
                            '/tmp/scout_system.log')
        print('기존 ROS 구성 기동 대기 중입니다. RViz에서 2D Pose Estimate로 현재 위치를 지정하세요.', flush=True)
        deadline = time.monotonic() + 120
        last_report = 0
        while not stop.is_set():
            if environment.poll() is not None:
                raise RuntimeError('ROS 환경 launch가 종료됐습니다. /tmp/scout_system.log를 확인하세요.')
            state = system_state()
            missing = REQUIRED_PUBLISHERS - set(dict(state[0])) if state is not None else REQUIRED_PUBLISHERS
            publishers = dict(state[0]) if state is not None else {}
            if not missing and publishers.get('/cmd_vel') == ['/scout_command_watchdog']:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('기동 시간 초과. 미연결 토픽: ' + ', '.join(sorted(missing)))
            if time.monotonic() - last_report > 5:
                print('기동 대기: ' + ', '.join(sorted(missing)), flush=True)
                last_report = time.monotonic()
            stop.wait(.5)
        if stop.is_set():
            return 130
        print('준비 환경 연결 완료. 이 터미널은 켜두고, 별도 터미널에서 최종 주행·파지 명령을 실행하세요.', flush=True)
        print('별도 터미널 환경 설정: source ' + str(root / 'local_ws/devel/setup.bash'), flush=True)
        while not stop.is_set():
            if environment.poll() is not None:
                raise RuntimeError('실행 중 ROS 환경 launch가 종료됐습니다.')
            stop.wait(.2)
        return 130
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print('실행 중단: ' + str(exc), file=sys.stderr)
        return 2
    finally:
        interrupt(signal.SIGINT, None)
        # Shut down only the preparation environment started here.
        for child in reversed(children):
            try:
                child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        for pump in pumps:
            pump.join(timeout=2)


if __name__ == '__main__':
    raise SystemExit(main())
