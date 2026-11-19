#!/usr/bin/env python3
"""One RViz or configured map goal; DWA output relayed through the independent watchdog."""
import argparse
import copy
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import rosgraph
import rosnode
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from scout_msgs.msg import ScoutStatus
from std_msgs.msg import Bool


class Navigation:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.data = {}
        self.goal = None
        self.goal_started = False
        self.cancel_reason = ''
        self.started_wall = time.monotonic()
        self.child = None
        self.goal_pub = rospy.Publisher('/scout_dwa/mission_goal', PoseStamped, queue_size=1, latch=True)
        self.output = rospy.Publisher('/scout_dwa/vehicle_cmd', Twist, queue_size=1)
        self.subscriptions = []
        for topic, cls, key in (
                ('/scout_dwa/cmd_vel_raw', Twist, 'command'),
                ('/scout/odom', Odometry, 'odom'),
                ('/scout_status', ScoutStatus, 'status'),
                ('/scout_dwa/dwa_planner/finish_flag', Bool, 'finish'),
                ('/scout_dwa/footprint', PolygonStamped, 'footprint')):
            self.subscriptions.append(rospy.Subscriber(topic, cls, self.receive(key), queue_size=1))
        if args.goal is None:
            self.subscriptions.append(rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb, queue_size=1))
        self.subscriptions.append(rospy.Subscriber('/initialpose', rospy.AnyMsg, self.initialpose_cb, queue_size=1))

    def receive(self, key):
        def callback(msg):
            with self.lock:
                self.data[key] = (msg, time.monotonic())
        return callback

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def goal_cb(self, msg):
        q = msg.pose.orientation
        values = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, q.x, q.y, q.z, q.w)
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        norm = math.sqrt(sum(v*v for v in values[3:]))
        if (msg.header.frame_id.lstrip('/') != 'map' or msg.header.stamp.is_zero() or
                not 0 <= age <= 5 or not all(math.isfinite(v) for v in values) or abs(norm - 1) > .01):
            rospy.logerr('잘못되었거나 오래된 목표를 거부했습니다. Fixed Frame=map에서 새 목표를 지정하세요.')
            return
        with self.lock:
            if self.goal_started:
                self.cancel_reason = '주행 중 새 목표 수신: 정지 후 새 실행이 필요합니다.'
                self.stop.set()
            elif self.goal is None:
                self.goal = copy.deepcopy(msg)
                self.goal.header.frame_id = 'map'
                self.goal.pose.orientation.x /= norm
                self.goal.pose.orientation.y /= norm
                self.goal.pose.orientation.z /= norm
                self.goal.pose.orientation.w /= norm

    def initialpose_cb(self, _):
        if self.goal is not None:
            self.cancel_reason = '주행 중 초기 위치 변경: 정지합니다.'
            self.stop.set()

    def check(self, data):
        now = time.monotonic()
        for key in ('command', 'odom', 'status'):
            if key not in data or now - data[key][1] > .3:
                return 'missing or stale ' + key
        status = data['status'][0]
        if status.fault_code or status.base_state != 0 or status.control_mode != 1:
            return 'vehicle fault or control mode changed'
        odom = data['odom'][0]
        age = (rospy.Time.now() - odom.header.stamp).to_sec()
        measured = (odom.twist.twist.linear.x, odom.twist.twist.angular.z)
        if (odom.header.stamp.is_zero() or not 0 <= age <= .3 or odom.child_frame_id != 'base_link' or
                not all(math.isfinite(v) for v in measured)):
            return 'invalid measured odometry'
        if abs(measured[0]) > 1.5 or abs(measured[1]) > 1.5:
            return 'measured velocity exceeds operating envelope'
        cmd = data['command'][0]
        values = (cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z)
        if (not all(math.isfinite(v) for v in values) or not 0 <= cmd.linear.x <= 1.0 or
                abs(cmd.angular.z) > 1.0 or any(abs(v) > 1e-9 for v in values[1:5])):
            return 'invalid or excessive command'
        return ''

    def arrived(self, data):
        return 'finish' in data and bool(data['finish'][0].data)

    def run(self):
        if self.args.goal is None:
            rospy.loginfo('RViz에서 2D Nav Goal로 새 목표를 지정하세요. 차량은 정지 대기 중입니다.')
        else:
            rospy.loginfo('고정 목표 사용: map X=%.3fm, Y=%.3fm, yaw=%.3frad. odometry 수신 후 주행을 시작합니다.',
                          *self.args.goal)
        while self.goal is None and not self.stop.is_set():
            self.output.publish(Twist())
            if time.monotonic()-self.started_wall > self.args.goal_wait_seconds:
                raise RuntimeError('목표 입력 대기 시간이 초과됐습니다.')
            if self.args.goal is not None:
                data = self.snapshot()
                if 'odom' in data and time.monotonic()-data['odom'][1] <= .3:
                    x, y, yaw = self.args.goal
                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.header.stamp = rospy.Time.now()
                    goal.pose.position.x = x
                    goal.pose.position.y = y
                    goal.pose.orientation.z = math.sin(yaw / 2)
                    goal.pose.orientation.w = math.cos(yaw / 2)
                    self.goal_cb(goal)
            self.stop.wait(.02)
        if self.stop.is_set():
            return 130
        data = self.snapshot()
        if 'odom' not in data or time.monotonic()-data['odom'][1] > .3:
            raise RuntimeError('정지 odometry를 확인할 수 없습니다.')
        v = data['odom'][0].twist.twist
        if not all(math.isfinite(x) for x in (v.linear.x, v.angular.z)) or abs(v.linear.x) > .01 or abs(v.angular.z) > .01:
            raise RuntimeError('차량이 정지한 상태에서 시작해야 합니다.')
        log = tempfile.NamedTemporaryFile(prefix='scout_goal_planner_', suffix='.log', delete=False)
        self.child = subprocess.Popen(['roslaunch', 'scout_dwa', 'low_speed_planner.launch',
                                       'goal_topic:=/scout_dwa/mission_goal'], stdout=log, stderr=subprocess.STDOUT)
        log.close()
        rospy.loginfo('원본 DWA 시작. planner 로그: %s', log.name)
        deadline = time.monotonic()+8
        while not self.goal_pub.get_num_connections() and not self.stop.is_set():
            self.output.publish(Twist())
            if self.child.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('원본 DWA 기동 실패: '+log.name)
            self.stop.wait(.02)
        if self.stop.is_set():
            return 130
        self.goal_started = True
        self.goal.header.stamp = rospy.Time.now()
        goal_wall = time.monotonic()
        self.goal_pub.publish(self.goal)
        deadline = goal_wall + 3
        rospy.loginfo('목표 발행 후 새 DWA 출력 대기 중입니다. 차량은 정지 상태를 유지합니다.')
        while not self.stop.is_set():
            self.output.publish(Twist())
            data = self.snapshot()
            if ('command' in data and data['command'][1] > goal_wall and
                    time.monotonic() - data['command'][1] <= .3):
                break
            if self.child.poll() is not None:
                raise RuntimeError('DWA 출력 대기 중 프로세스가 종료됐습니다.')
            if time.monotonic() > deadline:
                raise RuntimeError('목표 발행 후 새 DWA 출력이 수신되지 않았습니다.')
            self.stop.wait(.02)
        if self.stop.is_set():
            return 130
        rospy.loginfo('목표 주행 시작: x=%.3f, y=%.3f', self.goal.pose.position.x, self.goal.pose.position.y)
        start = time.monotonic()
        unhealthy_since = None
        while not self.stop.is_set():
            data = self.snapshot()
            if self.arrived(data):
                rospy.loginfo('목표 지점 도착! 정지 후 주행 프로그램을 종료합니다.')
                return 0
            if self.child.poll() is not None:
                raise RuntimeError('개선 DWA 프로세스가 종료됐습니다.')
            if time.monotonic()-start > self.args.max_seconds:
                raise RuntimeError('최대 주행 시간 초과: 도착하지 못해 정지합니다.')
            reason = self.check(data)
            if reason:
                self.output.publish(Twist())
                rospy.logwarn_throttle(2, '정지 상태: '+reason)
                if reason in ('vehicle fault or control mode changed', 'invalid measured odometry',
                              'measured velocity exceeds operating envelope', 'invalid or excessive command'):
                    raise RuntimeError(reason)
                unhealthy_since = unhealthy_since or time.monotonic()
                if time.monotonic()-unhealthy_since > 3:
                    raise RuntimeError('입력/감시 상태가 복구되지 않았습니다: '+reason)
            else:
                unhealthy_since = None
                self.output.publish(copy.deepcopy(data['command'][0]))
            self.stop.wait(.02)
        if self.cancel_reason:
            rospy.logwarn(self.cancel_reason)
        return 130

    def close(self):
        # Zero first; the independent watchdog also zeros on lost heartbeat.
        end = time.monotonic()+.5
        while time.monotonic()<end:
            self.output.publish(Twist())
            time.sleep(.02)
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.child.terminate()
                try:
                    self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.child.kill()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true', help='실제 차량 출력 연결')
    parser.add_argument('--max-seconds', type=float, default=300)
    parser.add_argument('--goal-wait-seconds', type=float, default=120)
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='고정 목표: map 기준 X(m) Y(m) 도착 방향 YAW(rad). 지정하면 RViz 목표 입력 없이 주행 시작')
    parser.add_argument('--map-pcd', default='/home/rilab/catkin_ws/map/m304.pcd')
    args = parser.parse_args()
    if args.goal is not None and not all(math.isfinite(v) for v in args.goal):
        parser.error('goal X, Y and YAW must be finite')
    if not args.run:
        parser.print_help()
        return 0
    if not all(math.isfinite(v) and 0 < v <= 1800 for v in (args.max_seconds, args.goal_wait_seconds)):
        parser.error('time limits must be finite and in (0,1800]')
    rospy.init_node('goal_navigation', anonymous=True, disable_signals=True)
    expected = os.path.realpath(os.path.expanduser(args.map_pcd))
    actual = os.path.realpath(rospy.get_param('/globalmap_server_nodelet/globalmap_pcd', ''))
    if not os.path.isfile(expected) or actual != expected:
        rospy.logerr('지도 경로가 일치하지 않습니다. 원하는 HDL 지도 launch를 먼저 실행하세요: %s', expected)
        return 2
    if '/scout_dwa/dwa_planner' in rosnode.get_node_names():
        rospy.logerr('기존 DWA를 종료하세요. 이 실행은 새 목표마다 개선 DWA를 새로 시작합니다.')
        return 2
    publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    if publishers.get('/cmd_vel') != ['/scout_command_watchdog'] or publishers.get('/scout_dwa/vehicle_cmd'):
        rospy.logerr('독립 command_watchdog 연결을 확인하세요. 다른 차량 명령 발행자가 있으면 시작하지 않습니다.')
        return 2
    navigation = Navigation(args)
    signal.signal(signal.SIGINT, lambda *_: navigation.stop.set())
    signal.signal(signal.SIGTERM, lambda *_: navigation.stop.set())
    rospy.on_shutdown(navigation.stop.set)
    result = 2
    try:
        result = navigation.run()
    except Exception as exc:
        rospy.logerr('주행 중단: %s', exc)
    finally:
        navigation.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
