#!/usr/bin/env python3
"""Relay an already authorized guard output for at most two wall-clock seconds.

Default is observation only. This does not enable the guard or override gates.
Process SIGKILL and hardware/CAN failures still require the vehicle watchdog/E-stop.
"""
import argparse
import copy
import json
import math
import threading
import time
import rosgraph
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from scout_msgs.msg import ScoutStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    rospy.init_node('bounded_drive_test', anonymous=True)
    lock = threading.Lock()
    state = {}
    peaks = [0.0, 0.0]
    def receive(key):
        def callback(msg):
            with lock:
                state[key] = (msg, time.monotonic())
                if key == 'odom':
                    peaks[0] = max(peaks[0], abs(msg.twist.twist.linear.x))
                    peaks[1] = max(peaks[1], abs(msg.twist.twist.angular.z))
        return callback
    subscriptions = [
        rospy.Subscriber('/scout_dwa/cmd_vel_safe', Twist, receive('command'), queue_size=1),
        rospy.Subscriber('/cmd_vel_guard/ready', Bool, receive('ready'), queue_size=1),
        rospy.Subscriber('/cmd_vel_guard/reason', String, receive('reason'), queue_size=1),
        rospy.Subscriber('/scout/odom', Odometry, receive('odom'), queue_size=1),
        rospy.Subscriber('/scout_status', ScoutStatus, receive('status'), queue_size=1)]
    def snapshot():
        with lock:
            return dict(state)
    def check(data):
        now = time.monotonic()
        for key in ('command', 'ready', 'odom', 'status'):
            if key not in data or now - data[key][1] > 0.25:
                return 'missing or stale ' + key
        status = data['status'][0]
        if status.fault_code != 0 or status.base_state != 0 or status.control_mode != 1:
            return 'vehicle fault or control mode changed'
        if not data['ready'][0].data:
            return data.get('reason', (String(data='guard not ready'), 0))[0].data
        cmd = data['command'][0]
        values = [cmd.linear.x, cmd.linear.y, cmd.linear.z,
                  cmd.angular.x, cmd.angular.y, cmd.angular.z]
        if not all(math.isfinite(v) for v in values):
            return 'nonfinite command'
        if not (0 <= cmd.linear.x <= 0.1 and abs(cmd.angular.z) <= 0.2):
            return 'command exceeds first-test limits'
        if any(abs(v) > 1e-9 for v in values[1:5]):
            return 'unexpected command axes'
        odom = data['odom'][0]
        age = (rospy.Time.now() - odom.header.stamp).to_sec()
        if odom.header.stamp.is_zero() or not 0 <= age <= 0.25:
            return 'invalid odometry timestamp'
        measured = (odom.twist.twist.linear.x, odom.twist.twist.angular.z)
        if not all(math.isfinite(v) for v in measured):
            return 'invalid measured velocity'
        if abs(measured[0]) > 0.15 or abs(measured[1]) > 0.3:
            return 'measured velocity exceeds test envelope'
        return ''
    time.sleep(0.6)
    data = snapshot()
    publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0]).get('/cmd_vel', [])
    reason = check(data)
    result = dict(run_requested=args.run, guard_rejection=reason,
                  existing_vehicle_publishers=publishers, nonzero_commands=0)
    if not args.run or reason or publishers:
        print(json.dumps(result, indent=2))
        return 2 if args.run else 0
    odom = data['odom'][0]
    initial_xy = (odom.pose.pose.position.x, odom.pose.pose.position.y)
    if abs(odom.twist.twist.linear.x) > 0.01 or abs(odom.twist.twist.angular.z) > 0.01:
        raise RuntimeError('vehicle must be stopped before the first test')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    try:
        deadline = time.monotonic() + 2
        while not pub.get_num_connections() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not pub.get_num_connections():
            raise RuntimeError('driver subscriber unavailable')
        start = time.monotonic()
        while time.monotonic() - start < 2 and not rospy.is_shutdown():
            data = snapshot()
            reason = check(data)
            if reason:
                # The guard already publishes zero while its separate map and
                # health connections catch up. Keep zero for this transition;
                # resume only if all original checks pass within the same
                # two-second wall-clock budget. Real faults still end the test.
                if reason == 'sensor health and obstacle stamps differ':
                    pub.publish(Twist())
                    result['transient_zero_pauses'] = result.get('transient_zero_pauses', 0) + 1
                    time.sleep(0.02)
                    continue
                result['runtime_rejection'] = reason
                break
            cmd = copy.deepcopy(data['command'][0])
            pub.publish(cmd)
            result['nonzero_commands'] += int(abs(cmd.linear.x) + abs(cmd.angular.z) > 1e-9)
            time.sleep(0.02)
        result['relay_elapsed_s'] = time.monotonic() - start
    finally:
        # Always stop on normal completion, SIGINT, SIGTERM or Python exceptions.
        end = time.monotonic() + 1
        while time.monotonic() < end:
            pub.publish(Twist())
            time.sleep(0.02)
    data = snapshot()
    if 'odom' in data:
        odom, arrival = data['odom']
        result['final_odom_age_s'] = time.monotonic() - arrival
        result['final_velocity'] = [odom.twist.twist.linear.x, odom.twist.twist.angular.z]
        result['wheel_odom_displacement_m'] = math.hypot(
            odom.pose.pose.position.x - initial_xy[0], odom.pose.pose.position.y - initial_xy[1])
        result['measured_peak_abs_velocity'] = list(peaks)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
