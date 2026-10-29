#!/usr/bin/env python3
"""Independent wall-clock watchdog between the mission and Scout driver."""
import copy
import math
import signal
import threading
import time
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('scout_command_watchdog', disable_signals=True)
    stop = threading.Event()
    lock = threading.Lock()
    last = [Twist(), float('-inf')]
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def receive(msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        valid = (all(math.isfinite(v) for v in values) and
                 0 <= msg.linear.x <= 1.0 and abs(msg.angular.z) <= 1.0 and
                 all(abs(v) < 1e-9 for v in values[1:5]))
        with lock:
            last[:] = [copy.deepcopy(msg) if valid else Twist(), time.monotonic()]

    rospy.Subscriber('/scout_dwa/vehicle_cmd', Twist, receive, queue_size=1)
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rospy.on_shutdown(stop.set)
    try:
        while not stop.is_set():
            with lock:
                msg = copy.deepcopy(last[0]) if time.monotonic() - last[1] <= .2 else Twist()
            pub.publish(msg)
            stop.wait(.02)
    finally:
        end = time.monotonic() + .5
        while time.monotonic() < end:
            pub.publish(Twist())
            time.sleep(.02)
        rospy.signal_shutdown('watchdog stopped')


if __name__ == '__main__':
    main()
