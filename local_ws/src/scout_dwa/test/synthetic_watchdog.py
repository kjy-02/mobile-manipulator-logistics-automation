#!/usr/bin/env python3
"""Test fixture only: monotonic downstream timeout on an isolated /test topic."""
import threading
import time
import rospy
from geometry_msgs.msg import Twist
rospy.init_node('synthetic_watchdog')
lock=threading.Lock()
latest=Twist()
arrival=0.0
pub=rospy.Publisher('/test/downstream_cmd',Twist,queue_size=1)
def receive(msg):
    global latest,arrival
    with lock:
        latest=msg; arrival=time.monotonic()
rospy.Subscriber('/scout_dwa/cmd_vel_safe',Twist,receive)
while not rospy.is_shutdown():
    with lock:
        pub.publish(latest if time.monotonic()-arrival<.2 else Twist())
    time.sleep(.02)
