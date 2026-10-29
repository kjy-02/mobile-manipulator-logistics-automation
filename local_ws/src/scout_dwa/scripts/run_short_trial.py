#!/usr/bin/env python3
"""Run one bounded trial and always send a final stop from the supervisor."""
import pathlib
import subprocess
import time
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

rospy.init_node('short_trial_supervisor', anonymous=True)
rospy.wait_for_service('/cmd_vel_guard/set_enabled', timeout=5)
set_enabled = rospy.ServiceProxy('/cmd_vel_guard/set_enabled', SetBool)
try:
    print(set_enabled(True), flush=True)
    time.sleep(.4)
    script = pathlib.Path(__file__).with_name('bounded_drive_test.py')
    result = subprocess.run(['python3', str(script), '--run'], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=12)
    print(result.stdout, flush=True)
    destination = pathlib.Path(__file__).parent.parent / 'validation/2026-09-30/latest_motion_result.log'
    destination.write_text(result.stdout + '\ntest_exit_code: ' + str(result.returncode) + '\n')
finally:
    # Stop independently even if the child fails or is killed by its deadline.
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    deadline = time.monotonic() + 1
    while not pub.get_num_connections() and time.monotonic() < deadline:
        time.sleep(.02)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        pub.publish(Twist())
        time.sleep(.02)
    print(set_enabled(False), flush=True)
