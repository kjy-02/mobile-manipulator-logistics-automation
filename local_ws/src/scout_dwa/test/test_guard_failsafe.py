#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist


class GuardFailsafeTest(unittest.TestCase):
    def test_missing_inputs_force_zero(self):
        received = []
        publisher = rospy.Publisher("/scout_dwa/cmd_vel_raw", Twist, queue_size=1)
        subscriber = rospy.Subscriber(
            "/scout_dwa/cmd_vel_safe", Twist, lambda message: received.append(message)
        )

        command = Twist()
        command.linear.x = 0.25
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            publisher.publish(command)
            rate.sleep()

        self.assertGreater(len(received), 5)
        for message in received[-5:]:
            self.assertAlmostEqual(message.linear.x, 0.0, places=9)
            self.assertAlmostEqual(message.angular.z, 0.0, places=9)
        subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("guard_failsafe_test")
    rostest.rosrun("scout_dwa", "guard_failsafe_test", GuardFailsafeTest)
