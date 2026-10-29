#!/usr/bin/env python3

import unittest

import rospy
import rostest
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry


class GuardFreshnessTest(unittest.TestCase):
    def setUp(self):
        self.safe_messages = []
        self.raw_pub = rospy.Publisher("/scout_dwa/cmd_vel_raw", Twist, queue_size=1)
        self.odom_pub = rospy.Publisher("/scout/odom", Odometry, queue_size=1)
        self.map_pub = rospy.Publisher("/scout_dwa/local_map", OccupancyGrid, queue_size=1)
        self.safe_sub = rospy.Subscriber(
            "/scout_dwa/cmd_vel_safe", Twist, self.safe_messages.append
        )
        self.tf_broadcaster = tf.TransformBroadcaster()

    def publish_for(self, duration, odom_stamp_fresh=True, publish_tf=True):
        end = rospy.Time.now() + rospy.Duration(duration)
        stale_stamp = rospy.Time.now() - rospy.Duration(5.0)
        command = Twist()
        command.linear.x = 0.20
        rate = rospy.Rate(40)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            now = rospy.Time.now()
            self.raw_pub.publish(command)

            odom = Odometry()
            odom.header.stamp = now if odom_stamp_fresh else stale_stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)

            local_map = OccupancyGrid()
            local_map.header.stamp = now
            local_map.header.frame_id = "base_link"
            local_map.info.resolution = 0.1
            local_map.info.width = 1
            local_map.info.height = 1
            local_map.info.origin.orientation.w = 1.0
            local_map.data = [0]
            self.map_pub.publish(local_map)

            if publish_tf:
                self.tf_broadcaster.sendTransform(
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    now,
                    "base_link",
                    "map",
                )
            rate.sleep()

    def latest_is(self, expected):
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.safe_messages and abs(self.safe_messages[-1].linear.x - expected) < 1e-6:
                return True
            rospy.sleep(0.02)
        return False

    def test_stale_odom_and_tf_force_zero(self):
        self.publish_for(1.0)
        self.assertTrue(self.latest_is(0.20), "fresh inputs did not pass through")

        self.publish_for(0.5, odom_stamp_fresh=False)
        self.assertTrue(self.latest_is(0.0), "stale odometry stamp was accepted")

        self.publish_for(0.5)
        self.assertTrue(self.latest_is(0.20), "guard did not recover with fresh odometry")

        self.publish_for(0.5, publish_tf=False)
        self.assertTrue(self.latest_is(0.0), "stale TF was accepted")


if __name__ == "__main__":
    rospy.init_node("guard_freshness_test")
    rostest.rosrun("scout_dwa", "guard_freshness_test", GuardFreshnessTest)
