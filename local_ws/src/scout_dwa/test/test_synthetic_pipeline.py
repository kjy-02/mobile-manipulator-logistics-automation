#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class SyntheticPipelineTest(unittest.TestCase):
    def setUp(self):
        self.mode = rospy.get_param("~obstacle_mode")
        self.expected_x = float(rospy.get_param("~expected_x", 1.0))
        self.expected_y = float(rospy.get_param("~expected_y", 0.5))
        self.expect_motion = bool(rospy.get_param("~expect_motion", True))
        self.messages = {}
        self.event = threading.Event()

        def receive(name):
            def callback(message):
                self.messages[name] = message
                self.event.set()

            return callback

        self.subscribers = [
            rospy.Subscriber("/scout_dwa/footprint", PolygonStamped, receive("footprint")),
            rospy.Subscriber("/scout_dwa/cmd_vel_raw", Twist, receive("raw")),
            rospy.Subscriber("/scout_dwa/cmd_vel_safe", Twist, receive("safe")),
            rospy.Subscriber(
                "/scout_dwa/dwa_planner/selected_trajectory", Marker, receive("selected")
            ),
            rospy.Subscriber(
                "/scout_dwa/dwa_planner/candidate_trajectories",
                MarkerArray,
                receive("candidates"),
            ),
        ]
        if self.mode == "map":
            self.subscribers.append(
                rospy.Subscriber("/scout_dwa/local_map", OccupancyGrid, receive("obstacle"))
            )
        else:
            self.subscribers.append(
                rospy.Subscriber("/scout_dwa/scan", LaserScan, receive("obstacle"))
            )

    def wait_for(self, names, timeout=15.0):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if all(name in self.messages for name in names):
                return
            self.event.wait(0.1)
            self.event.clear()
        self.fail("timed out waiting for: " + ", ".join(names))

    def test_pipeline(self):
        required = ["footprint", "obstacle", "raw", "safe", "selected", "candidates"]
        self.wait_for(required)

        footprint = self.messages["footprint"]
        self.assertEqual(footprint.header.frame_id, "base_link")
        self.assertEqual(len(footprint.polygon.points), 4)

        obstacle = self.messages["obstacle"]
        self.assertEqual(obstacle.header.frame_id, "base_link")
        self.assertFalse(obstacle.header.stamp.is_zero())
        if self.mode == "map":
            self.assertEqual(obstacle.info.origin.orientation.w, 1.0)
            x = int(math.floor((self.expected_x - obstacle.info.origin.position.x) /
                               obstacle.info.resolution))
            y = int(math.floor((self.expected_y - obstacle.info.origin.position.y) /
                               obstacle.info.resolution))
            occupied = []
            for offset_y in (-1, 0, 1):
                for offset_x in (-1, 0, 1):
                    cell_x = x + offset_x
                    cell_y = y + offset_y
                    if 0 <= cell_x < obstacle.info.width and 0 <= cell_y < obstacle.info.height:
                        occupied.append(obstacle.data[cell_x + cell_y * obstacle.info.width])
            self.assertIn(100, occupied)
        else:
            index = int(math.floor((math.atan2(self.expected_y, self.expected_x) - obstacle.angle_min) /
                                   obstacle.angle_increment))
            self.assertTrue(math.isfinite(obstacle.ranges[index]))
            self.assertAlmostEqual(obstacle.ranges[index],
                                   math.hypot(self.expected_x, self.expected_y), places=3)

        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            raw = self.messages.get("raw", Twist())
            safe = self.messages.get("safe", Twist())
            if self.expect_motion and raw.linear.x > 0.0 and safe.linear.x > 0.0:
                break
            if not self.expect_motion and "selected" in self.messages and abs(raw.linear.x) < 1e-9:
                break
            rospy.sleep(0.1)
        if self.expect_motion:
            self.assertGreater(self.messages["raw"].linear.x, 0.0)
            self.assertGreater(self.messages["safe"].linear.x, 0.0)
        else:
            # Observe a post-planning interval, including angular and unused axes.
            end = rospy.Time.now() + rospy.Duration(0.8)
            while rospy.Time.now() < end:
                for name in ("raw", "safe"):
                    command = self.messages[name]
                    for vector in (command.linear, command.angular):
                        for value in (vector.x, vector.y, vector.z):
                            self.assertTrue(math.isfinite(value))
                            self.assertAlmostEqual(value, 0.0, places=6)
                rospy.sleep(0.03)
        self.assertGreater(len(self.messages["selected"].points), 0)
        self.assertGreater(len(self.messages["candidates"].markers), 0)


if __name__ == "__main__":
    rospy.init_node("synthetic_pipeline_test")
    rostest.rosrun("scout_dwa", "synthetic_pipeline_test", SyntheticPipelineTest)
