#!/usr/bin/env python3
import math
import time
import unittest
import rospy
import rostest
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class FrontSectorTest(unittest.TestCase):
    def test_sector_boundary_and_independent_safety_map(self):
        maps, safety = [], []
        subs = [rospy.Subscriber('/scout_dwa/local_map', OccupancyGrid, maps.append),
                rospy.Subscriber('/scout_dwa/safety_map', OccupancyGrid, safety.append)]
        pub = rospy.Publisher('/front_sector_points', PointCloud2, queue_size=1)
        end = time.monotonic() + 3
        while (not pub.get_num_connections() or any(not s.get_num_connections() for s in subs)) and time.monotonic() < end:
            time.sleep(.02)
        self.assertGreater(pub.get_num_connections(), 0)
        for angle in (0, 59.9, -59.9, 60.1, -60.1, 90, 180):
            stamp = rospy.Time.now()
            radians = math.radians(angle)
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='base_link'),
                [(2 * math.cos(radians), 2 * math.sin(radians), .2)])
            pub.publish(cloud)
            end = time.monotonic() + 2
            while not (any(m.header.stamp == stamp for m in maps) and
                       any(m.header.stamp == stamp for m in safety)) and time.monotonic() < end:
                time.sleep(.02)
            planning = next(m for m in maps if m.header.stamp == stamp)
            full = next(m for m in safety if m.header.stamp == stamp)
            self.assertEqual(sum(v == 100 for v in planning.data), int(abs(angle) < 60), str(angle))
            self.assertEqual(sum(v == 100 for v in full.data), 1, 'side observation lost from safety map')
            self.assertEqual(planning.header, full.header)


if __name__ == '__main__':
    rospy.init_node('front_sector_test')
    rostest.rosrun('scout_dwa', 'front_sector', FrontSectorTest)
