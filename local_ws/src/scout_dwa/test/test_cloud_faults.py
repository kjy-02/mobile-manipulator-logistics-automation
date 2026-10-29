#!/usr/bin/env python3
import copy
import math
import struct
import time
import unittest
import rospy
import rostest
import tf
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from scout_dwa.msg import IntegrationHealth

class CloudFaultsTest(unittest.TestCase):
    def setUp(self):
        self.maps=[]; self.scans=[]; self.health=[]
        self.subs=[rospy.Subscriber('/scout_dwa/local_map',OccupancyGrid,self.maps.append),
                   rospy.Subscriber('/scout_dwa/scan',LaserScan,self.scans.append),
                   rospy.Subscriber('/scout_dwa/sensor_health',IntegrationHealth,self.health.append)]
        self.pub=rospy.Publisher('/test_cloud',point_cloud2.PointCloud2,queue_size=1)
        self.broadcaster=tf.TransformBroadcaster()
        deadline=time.monotonic()+3
        while not self.pub.get_num_connections() and time.monotonic()<deadline: time.sleep(.02)
        self.assertGreater(self.pub.get_num_connections(),0)
    def send(self, msg, expected):
        count=len(self.health); maps=len(self.maps)
        self.pub.publish(msg)
        deadline=time.monotonic()+2
        while len(self.health)==count and time.monotonic()<deadline: time.sleep(.02)
        self.assertGreater(len(self.health),count)
        self.assertEqual(self.health[-1].valid,expected,self.health[-1].reason)
        time.sleep(.04)
        if not expected: self.assertEqual(len(self.maps),maps,'invalid cloud emitted map')
    def cloud(self, points, frame='base_link'):
        return point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id=frame),points)
    def test_layout_filter_stamp_and_transform(self):
        cloud=self.cloud([(1,0,.2)])
        self.send(cloud,True)
        self.assertEqual(self.maps[-1].header.stamp,cloud.header.stamp)
        scan=self.scans[-1]
        self.assertTrue(any(math.isfinite(r) and abs(r-1)<.001 for r in scan.ranges),'centerline lost')
        for fault in ('offset','step','row','data','duplicate','endian','zero','stale','future','nan','empty','tf'):
            msg=self.cloud([(1,0,.2)])
            if fault=='offset': msg.fields[0].offset=msg.point_step
            elif fault=='step': msg.point_step=4
            elif fault=='row': msg.row_step=1
            elif fault=='data': msg.data=msg.data[:-1]
            elif fault=='duplicate': msg.fields.append(copy.deepcopy(msg.fields[0]))
            elif fault=='endian': msg.is_bigendian=True
            elif fault=='zero': msg.header.stamp=rospy.Time()
            elif fault=='stale': msg.header.stamp-=rospy.Duration(2)
            elif fault=='future': msg.header.stamp+=rospy.Duration(2)
            elif fault=='nan': msg=self.cloud([(float('nan'),0,.2)])
            elif fault=='empty': msg=self.cloud([])
            elif fault=='tf': msg.header.frame_id='missing_frame'
            self.send(msg,False)
        # Organized rows include explicit padding; each FLOAT32 point remains intact.
        msg=self.cloud([(1,0,.2),(1,.2,.2)])
        msg.height=2; msg.width=1; msg.row_step=16
        msg.data=msg.data[:12]+bytes(4)+msg.data[12:]+bytes(4)
        self.send(msg,True)
        # Rotation about z followed by translation: sensor (1,0,0) -> base (1,1,0).
        for _ in range(20):
            stamp=rospy.Time.now()
            self.broadcaster.sendTransform((1,0,0),tf.transformations.quaternion_from_euler(0,0,math.pi/2),stamp,'sensor_test','base_link')
            time.sleep(.02)
        msg=self.cloud([(1,0,0)],'sensor_test')
        msg.header.stamp=stamp
        self.send(msg,True)
        grid=self.maps[-1]
        x=int(math.floor((1-grid.info.origin.position.x)/grid.info.resolution))
        y=int(math.floor((1-grid.info.origin.position.y)/grid.info.resolution))
        self.assertEqual(grid.data[x+y*grid.info.width],100)
        self.assertTrue(any(math.isfinite(r) and abs(r-math.sqrt(2))<.001 for r in self.scans[-1].ranges))

if __name__=='__main__':
    rospy.init_node('cloud_faults_test')
    rostest.rosrun('scout_dwa','cloud_faults_test',CloudFaultsTest)
