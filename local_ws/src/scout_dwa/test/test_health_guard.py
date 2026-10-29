#!/usr/bin/env python3
import unittest
import rospy
import rostest
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from scout_dwa.msg import IntegrationHealth

class HealthGuardTest(unittest.TestCase):
    def setUp(self):
        self.received=[]
        self.raw=rospy.Publisher('/scout_dwa/cmd_vel_raw',Twist,queue_size=1)
        self.odom=rospy.Publisher('/scout/odom',Odometry,queue_size=1)
        self.grid=rospy.Publisher('/scout_dwa/local_map',OccupancyGrid,queue_size=1)
        self.health={n:rospy.Publisher('/scout_dwa/'+n+'_health',IntegrationHealth,queue_size=1) for n in ('input','sensor','localization')}
        self.sub=rospy.Subscriber('/scout_dwa/cmd_vel_safe',Twist,self.received.append)
        self.tf=tf.TransformBroadcaster()
    def pump(self,duration,valid_names):
        end=rospy.Time.now()+rospy.Duration(duration); rate=rospy.Rate(40)
        while rospy.Time.now()<end and not rospy.is_shutdown():
            now=rospy.Time.now(); cmd=Twist(); cmd.linear.x=.2; self.raw.publish(cmd)
            od=Odometry(); od.header.stamp=now; od.header.frame_id='odom'; od.child_frame_id='base_link'; od.pose.pose.orientation.w=1; self.odom.publish(od)
            grid=OccupancyGrid(); grid.header.stamp=now; grid.header.frame_id='base_link'; grid.info.resolution=.1; grid.info.width=1; grid.info.height=1; grid.info.origin.orientation.w=1; grid.data=[0]; self.grid.publish(grid)
            self.tf.sendTransform((0,0,0),(0,0,0,1),now,'base_link','map')
            for name,pub in self.health.items():
                h=IntegrationHealth(); h.header.stamp=now; h.valid=name in valid_names; h.reason='test'; pub.publish(h)
            rate.sleep()
    def test_all_health_gates_required(self):
        self.pump(.8,{'input','sensor'}); self.assertTrue(self.received); self.assertAlmostEqual(self.received[-1].linear.x,0.0)
        self.pump(.8,{'input','sensor','localization'}); self.assertAlmostEqual(self.received[-1].linear.x,.2)
        self.pump(.8,{'input','localization'}); self.assertAlmostEqual(self.received[-1].linear.x,0.0)

if __name__=='__main__':
    rospy.init_node('health_guard_test'); rostest.rosrun('scout_dwa','health_guard_test',HealthGuardTest)
