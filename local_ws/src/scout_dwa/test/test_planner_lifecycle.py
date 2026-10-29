#!/usr/bin/env python3
import math
import time
import unittest
import rospy
import rostest
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, Bool
from scout_dwa.msg import IntegrationHealth
from visualization_msgs.msg import Marker

class LifecycleTest(unittest.TestCase):
    def setUp(self):
        self.raw=[]; self.safe=[]; self.finished=[]; self.markers=[]
        self.subs=[rospy.Subscriber('/scout_dwa/cmd_vel_raw',Twist,lambda m:self.raw.append((time.monotonic(),m))),
                   rospy.Subscriber('/scout_dwa/cmd_vel_safe',Twist,lambda m:self.safe.append((time.monotonic(),m))),
                   rospy.Subscriber('/scout_dwa/dwa_planner/finish_flag',Bool,lambda m:self.finished.append((time.monotonic(),m.data))),
                   rospy.Subscriber('/scout_dwa/dwa_planner/selected_trajectory',Marker,self.markers.append)]
        self.cloud=rospy.Publisher('/scout_dwa/test_points',point_cloud2.PointCloud2,queue_size=1)
        self.goal=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        self.odom=rospy.Publisher('/scout/odom',Odometry,queue_size=1)
        self.health={key:rospy.Publisher('/scout_dwa/'+key+'_health',IntegrationHealth,queue_size=1) for key in ('input','localization')}
        self.tf=tf.TransformBroadcaster()
        time.sleep(.3)
    def pump(self, x=0, yaw=0, goal_x=1, obstacle=(1.5,1.5,.2), duration=1.3):
        start=time.monotonic()
        while time.monotonic()-start<duration:
            stamp=rospy.Time.now()
            self.tf.sendTransform((x,0,0),tf.transformations.quaternion_from_euler(0,0,yaw),stamp,'base_link','map')
            goal=PoseStamped(); goal.header=Header(stamp=stamp,frame_id='map'); goal.pose.position.x=goal_x
            q=tf.transformations.quaternion_from_euler(0,0,math.pi/2)
            goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w=q
            self.goal.publish(goal)
            od=Odometry(); od.header=Header(stamp=stamp,frame_id='odom'); od.child_frame_id='base_link'
            od.pose.pose.orientation.w=1; self.odom.publish(od)
            self.cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='base_link'),[obstacle]))
            for pub in self.health.values():
                h=IntegrationHealth(); h.header.stamp=stamp; h.valid=True; pub.publish(h)
            time.sleep(.03)
        return ([m for t,m in self.raw if t>start+duration-.4],
                [m for t,m in self.safe if t>start+duration-.4], start)
    def test_yaw_finish_and_upstream_goal_replacement(self):
        circle=rospy.get_param('~circle',False)
        self.assertEqual(rospy.get_param('/scout_dwa/dwa_planner/USE_FOOTPRINT'),not circle)
        raw,safe,_=self.pump()
        self.assertTrue(any(m.linear.x>0 for m in raw),'no initial translation')
        self.assertTrue(any(m.linear.x>0 for m in safe),'guard did not pass translation')
        raw,safe,_=self.pump(x=1,obstacle=(.1,0,.2))
        self.assertTrue(raw); self.assertTrue(safe)
        self.assertTrue(all(abs(m.linear.x)<1e-9 and m.angular.z>0 for m in raw),'final yaw not exercised')
        self.assertTrue(all(abs(m.linear.x)<1e-9 and abs(m.angular.z)<1e-9 for m in safe),'final rotation veto failed')
        raw,_,_=self.pump(x=1,goal_x=2)
        self.assertTrue(all(abs(m.linear.x)<1e-9 and m.angular.z>0 for m in raw),
                        'recorded upstream has_reached goal replacement limitation changed')
        raw,safe,start=self.pump(x=1,yaw=math.pi/2,duration=1.5)
        self.assertTrue(all(abs(m.linear.x)<1e-9 and abs(m.angular.z)<1e-9 for m in raw))
        self.assertGreater(sum(v for t,v in self.finished if t>start+.4),1,'repeated upstream finish flag not observed')
        self.assertTrue(self.markers)
        for marker in self.markers:
            self.assertEqual(marker.header.frame_id,'base_link')
            self.assertTrue(all(math.isfinite(v) for p in marker.points for v in (p.x,p.y,p.z)))

if __name__=='__main__':
    rospy.init_node('planner_lifecycle_test')
    rostest.rosrun('scout_dwa','planner_lifecycle',LifecycleTest)
