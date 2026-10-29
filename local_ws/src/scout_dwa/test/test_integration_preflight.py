#!/usr/bin/env python3
import time
import unittest
import rospy
import rostest
import rosgraph
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from scout_dwa.msg import IntegrationHealth

class PreflightTest(unittest.TestCase):
    def test_effective_config_disabled_output_and_session(self):
        config=[]; goals=[]; reasons=[]; safe=[]
        subs=[rospy.Subscriber('/scout_dwa/config_health',IntegrationHealth,config.append),
              rospy.Subscriber('/scout_dwa/validated_goal',PoseStamped,goals.append),
              rospy.Subscriber('/cmd_vel_guard/reason',String,lambda m:reasons.append(m.data)),
              rospy.Subscriber('/scout_dwa/cmd_vel_safe',Twist,safe.append)]
        deadline=time.monotonic()+8
        while not config and time.monotonic()<deadline: time.sleep(.05)
        self.assertTrue(config); self.assertTrue(config[-1].valid)
        self.assertEqual(rospy.get_param('/scout_dwa/dwa_planner/ROBOT_FRAME'),'custom_base')
        self.assertEqual(rospy.get_param('/cloud_obstacle_adapter/target_frame'),'custom_base')
        self.assertEqual(rospy.get_param('/static_footprint_publisher/frame_id'),'custom_base')
        publishers=rosgraph.Master(rospy.get_name()).getSystemState()[0]
        self.assertFalse(any(topic=='/cmd_vel' and nodes for topic,nodes in publishers))
        rospy.wait_for_service('/cmd_vel_guard/set_enabled',timeout=3)
        rospy.ServiceProxy('/cmd_vel_guard/set_enabled',SetBool)(True)
        time.sleep(.2)
        self.assertIn('hardware verification pending or simulation time enabled',reasons)
        self.assertTrue(safe)
        self.assertTrue(all(m.linear.x==0 and m.angular.z==0 for m in safe))
        pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        odom=rospy.Publisher('/scout/odom',Odometry,queue_size=1)
        time.sleep(.2)
        goal=PoseStamped(); goal.header.frame_id='map'; goal.pose.orientation.w=1; goal.pose.position.x=2
        pub.publish(goal); time.sleep(.2)
        self.assertTrue(goals); self.assertEqual(goals[-1].pose.position.x,2)
        goal.pose.position.x=3; pub.publish(goal); time.sleep(.2)
        self.assertEqual(goals[-1].pose.position.x,2,'replacement bypassed new session')
        od=Odometry(); od.header.frame_id='odom'; od.child_frame_id='custom_base'; od.header.stamp=rospy.Time.now()
        odom.publish(od); time.sleep(.05)
        rospy.wait_for_service('/scout_dwa/planner_preflight/restart_session',timeout=3)
        result=rospy.ServiceProxy('/scout_dwa/planner_preflight/restart_session',Trigger)()
        self.assertTrue(result.success,result.message)
        time.sleep(.3)
        self.assertEqual(goals[-1].pose.position.x,3)
        self.assertIn('disabled',reasons)
        for sub in subs: sub.unregister()

if __name__=='__main__':
    rospy.init_node('integration_preflight_test')
    rostest.rosrun('scout_dwa','integration_preflight',PreflightTest)
