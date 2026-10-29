#!/usr/bin/env python3
import copy
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from scout_dwa.msg import IntegrationHealth

class GatewayTest(unittest.TestCase):
    def setUp(self):
        self.health = None
        self.goals = []
        self.velocities = []
        self.subs = [rospy.Subscriber('/scout_dwa/input_health', IntegrationHealth, self.h),
                     rospy.Subscriber('/scout_dwa/validated_goal', PoseStamped, self.goals.append),
                     rospy.Subscriber('/scout_dwa/validated_target_velocity', Twist, self.velocities.append)]
        self.goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.path = rospy.Publisher('/scout_dwa/path_segment', Path, queue_size=1)
        self.velocity = rospy.Publisher('/target_velocity', Twist, queue_size=1)
        self.distance = rospy.Publisher('/dist_to_goal_th', Float64, queue_size=1)
        self.odom = rospy.Publisher('/scout/odom', Odometry, queue_size=1)
        deadline = time.monotonic()+3
        while not self.goal.get_num_connections() and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertGreater(self.goal.get_num_connections(), 0)
    def h(self, msg):
        self.health = msg
    def wait_health(self, valid):
        deadline = time.monotonic()+2
        while time.monotonic()<deadline:
            if self.health is not None and self.health.valid == valid:
                return
            time.sleep(.02)
        self.fail('health did not become %s' % valid)
    def send(self, pub, msg):
        pub.publish(msg); time.sleep(.12)
    def test_validation_and_downshift(self):
        good = PoseStamped(); good.header.frame_id='map'; good.pose.orientation.w=1
        good.pose.position.x=2
        bad = copy.deepcopy(good); bad.pose.orientation.w=0
        self.send(self.goal, bad); self.wait_health(False); self.assertEqual(len(self.goals),0)
        self.send(self.goal, good); self.wait_health(True); self.assertEqual(len(self.goals),1)
        bad.pose.orientation.w=1; bad.pose.position.x=float('nan')
        self.send(self.goal,bad); self.wait_health(False)
        self.send(self.distance,Float64(data=.1)); self.wait_health(False)
        self.send(self.goal,good); self.wait_health(True)
        self.send(self.path,Path()); self.wait_health(False)
        self.send(self.goal,good); self.wait_health(False)
        path=Path(); path.header.frame_id='map'
        a=copy.deepcopy(good); a.pose.position.x=0
        path.poses=[a,good]
        self.send(self.path,path); self.wait_health(True)
        target=Twist(); target.linear.x=-.1
        self.send(self.velocity,target); self.wait_health(False)
        self.send(self.goal,good); self.wait_health(False)
        target.linear.x=.3; self.send(self.velocity,target); self.wait_health(True)
        target.linear.x=.1; count=len(self.velocities)
        od=Odometry(); od.header.frame_id='odom'; od.child_frame_id='base_link'
        od.header.stamp=rospy.Time.now(); od.twist.twist.linear.x=.3
        self.send(self.odom,od); self.send(self.velocity,target); self.wait_health(False)
        self.assertEqual(len(self.velocities),count,'downshift forwarded before stopped')
        od.header.stamp=rospy.Time.now()-rospy.Duration(2); od.twist.twist.linear.x=0
        self.send(self.odom,od); self.assertEqual(len(self.velocities),count)
        od.header.stamp=rospy.Time.now()
        self.send(self.odom,od); self.wait_health(True)
        self.assertAlmostEqual(self.velocities[-1].linear.x,.1)

if __name__=='__main__':
    rospy.init_node('input_gateway_test')
    rostest.rosrun('scout_dwa','input_gateway_test',GatewayTest)
