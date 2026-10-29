#!/usr/bin/env python3
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from test_planner_lifecycle import LifecycleTest

class PathCostTest(LifecycleTest):
    def test_yaw_finish_and_upstream_goal_replacement(self):
        raw={'off':[],'on':[]}
        subs=[rospy.Subscriber('/test/raw_'+mode,Twist,lambda m,key=mode:raw[key].append((time.monotonic(),m))) for mode in raw]
        pub=rospy.Publisher('/test/path',Path,queue_size=1,latch=True)
        def run(y):
            path=Path(); path.header.frame_id='map'
            a=PoseStamped(); a.header.frame_id='map'; a.pose.position.x=-1; a.pose.position.y=y; a.pose.orientation.w=1
            b=PoseStamped(); b.header.frame_id='map'; b.pose.position.x=2; b.pose.position.y=y; b.pose.orientation.w=1
            path.poses=[a,b]
            pub.publish(path)
            _,_,start=self.pump(goal_x=2,obstacle=(-1.5,-1,.2),duration=1.8)
            values={key:[m.angular.z for t,m in messages if t>start+1] for key,messages in raw.items()}
            for key in values: self.assertTrue(values[key],key)
            return {key:sum(v)/len(v) for key,v in values.items()}
        time.sleep(.2)
        above=run(.25); below=run(-.25)
        self.assertGreater(above['on'],0.01,above)
        self.assertLess(below['on'],-0.01,below)
        self.assertAlmostEqual(above['off'],below['off'],places=4)
        self.assertGreater(abs(above['on']-above['off']),.01)
        for sub in subs: sub.unregister()

if __name__=='__main__':
    rospy.init_node('path_cost_selection_test')
    rostest.rosrun('scout_dwa','path_cost_selection',PathCostTest)
