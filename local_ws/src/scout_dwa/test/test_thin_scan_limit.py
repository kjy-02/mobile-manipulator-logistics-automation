#!/usr/bin/env python3
import math
import unittest
import rospy
import rostest
from sensor_msgs.msg import LaserScan
from test_planner_lifecycle import LifecycleTest

class ThinScanLimit(LifecycleTest):
    def test_yaw_finish_and_upstream_goal_replacement(self):
        scans=[]
        sub=rospy.Subscriber('/scout_dwa/scan',LaserScan,scans.append)
        increment=math.pi/180
        def point(bin_number):
            angle=-math.pi+(bin_number+.5)*increment
            return (.1*math.cos(angle),.1*math.sin(angle),.2)
        raw,_,_=self.pump(obstacle=point(0),duration=1.5)
        self.assertTrue(raw)
        self.assertTrue(all(abs(m.linear.x)<1e-9 and abs(m.angular.z)<1e-9 for m in raw),
                        'sampled obstacle did not block all candidates')
        raw,safe,_=self.pump(obstacle=point(1),duration=1.5)
        self.assertTrue(scans)
        self.assertTrue(math.isfinite(scans[-1].ranges[1]),'adapter dropped point')
        self.assertTrue(any(m.linear.x>0 for m in raw),'expected pinned subsampling limitation not reproduced')
        self.assertTrue(safe)
        self.assertTrue(all(abs(m.linear.x)<1e-9 and abs(m.angular.z)<1e-9 for m in safe),
                        'current footprint occupancy veto failed')
        rospy.logwarn('KNOWN LIMITATION: bin 1 retained by adapter but skipped by upstream step 4. Current footprint guard vetoes safe output; future-path sampling limitation remains.')
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('thin_scan_limit_test')
    rostest.rosrun('scout_dwa','thin_scan_limit',ThinScanLimit)
