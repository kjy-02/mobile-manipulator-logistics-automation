#!/usr/bin/env python3
import time
import unittest
import rospy
import rostest
import tf
from std_srvs.srv import SetBool
from hdl_localization.msg import ScanMatchingStatus
from scout_dwa.msg import IntegrationHealth

class JumpTest(unittest.TestCase):
    def test_map_odom_correction_and_latched_recovery(self):
        health=[]
        sub=rospy.Subscriber('/scout_dwa/localization_health',IntegrationHealth,health.append)
        pub=rospy.Publisher('/localization/status',ScanMatchingStatus,queue_size=1)
        broadcaster=tf.TransformBroadcaster()
        rospy.wait_for_service('/localization_monitor/set_inhibit',timeout=3)
        recheck=rospy.ServiceProxy('/localization_monitor/set_inhibit',SetBool)
        def pump(correction=0,wheel=0,yaw=0):
            for _ in range(10):
                stamp=rospy.Time.now()
                broadcaster.sendTransform((correction,0,0),tf.transformations.quaternion_from_euler(0,0,yaw),stamp,'odom','map')
                broadcaster.sendTransform((wheel,0,0),(0,0,0,1),stamp,'base_link','odom')
                time.sleep(.02)
                msg=ScanMatchingStatus(); msg.header.stamp=stamp; msg.has_converged=True
                msg.matching_error=.1; msg.inlier_fraction=.9; pub.publish(msg)
                time.sleep(.03)
        pump(); self.assertTrue(health[-1].valid)
        pump(wheel=1); self.assertTrue(health[-1].valid,'wheel motion tripped map correction check')
        pump(correction=1,wheel=1); self.assertFalse(health[-1].valid)
        pump(correction=1,wheel=1); self.assertFalse(health[-1].valid,'jump latch cleared by good frames')
        recheck(False); pump(correction=1,wheel=1); self.assertTrue(health[-1].valid)
        pump(correction=1,wheel=1,yaw=.5); self.assertFalse(health[-1].valid)
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('localization_jump_test')
    rostest.rosrun('scout_dwa','localization_jump',JumpTest)
