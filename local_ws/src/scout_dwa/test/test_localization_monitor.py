#!/usr/bin/env python3
import unittest
import rospy
import rostest
import time
from std_srvs.srv import SetBool
from hdl_localization.msg import ScanMatchingStatus
from scout_dwa.msg import IntegrationHealth

class LocalizationMonitorTest(unittest.TestCase):
    def test_quality_and_consecutive_gate(self):
        messages=[]
        pub=rospy.Publisher('/localization/status',ScanMatchingStatus,queue_size=1)
        sub=rospy.Subscriber('/scout_dwa/localization_health',IntegrationHealth,messages.append)
        rate=rospy.Rate(20)
        deadline=rospy.Time.now()+rospy.Duration(3.0)
        while pub.get_num_connections()==0 and rospy.Time.now()<deadline: rate.sleep()
        self.assertGreater(pub.get_num_connections(),0)
        for converged,error,inlier,count in ((False,.1,.9,4),(True,.1,.9,2),(True,.1,.9,4),(True,2.0,.9,4)):
            for _ in range(count):
                msg=ScanMatchingStatus(); msg.header.stamp=rospy.Time.now(); msg.has_converged=converged; msg.matching_error=error; msg.inlier_fraction=inlier; pub.publish(msg); rate.sleep()
            rospy.sleep(.1)
            self.assertTrue(messages)
            expected=converged and error<=1.0 and inlier>=.5 and count>=3
            self.assertEqual(messages[-1].valid,expected)
        sub.unregister()

    def test_timeout_stamp_nan_and_session(self):
        messages=[]
        pub=rospy.Publisher('/localization/status',ScanMatchingStatus,queue_size=1)
        sub=rospy.Subscriber('/scout_dwa/localization_health',IntegrationHealth,messages.append)
        rospy.wait_for_service('/localization_monitor/set_inhibit',timeout=3)
        inhibit=rospy.ServiceProxy('/localization_monitor/set_inhibit',SetBool)
        def send(count=5,offset=0,error=.1,inlier=.9):
            for _ in range(count):
                msg=ScanMatchingStatus(); msg.header.stamp=rospy.Time.now()+rospy.Duration(offset)
                msg.has_converged=True; msg.matching_error=error; msg.inlier_fraction=inlier
                pub.publish(msg); time.sleep(.05)
        time.sleep(.1)
        send(); self.assertTrue(messages[-1].valid)
        time.sleep(.45); self.assertFalse(messages[-1].valid)
        send(count=1); self.assertFalse(messages[-1].valid,'single frame restored after outage')
        send(); self.assertTrue(messages[-1].valid)
        send(offset=2); self.assertFalse(messages[-1].valid)
        send(offset=-2); self.assertFalse(messages[-1].valid)
        send(error=float('nan')); self.assertFalse(messages[-1].valid)
        send(inlier=float('nan')); self.assertFalse(messages[-1].valid)
        inhibit(True); send(); self.assertFalse(messages[-1].valid)
        inhibit(False); send(count=1); self.assertFalse(messages[-1].valid)
        send(); self.assertTrue(messages[-1].valid)
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('localization_monitor_test'); rostest.rosrun('scout_dwa','localization_monitor_test',LocalizationMonitorTest)
