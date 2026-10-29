#!/usr/bin/env python3
import time
import unittest
import rospy
import rostest
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse, Trigger, TriggerResponse
from hdl_localization.msg import ScanMatchingStatus
from scout_dwa.msg import IntegrationHealth

class SessionTest(unittest.TestCase):
    def test_inhibit_and_explicit_quality_recheck(self):
        calls=[]; health=[]
        guard=rospy.Service('/cmd_vel_guard/set_enabled',SetBool,lambda r:(calls.append(('enable',r.data)) or SetBoolResponse(success=True)))
        stopped=rospy.Service('/input_gateway/check_stopped',Trigger,lambda _ :TriggerResponse(success=True,message='synthetic stopped'))
        hdl=rospy.Service('/relocalize',Empty,lambda _:(calls.append(('relocalize',True)) or EmptyResponse()))
        sub=rospy.Subscriber('/scout_dwa/localization_health',IntegrationHealth,health.append)
        pub=rospy.Publisher('/localization/status',ScanMatchingStatus,queue_size=1)
        rospy.wait_for_service('/localization_session/relocalize',timeout=3)
        def pump():
            for _ in range(6):
                msg=ScanMatchingStatus(); msg.header.stamp=rospy.Time.now()
                msg.has_converged=True; msg.matching_error=.1; msg.inlier_fraction=.9
                pub.publish(msg); time.sleep(.05)
        time.sleep(.2); pump(); self.assertTrue(health[-1].valid)
        result=rospy.ServiceProxy('/localization_session/relocalize',Trigger)()
        self.assertTrue(result.success,result.message)
        self.assertEqual(calls[:2],[('enable',False),('relocalize',True)])
        pump(); self.assertFalse(health[-1].valid,'status cleared inhibit automatically')
        result=rospy.ServiceProxy('/localization_session/allow_quality_recheck',Trigger)()
        self.assertTrue(result.success,result.message)
        self.assertEqual(calls[-1],('enable',False))
        pump(); self.assertTrue(health[-1].valid)
        self.assertFalse(any(k=='enable' and v for k,v in calls),'session enabled movement')
        sub.unregister(); guard.shutdown(); stopped.shutdown(); hdl.shutdown()

if __name__=='__main__':
    rospy.init_node('localization_session_test')
    rostest.rosrun('scout_dwa','localization_session',SessionTest)
