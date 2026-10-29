#!/usr/bin/env python3
import os
import signal
import time
import xmlrpc.client
import rosgraph
import rospy
import rostest
from geometry_msgs.msg import Twist
from test_guard_fault_matrix import GuardMatrix, components

class GuardDeath(GuardMatrix):
    def test_independent_faults_and_recovery(self):
        downstream=[]
        sub=rospy.Subscriber('/test/downstream_cmd',Twist,lambda m:downstream.append((time.monotonic(),m)))
        self.assert_pass()
        time.sleep(.05)
        self.assertTrue(any(m.linear.x==.2 for _,m in downstream),'downstream never received live motion')
        uri=rosgraph.Master(rospy.get_name()).lookupNode('/cmd_vel_guard')
        code,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        self.assertEqual(code,1); self.assertGreater(pid,1)
        started=time.monotonic()
        os.kill(pid,signal.SIGKILL if rospy.get_param('~sigkill',True) else signal.SIGTERM)
        self.pump(duration=.7)
        samples=[m for t,m in downstream if t>started+.4]
        self.assertTrue(samples)
        self.assertTrue(all(components(m)==(0,0,0,0,0,0) for m in samples))
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('guard_death_test')
    rostest.rosrun('scout_dwa','guard_death',GuardDeath)
