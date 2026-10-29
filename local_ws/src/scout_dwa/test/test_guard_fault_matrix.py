#!/usr/bin/env python3
import time
import unittest
import rospy
import rostest
import tf
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import SetBool
from scout_dwa.msg import IntegrationHealth

def components(t):
    return (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)

class GuardMatrix(unittest.TestCase):
    def setUp(self):
        self.received=[]; self.reasons=[]
        self.subs=[rospy.Subscriber('/scout_dwa/cmd_vel_safe',Twist,lambda m:self.received.append((time.monotonic(),m))),
                   rospy.Subscriber('/cmd_vel_guard/reason',String,lambda m:self.reasons.append(m.data))]
        self.raw=rospy.Publisher('/scout_dwa/cmd_vel_raw',Twist,queue_size=1)
        self.odom=rospy.Publisher('/scout/odom',Odometry,queue_size=1)
        self.grid=rospy.Publisher('/scout_dwa/local_map',OccupancyGrid,queue_size=1)
        self.footprint=rospy.Publisher('/scout_dwa/footprint',PolygonStamped,queue_size=1)
        self.health={k:rospy.Publisher('/scout_dwa/'+k+'_health',IntegrationHealth,queue_size=1) for k in ('input','sensor','localization','config')}
        self.broadcaster=tf.TransformBroadcaster()
        rospy.wait_for_service('/cmd_vel_guard/set_enabled',timeout=3)
        self.enable=rospy.ServiceProxy('/cmd_vel_guard/set_enabled',SetBool)
        time.sleep(.2)
    def pump(self, fault=None, duration=.65, rotation=False):
        start=time.monotonic(); end=start+duration
        while time.monotonic()<end:
            stamp=rospy.Time.now()
            cmd=Twist(); cmd.linear.x=0 if rotation else .2; cmd.angular.z=.3
            if fault=='nan': cmd.linear.y=float('nan')
            if fault=='limit': cmd.linear.x=1
            if fault!='raw': self.raw.publish(cmd)
            od=Odometry(); od.header.stamp=stamp; od.header.frame_id='odom'; od.child_frame_id='base_link'
            od.pose.pose.orientation.w=1
            if fault=='odom_future': od.header.stamp+=rospy.Duration(2)
            if fault=='odom_frame': od.child_frame_id='wrong'
            if fault!='odom': self.odom.publish(od)
            grid=OccupancyGrid(); grid.header.stamp=stamp; grid.header.frame_id='base_link'
            grid.info.resolution=.1; grid.info.width=1; grid.info.height=1; grid.info.origin.orientation.w=1
            grid.data=[100 if fault=='near' else 0]
            if fault=='map_future': grid.header.stamp+=rospy.Duration(2)
            if fault=='map_bad': grid.data=[]
            if fault=='map_frame': grid.header.frame_id='wrong'
            if fault!='obstacle': self.grid.publish(grid)
            polygon=PolygonStamped(); polygon.header.frame_id='base_link'
            polygon.polygon.points=[Point32(x=x,y=y,z=0) for x,y in ((-.3,-.2),(.3,-.2),(.3,.2),(-.3,.2))]
            if fault=='polygon_nan': polygon.polygon.points[0].x=float('nan')
            if fault=='polygon_cross': polygon.polygon.points=[polygon.polygon.points[i] for i in (0,2,1,3)]
            self.footprint.publish(polygon)
            if fault!='tf':
                self.broadcaster.sendTransform((0,0,0),(0,0,0,1),stamp,'base_link','map')
            for key,pub in self.health.items():
                if fault==key+'_stop': continue
                h=IntegrationHealth(); h.header.stamp=stamp; h.valid=fault!=key+'_false'
                if key=='sensor' and fault=='sensor_mismatch': h.header.stamp-=rospy.Duration(.05)
                pub.publish(h)
            time.sleep(.025)
        return [m for t,m in self.received if t>start+duration-.12]
    def assert_pass(self):
        samples=self.pump()
        self.assertTrue(samples)
        self.assertTrue(any(components(m)==(.2,0,0,0,0,.3) for m in samples),self.reasons[-5:])
    def test_independent_faults_and_recovery(self):
        self.assert_pass()
        for fault in ('raw','odom','obstacle','tf','localization_stop','sensor_stop','input_stop','config_stop',
                      'localization_false','sensor_false','input_false','config_false','sensor_mismatch',
                      'odom_future','map_future','odom_frame','map_frame','map_bad','nan','limit',
                      'polygon_nan','polygon_cross'):
            with self.subTest(fault=fault):
                samples=self.pump(fault)
                self.assertTrue(samples)
                self.assertTrue(all(components(m)==(0,0,0,0,0,0) for m in samples),fault)
                self.assert_pass()
        samples=self.pump('near',rotation=True)
        self.assertTrue(all(components(m)==(0,0,0,0,0,0) for m in samples))
        self.assertTrue(any(reason in self.reasons for reason in
                            ('rotation clearance occupied','current footprint circumscribed area occupied')))
        self.enable(False)
        samples=self.pump()
        self.assertTrue(all(components(m)==(0,0,0,0,0,0) for m in samples))
        self.enable(True)
        self.assert_pass()

if __name__=='__main__':
    rospy.init_node('guard_matrix_test')
    rostest.rosrun('scout_dwa','guard_fault_matrix',GuardMatrix)
