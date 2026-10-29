#!/usr/bin/env python3
"""All initialization requests stop output first; resume requires a separate instruction."""
import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool, Trigger, TriggerResponse, Empty
from std_msgs.msg import String

class LocalizationSession:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('~initial_pose', PoseWithCovarianceStamped, self.initial_pose)
        rospy.Service('~relocalize', Trigger, self.relocalize)
        rospy.Service('~allow_quality_recheck', Trigger, self.recheck)
        self.reason_pub = rospy.Publisher('~reason', String, queue_size=1, latch=True)
    def inhibit(self):
        rospy.wait_for_service('/cmd_vel_guard/set_enabled', timeout=2)
        rospy.ServiceProxy('/cmd_vel_guard/set_enabled', SetBool)(False)
        rospy.wait_for_service('/localization_monitor/set_inhibit', timeout=2)
        rospy.ServiceProxy('/localization_monitor/set_inhibit', SetBool)(True)
        rospy.wait_for_service('/input_gateway/check_stopped', timeout=2)
        stopped = rospy.ServiceProxy('/input_gateway/check_stopped', Trigger)()
        if not stopped.success:
            raise rospy.ServiceException(stopped.message)
    def initial_pose(self, msg):
        try:
            self.inhibit()
            p = msg.pose.pose
            values = (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
            if msg.header.frame_id != rospy.get_param('~global_frame','map') or not all(math.isfinite(v) for v in values+tuple(msg.pose.covariance)) or abs(sum(v*v for v in values[3:])-1)>1e-3:
                raise rospy.ServiceException('invalid initial pose')
            self.pose_pub.publish(msg)
            self.reason_pub.publish('initial pose sent; quality recheck and explicit resume required')
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.reason_pub.publish(str(exc))
    def relocalize(self, _):
        try:
            self.inhibit()
            service = rospy.get_param('~hdl_relocalize_service','/relocalize')
            rospy.wait_for_service(service,timeout=2)
            rospy.ServiceProxy(service, Empty)()
            return TriggerResponse(success=True,message='relocalization requested; inhibited until explicit quality recheck')
        except (rospy.ROSException,rospy.ServiceException) as exc:
            return TriggerResponse(success=False,message=str(exc))
    def recheck(self, _):
        try:
            rospy.wait_for_service('/cmd_vel_guard/set_enabled',timeout=2)
            rospy.ServiceProxy('/cmd_vel_guard/set_enabled',SetBool)(False)
            rospy.wait_for_service('/localization_monitor/set_inhibit',timeout=2)
            result=rospy.ServiceProxy('/localization_monitor/set_inhibit',SetBool)(False)
            return TriggerResponse(success=result.success,message='fresh consecutive status required; guard stays disabled')
        except (rospy.ROSException,rospy.ServiceException) as exc:
            return TriggerResponse(success=False,message=str(exc))

if __name__=='__main__':
    rospy.init_node('localization_session')
    LocalizationSession()
    rospy.spin()
