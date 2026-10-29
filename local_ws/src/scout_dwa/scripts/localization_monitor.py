#!/usr/bin/env python3
import math
import time
import threading
import rospy
import numpy as np
import tf
from hdl_localization.msg import ScanMatchingStatus
from scout_dwa.msg import IntegrationHealth
from std_srvs.srv import SetBool, SetBoolResponse

class Monitor:
    def __init__(self):
        self.lock = threading.RLock()
        self.require_quality = rospy.get_param("~require_quality_check", True)
        self.max_error = rospy.get_param("~max_matching_error", None)
        self.min_inlier = rospy.get_param("~min_inlier_fraction", None)
        self.required = int(rospy.get_param("~required_consecutive", 3)); self.good = 0
        self.timeout = float(rospy.get_param("~status_timeout", 0.3))
        self.configured = self.required > 0 and math.isfinite(self.timeout) and self.timeout > 0
        if self.require_quality:
            self.configured = (self.configured and isinstance(self.max_error, (int, float)) and
                               math.isfinite(self.max_error) and self.max_error > 0 and
                               isinstance(self.min_inlier, (int, float)) and
                               math.isfinite(self.min_inlier) and 0 <= self.min_inlier <= 1)
        self.require_jump = rospy.get_param("~require_pose_jump_check", False)
        self.max_translation = rospy.get_param("~max_correction_translation", None)
        self.max_rotation = rospy.get_param("~max_correction_rotation", None)
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        if self.require_jump:
            self.configured = self.configured and all(isinstance(v, (int,float)) and math.isfinite(v) and v>0
                                                     for v in (self.max_translation,self.max_rotation))
        self.listener = tf.TransformListener() if self.require_jump else None
        self.previous_correction = None
        self.jump_latched = False
        self.inhibited = False
        self.last_stamp = rospy.Time()
        self.last_wall = 0.0
        self.latest = IntegrationHealth()
        self.pub = rospy.Publisher("/scout_dwa/localization_health", IntegrationHealth, queue_size=1)
        rospy.Subscriber(rospy.get_param("~status_topic", "/localization/status"), ScanMatchingStatus, self.cb)
        rospy.Service("~set_inhibit", SetBool, self.set_inhibit)
        rospy.Timer(rospy.Duration(0.05), self.tick)
    def set_inhibit(self, request):
        with self.lock:
            self.inhibited = request.data
            self.good = 0
            if not request.data:
                self.jump_latched = False
                self.previous_correction = None
            self.latest.valid = False
            self.latest.reason = "localization session inhibited" if request.data else "awaiting new consecutive status"
            self.pub.publish(self.latest)
            return SetBoolResponse(success=True, message=self.latest.reason)
    def cb(self, status):
        with self.lock:
            age = (rospy.Time.now()-status.header.stamp).to_sec()
            ok = (self.configured and not self.inhibited and not status.header.stamp.is_zero() and
                  0 <= age <= self.timeout and status.header.stamp > self.last_stamp)
            if ok and self.require_quality:
                ok = (status.has_converged and math.isfinite(status.matching_error) and
                      math.isfinite(status.inlier_fraction) and
                      0 <= status.matching_error <= self.max_error and
                      self.min_inlier <= status.inlier_fraction <= 1)
            if ok and self.require_jump:
                try:
                    # Status and TF travel on separate connections. Allow the
                    # matching stamped TF a bounded time to reach this listener.
                    self.listener.waitForTransform(self.global_frame, self.odom_frame,
                                                   status.header.stamp, rospy.Duration(0.1))
                    translation, quaternion = self.listener.lookupTransform(self.global_frame,self.odom_frame,status.header.stamp)
                    correction = tf.transformations.quaternion_matrix(quaternion)
                    correction[:3,3] = translation
                    if not np.isfinite(correction).all():
                        ok = False
                    elif self.previous_correction is not None:
                        delta = np.linalg.inv(self.previous_correction).dot(correction)
                        distance = float(np.linalg.norm(delta[:3,3]))
                        rotation = math.acos(max(-1,min(1,(float(np.trace(delta[:3,:3]))-1)/2)))
                        if distance > self.max_translation or rotation > self.max_rotation:
                            self.jump_latched = True
                    self.previous_correction = correction
                except tf.Exception:
                    ok = False
                ok = ok and not self.jump_latched
            if time.monotonic()-self.last_wall > self.timeout:
                self.good = 0
            self.good = self.good + 1 if ok else 0
            self.last_stamp = status.header.stamp
            self.last_wall = time.monotonic()
            msg = IntegrationHealth(); msg.header = status.header
            msg.valid = ok and self.good >= self.required
            msg.reason = "localization valid" if msg.valid else ("map->odom correction jump; explicit session recheck required" if self.jump_latched else "localization inhibited, stale, unconfigured or quality rejected")
            self.latest = msg
            self.pub.publish(msg)
    def tick(self, _):
        with self.lock:
            if self.inhibited or time.monotonic()-self.last_wall > self.timeout:
                self.good = 0
                self.latest.valid = False
                self.latest.reason = "localization session inhibited" if self.inhibited else "localization status timeout"
                self.pub.publish(self.latest)
if __name__ == "__main__":
    rospy.init_node("localization_monitor"); Monitor(); rospy.spin()
