#!/usr/bin/env python3
"""Validate each channel independently and defer velocity reductions until stopped."""
import math
import threading
import time
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from scout_dwa.msg import IntegrationHealth
from std_srvs.srv import Trigger, TriggerResponse

class Gateway:
    def __init__(self):
        self.lock = threading.RLock()
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.min_velocity = float(rospy.get_param('~min_velocity', rospy.get_param('~MIN_VELOCITY', 0.0)))
        self.max_velocity = float(rospy.get_param('~max_velocity', rospy.get_param('~MAX_VELOCITY', 0.5)))
        self.max_threshold = float(rospy.get_param('~max_goal_distance_threshold', 2.0))
        self.odom_timeout = float(rospy.get_param('~odom_timeout', 0.3))
        self.stop_tolerance = float(rospy.get_param('~stop_tolerance', 0.01))
        self.target = float(rospy.get_param('~initial_target_velocity', rospy.get_param('~TARGET_VELOCITY', 0.3)))
        self.pending = None
        self.pending_goal = None
        self.active_goal = None
        self.odom = None
        self.odom_wall = 0
        self.valid = dict(goal=False, path=True, velocity=True, distance=True)
        self.reason = 'waiting for valid goal'
        self.listener = tf.TransformListener()
        self.health_pub = rospy.Publisher('/scout_dwa/input_health', IntegrationHealth, queue_size=1)
        self.goal_pub = rospy.Publisher('/scout_dwa/validated_goal', PoseStamped, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/scout_dwa/validated_path', Path, queue_size=1, latch=True)
        self.velocity_pub = rospy.Publisher('/scout_dwa/validated_target_velocity', Twist, queue_size=1, latch=True)
        self.distance_pub = rospy.Publisher('/scout_dwa/validated_dist_to_goal', Float64, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param('~goal_input', '/move_base_simple/goal'), PoseStamped, self.goal)
        rospy.Subscriber(rospy.get_param('~path_input', '/scout_dwa/path_segment'), Path, self.path)
        rospy.Subscriber(rospy.get_param('~velocity_input', '/target_velocity'), Twist, self.velocity)
        rospy.Subscriber(rospy.get_param('~distance_input', '/dist_to_goal_th'), Float64, self.distance)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/scout/odom'), Odometry, self.odometry)
        rospy.Service('~commit_new_session', Trigger, self.commit_session)
        rospy.Service('~check_stopped', Trigger, lambda _: TriggerResponse(
            success=self.stopped(), message='fresh stopped odometry required'))
        rospy.Timer(rospy.Duration(0.05), self.tick)

    @staticmethod
    def finite_pose(p):
        values = (p.position.x, p.position.y, p.position.z, p.orientation.x,
                  p.orientation.y, p.orientation.z, p.orientation.w)
        return all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:])-1) < 1e-3

    def result(self, channel, valid, reason):
        self.valid[channel] = valid
        self.reason = reason

    def goal(self, msg):
        with self.lock:
            self.result('goal', False, 'invalid goal or quaternion')
            if not msg.header.frame_id or not self.finite_pose(msg.pose):
                return
            try:
                out = self.listener.transformPose(self.global_frame, msg)
            except tf.Exception as exc:
                self.reason = 'goal TF: %s' % exc
                return
            if self.active_goal is not None and out.pose != self.active_goal.pose:
                self.pending_goal = out
                self.reason = 'new goal requires stopped planner restart session'
                return
            self.goal_pub.publish(out)
            self.result('goal', True, 'goal accepted')
            self.active_goal = out

    def path(self, msg):
        with self.lock:
            self.result('path', False, 'invalid path frame, quaternion or geometry')
            if len(msg.poses) < 2 or not msg.header.frame_id:
                return
            if any(p.header.frame_id != msg.header.frame_id or not self.finite_pose(p.pose) for p in msg.poses):
                return
            a, b = msg.poses[0].pose.position, msg.poses[-1].pose.position
            if math.hypot(a.x-b.x, a.y-b.y) < 1e-6:
                return
            out = Path(); out.header = msg.header; out.header.frame_id = self.global_frame
            try:
                out.poses = [self.listener.transformPose(self.global_frame, p) for p in msg.poses]
            except tf.Exception as exc:
                self.reason = 'path TF: %s' % exc
                return
            self.path_pub.publish(out)
            self.result('path', True, 'path accepted')

    def velocity(self, msg):
        with self.lock:
            components = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
            if not all(math.isfinite(v) for v in components) or any(abs(v)>1e-9 for v in components[1:]) or not self.min_velocity <= msg.linear.x <= self.max_velocity:
                self.pending = None
                self.result('velocity', False, 'target velocity out of range')
                return
            if msg.linear.x < self.target:
                self.pending = msg
                self.result('velocity', False, 'downshift waiting for fresh stopped odometry')
            else:
                self.pending = None
                self.target = msg.linear.x
                self.velocity_pub.publish(msg)
                self.result('velocity', True, 'target velocity accepted')

    def distance(self, msg):
        with self.lock:
            valid = math.isfinite(msg.data) and 0 < msg.data <= self.max_threshold
            self.result('distance', valid, 'threshold accepted' if valid else 'threshold out of range')
            if valid:
                self.distance_pub.publish(msg)

    def odometry(self, msg):
        with self.lock:
            self.odom = msg
            self.odom_wall = time.monotonic()

    def stopped(self):
        if self.odom is None:
            return False
        age = (rospy.Time.now()-self.odom.header.stamp).to_sec()
        v, w = self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z
        return (self.odom.child_frame_id == self.robot_frame and not self.odom.header.stamp.is_zero() and
                0 <= age <= self.odom_timeout and time.monotonic()-self.odom_wall <= self.odom_timeout and
                math.isfinite(v) and math.isfinite(w) and abs(v)<=self.stop_tolerance and abs(w)<=self.stop_tolerance)

    def commit_session(self, _):
        with self.lock:
            if not self.stopped():
                return TriggerResponse(success=False, message='fresh stopped odometry required; guard remains disabled')
            if self.pending_goal is not None:
                self.active_goal = self.pending_goal
                self.goal_pub.publish(self.pending_goal)
                self.pending_goal = None
                self.result('goal', True, 'new goal session committed')
            return TriggerResponse(success=True, message='latched valid goal ready for new planner process')

    def tick(self, _):
        with self.lock:
            now = rospy.Time.now()
            if self.pending is not None:
                if self.stopped():
                    self.target = self.pending.linear.x
                    self.velocity_pub.publish(self.pending)
                    self.pending = None
                    self.result('velocity', True, 'stopped downshift accepted')
            health = IntegrationHealth()
            health.header.stamp = now
            health.valid = all(self.valid.values()) and self.pending is None and self.pending_goal is None
            health.reason = 'inputs valid' if health.valid else 'invalid channels: '+','.join(k for k,v in self.valid.items() if not v)+'; '+self.reason
            self.health_pub.publish(health)

if __name__ == '__main__':
    rospy.init_node('input_gateway')
    Gateway()
    rospy.spin()
