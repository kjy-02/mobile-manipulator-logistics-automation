#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64, Header
from scout_dwa.msg import IntegrationHealth


def main():
    rospy.init_node("synthetic_inputs")
    cloud_topic = rospy.get_param("~cloud_topic", "/scout_dwa/test_points")
    odom_topic = rospy.get_param("~odom_topic", "/scout/odom")
    goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
    path_topic = rospy.get_param("~path_topic", "/scout_dwa/path_segment")
    rate_hz = float(rospy.get_param("~rate", 30.0))
    goal_x = float(rospy.get_param("~goal_x", 2.0))
    goal_y = float(rospy.get_param("~goal_y", 0.0))
    obstacle_x = float(rospy.get_param("~obstacle_x", 1.0))
    obstacle_y = float(rospy.get_param("~obstacle_y", 0.5))
    obstacle_z = float(rospy.get_param("~obstacle_z", 0.2))

    cloud_pub = rospy.Publisher(cloud_topic, point_cloud2.PointCloud2, queue_size=1)
    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
    goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1, latch=True)
    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    target_pub = rospy.Publisher("/target_velocity", Twist, queue_size=1, latch=True)
    distance_pub = rospy.Publisher("/dist_to_goal_th", Float64, queue_size=1, latch=True)
    input_health_pub = rospy.Publisher("/scout_dwa/input_health", IntegrationHealth, queue_size=1)
    localization_health_pub = rospy.Publisher("/scout_dwa/localization_health", IntegrationHealth, queue_size=1)
    broadcaster = tf.TransformBroadcaster()

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1.0

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            stamp,
            "base_link",
            "map",
        )

        header = Header(stamp=stamp, frame_id="base_link")
        cloud = point_cloud2.create_cloud_xyz32(
            header, [(obstacle_x, obstacle_y, obstacle_z)]
        )
        cloud_pub.publish(cloud)

        odom = Odometry()
        odom.header = header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        goal.header.stamp = stamp
        goal_pub.publish(goal)

        path = Path()
        path.header = Header(stamp=stamp, frame_id="map")
        start = PoseStamped()
        start.header = path.header
        start.pose.orientation.w = 1.0
        end = PoseStamped()
        end.header = path.header
        end.pose.position.x = goal_x
        end.pose.position.y = goal_y
        end.pose.orientation.w = 1.0
        path.poses = [start, end]
        path_pub.publish(path)

        target = Twist()
        target.linear.x = 0.2
        target_pub.publish(target)
        distance_pub.publish(Float64(data=0.1))
        health = IntegrationHealth()
        health.header.stamp = stamp
        health.valid = True
        health.reason = "synthetic input valid"
        input_health_pub.publish(health)
        health.reason = "synthetic localization valid"
        localization_health_pub.publish(health)
        rate.sleep()


if __name__ == "__main__":
    main()
