#!/usr/bin/env python3
"""Adjust copied markers for display; never publish control commands."""
import copy
import math
import rospy
from visualization_msgs.msg import Marker, MarkerArray


def display_marker(source, selected):
    marker = copy.deepcopy(source)
    if marker.action == Marker.ADD:
        if len(marker.points) < 2 or not all(
                math.isfinite(v) for p in marker.points for v in (p.x, p.y, p.z)):
            marker.action = Marker.DELETE
        else:
            for point in marker.points:
                point.z += 1.0
            marker.scale.x = 0.07 if selected else 0.02
            marker.color.a = 1.0 if selected else 0.45
            marker.lifetime = rospy.Duration(0.3)
    return marker


if __name__ == '__main__':
    rospy.init_node('trajectory_display')
    selected_pub = rospy.Publisher('/scout_dwa/display/selected_trajectory', Marker, queue_size=1)
    candidates_pub = rospy.Publisher('/scout_dwa/display/candidate_trajectories', MarkerArray, queue_size=1)
    rospy.Subscriber('/scout_dwa/dwa_planner/selected_trajectory', Marker,
                     lambda msg: selected_pub.publish(display_marker(msg, True)), queue_size=1)
    rospy.Subscriber('/scout_dwa/dwa_planner/candidate_trajectories', MarkerArray,
                     lambda msg: candidates_pub.publish(MarkerArray(
                         markers=[display_marker(m, False) for m in msg.markers])), queue_size=1)
    rospy.spin()
