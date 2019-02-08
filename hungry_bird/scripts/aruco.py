#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from aruco_msgs.msg import MarkerArray


# Defining a class
class Marker_detect():

    def __init__(self):
        # initializing a ros node with name marker_detection
        rospy.init_node('convert_type', anonymous=False)

        self.aruco_marker = {}

        rospy.Subscriber('/aruco_marker_publisher/markers',
                         MarkerArray, self.aruco_data)  # Subscribing to topic
        self.command_pub = rospy.Publisher(
            '/aruco_q', Quaternion, queue_size=1)
        self.cmd = Quaternion()

    # Callback for /aruco_marker_publisher/markers
    def aruco_data(self, msg):
        for i in range(len(msg.markers)):
            orient_x = msg.markers[i].pose.pose.orientation.x
            orient_y = msg.markers[i].pose.pose.orientation.y
            orient_z = msg.markers[i].pose.pose.orientation.z
            orient_w = msg.markers[i].pose.pose.orientation.w

        self.cmd.x = orient_x
        self.cmd.y = orient_y
        self.cmd.z = orient_z
        self.cmd.w = orient_w

        self.command_pub.publish(self.cmd)


if __name__ == "__main__":

    marker = Marker_detect()

    while not rospy.is_shutdown():
        rospy.spin()
