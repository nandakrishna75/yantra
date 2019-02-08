#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from plutodrone.srv import *
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Plutodrone():
    """docstring for Plutodrone"""

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # [x,y,z,yaw_value]
        self.drone_position = [0.0, 0.0, 0.0, 0.0]

        self.initial_waypoint = [-4.85, -1.36, 25.0, 280]

        # initially setpoint and destination are both initial_waypoint
        self.destination = [-4.85, -1.36, 25.0, 280]
        self.setpoint = [-4.85, -1.36, 25.0, 280]

        # counter to loop thrrough the points in the path
        self.count = 0

        # Declaring a cmd of message type PlutoMsg and initializing values
        self.cmd = PlutoMsg()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        # self.cmd.plutoIndex = 0

        self.Kp = [10, 10, 50, 10]
        self.Ki = [1, 1, 4, 0]
        self.Kd = [20, 20, 10, 5]

        # -----------------------Add other required variables for pid here ----------------------------------------------

        self.dis = [0, 0, 0, 0]  # distance between drone and destination
        self.dis_set = [0, 0, 0, 0]  # distance between drone and set point

        # PID variables
        self.error = [0, 0, 0, 0]
        self.d_error = [0, 0, 0, 0]
        self.last_time = time.time()

        self.output = [0, 0, 0, 0]
        self.error_sum = [0, 0, 0, 0]
        self.last_error = [0, 0, 0, 0]
        self.max_values = [175, 175, 300, 90]
        self.min_values = [-175, -175, -300, -90]
        self.path = PoseArray()

        self.path_retrieved = False  # bool value to check if new path has been received
        # to keep a track of the goals reached to disarm thre drone when waypoint is reached
        self.goals_reached = 0

        self.sample_time = 0.05  # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
        self.command_pub = rospy.Publisher(
            '/drone_command', PlutoMsg, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.alt_error_pub = rospy.Publisher(
            '/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)
        self.yaw_error_pub = rospy.Publisher(
            '/yaw_error', Float64, queue_size=1)
        self.zero_line_pub = rospy.Publisher(
            '/zero_line', Float64, queue_size=1)
        self.path_req_pub = rospy.Publisher(
            '/path_plan', Bool, queue_size=10)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/drone_yaw', Int32, self.drone_yaw_callback)

        rospy.Subscriber('/vrep/waypoints', PoseArray, self.update)
        # ------------------------------------------------------------------------------------------------------------

        self.arm()

    # Disarming condition of the drone

    def disarm(self):
        self.cmd.rcThrottle = 1300
        self.cmd.rcAUX4 = 1200
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone

    def arm(self):
        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def decrease_height(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1400
        self.command_pub.publish(self.cmd)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------
    def drone_yaw_callback(self, msg):
        self.drone_position[3] = msg.data

    def update(self, msg):
        # setting counter to 0 to traverse the points
        self.count = 0

        # setting destination to the last point of the path
        self.destination = [msg.poses[39].position.x,
                            msg.poses[39].position.y, msg.poses[39].position.z, 0]

        # setting set point to the first point of the path
        self.setpoint = [msg.poses[self.count].position.x,
                         msg.poses[self.count].position.y, msg.poses[self.count].position.z, 0]

        # storing the pose array to loop the points of the path
        self.path = msg

        # bool value confirm that path has been received
        self.path_retrieved = True

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        self.now = time.time()
        self.time_change = self.now - self.last_time
        if(self.time_change >= self.sample_time):
            for i in range(4):
                self.error[i] = self.setpoint[i] - self.drone_position[i]

                # if self.error[3] > 180:
                #     self.error[3] = self.error[3] - 360

                self.error_sum[i] += self.error[i]*self.time_change
                if (self.error_sum[i] > self.max_values[i]):
                    self.error_sum[i] = self.max_values[i]
                elif (self.error_sum[i] < self.min_values[i]):
                    self.error_sum[i] = self.min_values[i]

                self.d_error[i] = (
                    self.error[i]-self.last_error[i])/self.time_change

                self.output[i] = self.Kp[i]*self.error[i] +\
                    self.Ki[i] * self.error_sum[i] + \
                    self.Kd[i] * self.d_error[i]

                if (self.output[i] > self.max_values[i]):
                    self.output[i] = self.max_values[i]
                elif (self.output[i] < self.min_values[i]):
                    self.output[i] = self.min_values[i]
                self.last_error[i] = self.error[i]

                # determining the error between drone and destination
                self.dis[i] = self.destination[i] - self.drone_position[i]

                # determining the error between drone and set_point
                self.dis_set[i] = self.setpoint[i] - self.drone_position[i]

            self.last_time = self.now

            self.cmd.rcPitch = 1500 - self.output[0]
            self.cmd.rcRoll = 1500 - self.output[1]
            self.cmd.rcThrottle = 1500 - self.output[2]
            self.cmd.rcYaw = 1500 + self.output[3]

            if (all(-0.75 < i < 0.75 for i in self.dis[:3])):  # if goal is reached
                self.goals_reached += 1
                if (self.goals_reached == 4): #landing the drone
                    self.decrease_height()  # back to initial_waypoint
                    rospy.sleep(1.5)
                    self.disarm()
                    # resetting destination until the new path is received and destination is updated
                self.destination = [0, 0, 0, 0]
                self.path_req_pub.publish(True)  # requesting new path
                # setting to false so that program waits at the same setpoint until new path is received
                self.path_retrieved = False
            else:
                self.path_req_pub.publish(False)

            if (all(-0.75 < i < 0.75 for i in self.dis_set[:3]) and self.path_retrieved == True):
                if(self.count < 39):  # to prevent counter from going out of index while waiting for a new path
                    self.count += 1
                    print(self.count)
                # looping through the points in the path
                self.setpoint = [self.path.poses[self.count].position.x,
                                 self.path.poses[self.count].position.y, self.path.poses[self.count].position.z, 0]
                rospy.sleep(0.005)

            # publishing zero line, errors and the drone command
            self.zero_line_pub.publish(0.0)
            self.pitch_error_pub.publish(self.error[0])
            self.roll_error_pub.publish(self.error[1])
            self.alt_error_pub.publish(self.error[2])
            self.yaw_error_pub.publish(self.error[3])
            self.command_pub.publish(self.cmd)


if __name__ == '__main__':

    drone = Plutodrone()

    while not rospy.is_shutdown():
        drone.pid()
