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

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
    """docstring for Edrone"""

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z,yaw_value]
        self.drone_position = [0.0, 0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

        # previous value - [5.68, -1.91, 33.40, 0]

        self.initial_waypoint = [5.68, -1.91, 33.40, 0]

        # initially setpoint and destination are both initial_waypoint
        self.destination = [5.68, -1.91, 33.40, 0]
        self.setpoint = [5.68, -1.91, 33.40, 0]

        # counter to loop thrrough the points in the path
        self.count = 0

        # Declaring a cmd of message type PlutoMsg and initializing values
        self.cmd = PlutoMsg()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500
        # self.cmd.plutoIndex = 0

        # initial setting of Kp, Kd and Ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [10.15, 10.205, 6.85, 6.25]
        self.Ki = [0, 0, 0, 9.6]
        self.Kd = [17.63, 16.03, 33.01, 18.80]

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
        self.max_values = [75, 75, 75, 180]
        self.min_values = [-75, -75, -75, -179]
        self.path = PoseArray()

        self.path_retrieved = False  # bool value to check if new path has been received
        # to keep a track of the goals reached to disarm thre drone when waypoint is reached
        self.goals_reached = 0

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
        #		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
        #													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
        #																	You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.1  # in seconds

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
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)

        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/drone_yaw', Float64, self.drone_yaw_callback)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/vrep/waypoints', PoseArray, self.update)
        # ------------------------------------------------------------------------------------------------------------

        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.

    def arm(self):

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

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

    def altitude_set_pid(self, alt):
        # This is just for an example. You can change the fraction value accordingly
        self.Kp[2] = alt.Kp * 0.001
        self.Ki[2] = alt.Ki * 0.001
        self.Kd[2] = alt.Kd * 0.01

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------
    def drone_yaw_callback(self, msg):
        self.drone_position[3] = msg.data

    def pitch_set_pid(self, pit):
        self.Kp[0] = pit.Kp * 0.001
        self.Ki[0] = pit.Ki * 0.001
        self.Kd[0] = pit.Kd * 0.01

    def roll_set_pid(self, roll):
        self.Kp[1] = roll.Kp * 0.001
        self.Ki[1] = roll.Ki * 0.001
        self.Kd[1] = roll.Kd * 0.01

    def yaw_set_pid(self, yaw):
        self.Kp[3] = yaw.Kp * 0.001
        self.Ki[3] = yaw.Ki * 0.001
        self.Kd[3] = yaw.Kd * 0.01

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
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
        #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
        #	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        #																														self.cmd.rcPitch = self.max_values[1]
        #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #	8. Add error_sum

        # ------------------------------------------------------------------------------------------------------------------------
        self.now = time.time()
        self.time_change = self.now - self.last_time
        if(self.time_change >= self.sample_time):
            for i in range(4):
                self.error[i] = self.setpoint[i] - self.drone_position[i]

                self.error_sum[i] += self.error[i]*self.time_change
                if (self.error_sum[i] > self.max_values[i]):
                    self.error_sum[i] = self.max_values[i]
                elif (self.error_sum[i] < self.min_values[i]):
                    self.error_sum[i] = self.min_values[i]

                self.d_error[i] = (
                    self.error[i]-self.last_error[i])/self.time_change

                self.output[i] = self.Kp[i]*self.error[i] + \
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

            if (all(-1.0 < i < 1.0 for i in self.dis)):  # if goal is reached
                self.goals_reached += 1
                if (self.goals_reached == 4):
                    self.disarm()  # back to initial_waypoint
                # resetting destination until the new path is received and destination is updated
                self.destination = [0, 0, 0, 0]
                self.path_req_pub.publish(True)  # requesting new path
                # setting to false so that program waits at the same setpoint until new path is received
                self.path_retrieved = False
            else:
                self.path_req_pub.publish(False)

            if (all(-1.0 < i < 1.0 for i in self.dis_set) and self.path_retrieved == True):
                if(self.count < 39):  # to prevent counter from going out of index while waiting for a new path
                    self.count += 1

                # looping through the points in the path
                self.setpoint = [self.path.poses[self.count].position.x,
                                 self.path.poses[self.count].position.y, self.path.poses[self.count].position.z, 0]

            # publishing zero line, errors and the drone command
            self.zero_line_pub.publish(0.0)
            self.pitch_error_pub.publish(self.error[0])
            self.roll_error_pub.publish(self.error[1])
            self.alt_error_pub.publish(self.error[2])
            self.yaw_error_pub.publish(self.error[3])
            self.command_pub.publish(self.cmd)


if __name__ == '__main__':

    e_drone = Edrone()

    while not rospy.is_shutdown():
        e_drone.pid()
