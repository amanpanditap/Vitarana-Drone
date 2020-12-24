#!/usr/bin/env python
# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import *
from vitarana_drone.srv import *
from std_msgs.msg import String
import numpy as np
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String
import rospy
import sys
import time
import tf
from numpy import inf


class Edrone():
    """docstring for Edrone"""

    def __init__(self):
        rospy.init_node('position_controller')

        #drone starting position
        self.drone_position = [19.0009248718, 71.9998318945, 22.16]

        # target_position to achieve
        self.target_position = [[19.0009248718, 71.9998318945, 30.16], [19.000704022, 71.999897506, 30.16], [19.0007049952, 71.999895606, 21.84599967919]]

        # edrone_cmd initial setup
        self.cmd = edrone_cmd()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.aux1 = 1500
        self.cmd.aux2 = 1500
        self.cmd.aux3 = 1500
        self.cmd.aux4 = 1500

        #variables to store the destination location after scanning qr code
        self.lat = 0.0
        self.long = 0.0
        self.alt = 0.0

        #variable for x and y co-ordinate
        self.x = 0
        self.y = 0

        #list for storing the laser scan values
        self.laser = [0, 0, 0, 0, 0]

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        self.Kp = [22000, 25000, 6900, 180]
        self.Ki = [0.003, 0.003, 0.8, 0.001]
        self.Kd = [9300000, 9300000, 1500, 10000]
        # -----------------------Add other required variables for pid here ----------------------------------------------

        #error terms for PID
        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.diff_error = [0, 0, 0]

        # max values of Pitch, Roll,Yaw, Throttle
        self.rc_roll_max = 1550
        self.rc_pitch_max = 1550
        self.rc_yaw_max = 1550
        self.rc_throttle_max = 1800

        # min values of Pitch, Roll,Yaw, Throttle
        self.rc_roll_min = 1450
        self.rc_pitch_min = 1450
        self.rc_yaw_min = 1450
        self.rc_throttle_min = 1200

        #path variable checker
        self.path = 0
        # flag
        self.flag = 0
        # index
        self.index = 0
        # nextpos tracker
        self.nextpos = [0, 0, 0]
        # set next target_position
        self.nextpos = self.target_position[self.index]
        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 40  # in seconds

        #Subscriber and service for gripper (client node)
        self.req = GripperRequest()
        rospy.Subscriber('/edrone/gripper_check', String, self.grippercheck)
        rospy.wait_for_service('/edrone/activate_gripper')
        self.gripvalue = 'False'

        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.edrone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------
        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.laser_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.throttle_set_pid)
        rospy.Subscriber('/qrcode_gps', position, self.location)

    # ---------------------------------------------------------------------------------------------------------------
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.6  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.001
        self.Kd[0] = roll.Kd * 0.8

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.6  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 60  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = yaw.Ki * 0.8
        self.Kd[2] = yaw.Kd * 1500

    def throttle_set_pid(self, alt):
        self.Kp[3] = alt.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[3] = alt.Ki * 0.001
        self.Kd[3] = alt.Kd * 2.0

    # get the location from qr code
    def location(self, gps):
        self.lat = gps.lat
        self.long = gps.long
        self.alt = gps.alt

    # get the laser value ranges
    def laser_callback(self, range):
        self.laser = range.ranges

    # convert latitude to x co-ordinate
    def lat_to_x(self):
        self.x = 110692.0702932625 * (self.drone_position[0] - 19)

    # convert longitude to y co-ordinate
    def long_to_y(self):
        self.y = -105292.0089353767 * (self.drone_position[1] - 72)

    # convert x co-ordinate to latitude
    def x_to_lat(self, x):
        return 19 + (x / 110692.0702932625)

    # convert y co-ordinate to longitude
    def y_to_long(self, y):
        return 72 - (y / 105292.0089353767)

    # method to arm the drone
    def arm(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.aux4 = 1500
        self.edrone_pub.publish(self.cmd)
        # delay time for adjusting in gazebo sync loading time
        rospy.sleep(0.5)

    # method to disarm the drone
    def disarm(self):
        self.cmd.rcThrottle = 1300
        self.cmd.aux4 = 1200
        self.edrone_pub.publish(self.cmd)
        # delay time for adjusting in gazebo sync loading time
        rospy.sleep(0.5)

    # get drone's current position
    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

    # check whether gripper can be activated or not
    def grippercheck(self, String):
        self.gripvalue = String.data

    # function to activate the gripper and attach the package
    def attachment(self):
        self.attacher =  rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

        if ((self.gripvalue == 'True') and (self.req.activate_gripper != True)) and self.path == 0:
            self.req = GripperRequest()
            rospy.loginfo("attach req send")
            self.req.activate_gripper = True
            self.attacher.call(self.req)
            return self.req.activate_gripper

        if (self.req.activate_gripper != False) and self.index == 2 and self.path == 1:
            self.req = GripperRequest()
            rospy.loginfo("gripper detached")
            self.req.activate_gripper = False
            self.attacher.call(self.req)
            return self.req.activate_gripper

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------
        if self.flag == 0:
            self.p_roll = 0
            self.p_pitch = 0
            self.p_yaw = 0
            self.i_roll = 0
            self.i_pitch = 0
            self.i_yaw = 0
            self.d_roll = 0
            self.d_pitch = 0
            self.d_yaw = 0
            self.i_alt = 0
            self.p_alt = 0
            self.d_alt = 0
            self.disarm()
            rospy.sleep(0.5)
            self.arm()
            rospy.sleep(0.5)
            self.flag += 1

        # call to activate the gripper
        self.attachment()

        # initialize destination location by confirming first whether package attached or not
        if self.req.activate_gripper == True and self.index == 3:
            self.target_position = [[self.lat, self.long, 30.16], [self.lat, self.long, self.alt - 0.1], [self.lat, self.long, self.alt - 1]]
            self.disarm()
            self.path = 1
            rospy.sleep(0.5)
            self.arm()
            rospy.sleep(0.5)
            self.index = 0
            self.nextpos = self.target_position[self.index]

	if self.path == 1 and self.index == 2:
            self.disarm()
            rospy.sleep(10)

        # get the value of x and y from lat, long
        self.lat_to_x()
        self.long_to_y()

        # obstacle avoidance using laser scan and path planning (applicable only for this current task)
        if self.path == 1:           #laser values as per the drone orientation
            if (self.laser[3] <= 18.0000000000 and self.laser[3] >= 1.0000000000 and self.laser[3] is not inf and self.laser[3] != 0):  #Front value
                self.y = self.y - 25  # change the value in y direction to affect the drone's direction in longitude
                self.nextpos = [self.x_to_lat(self.x), self.y_to_long(self.y), 30.16]  # initialize next target location with the modfified values of lat, long
            elif (self.laser[4] <= 2.0000000000 and self.laser[4] >= 1.0000000000 and self.laser[4] is not inf and self.laser[4] != 0):   # Right value
                self.x = self.x - 20  # change the value in x direction to affect the drone's direction in latitude
                self.y = self.y + 6
                self.nextpos = [self.x_to_lat(self.x), self.y_to_long(self.y), 30.16]
            elif (self.laser[2] <= 2.0000000000 and self.laser[2] >= 1.0000000000 and self.laser[2] is not inf and self.laser[2] != 0):      #Left value
                self.y = self.y + 14
                self.nextpos = [self.x_to_lat(self.x), self.y_to_long(self.y), 30.16]
            elif (self.laser[1] <= 2.0000000000 and self.laser[1] >= 1.0000000000 and self.laser[1] is not inf and self.laser[1] != 0):      #Bacck value
                self.x = self.x + 20
                self.y = self.y + 6
                self.nextpos = [self.x_to_lat(self.x), self.y_to_long(self.y), 30.16]
            else:
                self.nextpos = self.target_position[self.index]

        # calcuate errors for roll pitch yaw
        self.error[0] = self.nextpos[0] - self.drone_position[0]
        self.error[1] = self.nextpos[1] - self.drone_position[1]
        self.error[2] = self.nextpos[2] - self.drone_position[2]

        self.diff_error[0] = self.error[0] - self.prev_error[0]
        self.diff_error[1] = self.error[1] - self.prev_error[1]
        self.diff_error[2] = self.error[2] - self.prev_error[2]

        # safe zone error limitation
        if self.error[0] > -0.000002017 and self.error[0] < 0.000002017 and self.error[1] > -0.0000047487 and \
                self.error[1] < 0.0000047487 and self.error[2] > -0.2 and self.error[2] < 0.2 and self.index < 3:
            self.nextpos = self.target_position[self.index]
            self.index += 1

        # Kp*e(t)
        self.p_roll = self.Kp[0] * self.error[0]
        self.p_pitch = self.Kp[1] * self.error[1]
        self.p_alt = self.Kp[3] * self.error[2]

        # integral(e(t))
        self.i_roll += self.error[0]
        self.i_pitch += self.error[1]
        self.i_alt += self.error[2]

        # derivative(e(t))
        self.d_roll = self.Kd[0] * self.diff_error[0]
        self.d_pitch = self.Kd[1] * self.diff_error[1]
        self.d_alt = self.Kd[3] * self.diff_error[2]

        # output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
        self.out_roll = self.p_roll + self.Ki[0] * self.i_roll + self.d_roll
        self.out_pitch = self.p_pitch + self.Ki[1] * self.i_pitch + self.d_pitch
        self.out_throttle = self.p_alt + self.Ki[3] * self.i_alt + self.d_alt

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        # compute the values that will be published on topic /drone_command
        self.cmd.rcRoll = 1500 + self.out_roll
        self.cmd.rcPitch = 1500 + self.out_pitch
        self.cmd.rcThrottle = 1500 + self.out_throttle

        # cap the values if they exceed the pre determined limits
        if self.cmd.rcThrottle > self.rc_throttle_max:
            self.cmd.rcThrottle = self.rc_throttle_max
        elif self.cmd.rcThrottle < self.rc_throttle_min:
            self.cmd.rcThrottle = self.rc_throttle_min

        if self.cmd.rcRoll > self.rc_roll_max:
            self.cmd.rcRoll = self.rc_roll_max
        elif self.cmd.rcRoll < self.rc_roll_min:
            self.cmd.rcRoll = self.rc_roll_min

        if self.cmd.rcPitch > self.rc_pitch_max:
            self.cmd.rcPitch = self.rc_pitch_max
        elif self.cmd.rcPitch < self.rc_pitch_min:
            self.cmd.rcPitch = self.rc_pitch_min

        self.edrone_pub.publish(self.cmd)
        self.z_error_pub.publish(self.error[2])
        self.zero_error_pub.publish(0)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
