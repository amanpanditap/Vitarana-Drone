#!/usr/bin/env python
# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""

    def __init__(self):
        rospy.init_node('position_controller')
        # initial position of drone
        self.drone_position = [19.0, 72.0, 0.31]
        # target_position to achieve
        self.target_position = [[19.0, 72.0, 3.0], [19.0000451704, 72.0, 3.0], [19.0000451704, 72.0, 0]]

        #edrone_cmd initial setup
        self.cmd = edrone_cmd()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.aux1 = 1500
        self.cmd.aux2 = 1500
        self.cmd.aux3 = 1500
        self.cmd.aux4 = 1500

        # initial setting o Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        self.Kp = [146.7, 209.58, 0, 180]
        self.Ki = [0, 0, 0, 0]
        self.Kd = [412.8, 85.2, 0, 209.7]
        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
        self.error = [0, 0, 0, 0]
        self.prev_error = [0, 0, 0, 0]
        self.diff_error = [0, 0, 0, 0]
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

        # flag
        self.flag = 0
        # index
        self.index = 0
        # nextpos tracker
        self.nextpos = [0, 0, 0]
        # set next target_position
        self.nextpos = self.target_position[self.index]
        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.03  # in seconds
        self.current_time = 0.0
        self.prev_time = 0.0
        self.delta_time = 0.0

        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.edrone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------
        # Subscriber
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # ------------------------------------------------------------------------------------------------------------

        # ---------------------------------------------------------------------------------------------------------------

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

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def gps_callback(self, msg):
        self.drone_position[0] = msg.latitude
        self.drone_position[1] = msg.longitude
        self.drone_position[2] = msg.altitude

    # ---------------------------------------------------------------------------------------------------------------------

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

        self.current_time = time.time()
        self.delta_time = self.current_time - self.prev_time

        # calcuate errors for roll pitch yaw
        self.error[0] = self.nextpos[0] - self.drone_position[0]
        self.error[1] = self.nextpos[1] - self.drone_position[1]
        self.error[3] = self.nextpos[2] - self.drone_position[2]

        self.diff_error[0] = self.error[0] - self.prev_error[0]
        self.diff_error[1] = self.error[1] - self.prev_error[1]
        self.diff_error[3] = self.error[3] - self.prev_error[3]

        # error range for caluclating the nextpos
        if self.error[0] > -0.0000022585 and self.error[0] < 0.0000022585 and self.error[1] > -0.0000047487 and \
                self.error[1] < 0.0000047487 and self.error[2] > -0.2 and self.error[2] < 0.2 and self.index < len(
                self.target_position):
            self.nextpos = self.target_position[self.index]
            self.index += 1

        # Kp*e(t)
        self.p_roll = self.Kp[0] * self.error[0]
        self.p_pitch = self.Kp[1] * self.error[1]
        self.p_yaw = self.Kp[2] * self.error[2]
        self.p_alt = self.Kp[3] * self.error[3]

        # integral(e(t))
        self.i_roll += self.error[0] * self.delta_time
        self.i_pitch += self.error[1] * self.delta_time
        self.i_yaw += self.error[2] * self.delta_time
        self.i_alt += self.error[3] * self.delta_time

        # derivative(e(t))
        self.d_roll = self.Kd[0] * self.diff_error[0] / self.delta_time
        self.d_pitch = self.Kd[1] * self.diff_error[1] / self.delta_time
        self.d_yaw = self.Kd[2] * self.diff_error[2] / self.delta_time
        self.d_alt = self.Kd[3] * self.diff_error[3] / self.delta_time

        # output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
        self.out_roll = self.p_roll + self.Ki[0] * self.i_roll + self.d_roll
        self.out_pitch = self.p_pitch + self.Ki[1] * self.i_pitch + self.d_pitch
        self.out_yaw = self.p_yaw + self.Ki[2] * self.i_yaw + self.d_yaw
        self.out_throttle = self.p_alt + self.Ki[3] * self.i_alt + self.d_alt

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        self.prev_error[3] = self.error[3]
        self.prev_time = self.current_time

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


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(30)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
