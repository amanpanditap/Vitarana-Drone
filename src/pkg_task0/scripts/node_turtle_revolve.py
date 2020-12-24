#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys


def pose_callback(msg):
    rospy.loginfo("Moving in a circle")


def main():
    # Initializing the Ros node :
    rospy.init_node('turtle_node_revolve', anonymous=True)

    # Creating a publish myMsg
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate = rospy.Rate(62.5)  # 62.5hz

    # algebric operations
    speed = 2
    angular_speed = 1
    radius = speed / angular_speed
    distance = 2 * 3.249998 * radius  # circumference of circle

    # setting linear and angular velocities
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = angular_speed

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    # Creating Subscriber pose
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    print "Move"
    # Distance Algorithm for revolution of 1 complete circle
    while (current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        rate.sleep()
        print current_distance
        current_distance = speed * (t1 - t0)

    # Circular revolution complete
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("Goal Reached")
    rospy.is_shutdown()
    sys.exit()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
