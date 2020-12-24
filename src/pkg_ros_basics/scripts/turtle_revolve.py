#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def pose_callback(pose):
    rospy.loginfo("Robot X = %f : Y = %f : Z = %f \n",pose.x,pose.y,pose.theta)

def turtle_revolve():
    # 1. Initializing the Ros node :
    rospy.init_node('turtle_revolve', anonymous=True)
    # 2. Creating a publish myMsg
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    vel_msg = Twist()
    rate = rospy.Rate(10)   #10Hz

    radius = 2
    angular_speed = 1
    distance = 2 * 3.25 * radius
    speed = angular_speed * radius

    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = angular_speed

    count = None
    while count != 1:
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        i = 0
        while(current_distance < distance):
            velocity_publisher.publish(vel_msg)
            print(current_distance)
            rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_distance = speed*(t1-t0)

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        count = 1
        velocity_publisher.publish(vel_msg)
        print("DONE")

    #terminate loop
    rospy.is_shutdown()

if __name__ == '__main__':
  try:
      turtle_revolve()
  except rospy.ROSInterrptException:
      pass
