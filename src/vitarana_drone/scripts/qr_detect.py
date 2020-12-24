#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from vitarana_drone.msg import *
import cv2
import numpy as np
import rospy

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.gps_location = rospy.Publisher("/qrcode_gps", position, queue_size=1)
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.gps = [0, 0, 0]
		self.msg = position()
		self.msg.lat = 0.0
		self.msg.long = 0.0
		self.msg.alt = 0.0


	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			result = decode(self.img)
			for i in result:
				self.gps = i.data.decode("utf-8").split(',')
				self.msg = position()
				self.msg.lat = float(self.gps[0])
				self.msg.long = float(self.gps[1])
				self.msg.alt = float(self.gps[2])
		except CvBridgeError as e:
			print(e)
			return
		self.gps_location.publish(self.msg)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
