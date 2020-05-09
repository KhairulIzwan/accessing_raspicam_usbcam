#!/usr/bin/env python

################################################################################
## {Description}: Preview an Image from Raspberry Pi Camera (raspicam)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from __future__ import division

import sys
import cv2
import time
import numpy as np

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class RaspicamPreview_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('RaspicamPreview_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Camera Preview"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/raspicam_node_robot/image/compressed", 
				CompressedImage, self.callback, queue_size=1)

	def callback(self,data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Get the width and height of the image
		self.getCameraInfo()

		# Overlay some text onto the image display
		self.textInfo()

		# Refresh the image on the screen
		self.displayImg()

	# Get the width and height of the image
	def getCameraInfo(self):
		self.image_width = rospy.get_param("/raspicam_node_robot/width") 
		self.image_height = rospy.get_param("/raspicam_node_robot/height") 

		rospy.set_param("/raspicam_node_robot/raspicam_node_robot/vFlip", True)

	# Convert the raw image to OpenCV format
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format """
			# self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# direct conversion to CV2 ####
			self.cv_image = np.fromstring(data.data, np.uint8)
			self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

			# OTIONAL -- image-rotate """
			self.cv_image = imutils.rotate(self.cv_image, angle=-90)
			self.cv_image = cv2.flip(self.cv_image,1)
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

	# Overlay some text onto the image display
	def textInfo(self):
		cv2.putText(self.cv_image, "Sample", (10, self.image_height-10), 
			cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, 
			cv2.LINE_AA, False)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.image_width, 
			self.image_height), (self.image_width-100, 
			self.image_height-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, 
			(255, 255, 255), 1, cv2.LINE_AA, False)

	# Refresh the image on the screen
	def displayImg(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] Raspicam_Preview_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

class RaspicamPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False

		# Connect image topic
		img_topic = "/cv_camera/image_raw"
		self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

		# Allow up to one second to connection
		rospy.sleep(1)

	def callback(self, data):

		# Convert image to OpenCV format
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_received = True
		self.image = cv_image

	# show the output image and the final shape count
	def preview(self):

		if self.image_received:
			# Overlay some text onto the image display
			timestr = time.strftime("%Y%m%d-%H%M%S")
			cv2.putText(self.image, timestr, 
				(10, 20), 1, 1, 
				(255, 255, 255), 1, cv2.LINE_AA, False)

			# show the output frame
			cv2.imshow("Frame", self.image)
			cv2.waitKey(1)

			return True
		else:
			return False

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_preview', anonymous=False)
	camera = CameraPreview()

	# Camera preview
	while not rospy.is_shutdown():
		camera.preview()
