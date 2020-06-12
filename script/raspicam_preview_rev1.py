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
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class RaspicamPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False

		# Connect image topic
		img_topic = "/raspicam/image/compressed"
		self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.cbImage)

		# Allow up to one second to connection
		rospy.sleep(1)

	def cbImage(self, msg):

		# Convert image to OpenCV format
		try:
			self.cv_image = np.fromstring(msg.data, np.uint8)
			self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

			# OPTIONAL -- image-rotate """
			self.cv_image = imutils.rotate(self.cv_image, angle=-90)

		except CvBridgeError as e:
			print(e)

		self.image_received = True

	# Get the width and height of the image
	def cbCameraInfo(self):

		self.imgWidth = rospy.get_param("/raspicam/width") 
		self.imgHeight = rospy.get_param("/raspicam/height") 

		rospy.set_param("/raspicam/vFlip", True)

	# Overlay some text onto the image display
	def showInfo(self):

		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)
		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
			color, thickness, lineType, bottomLeftOrigin)

	# show the output image and the final shape count
	def preview(self):

		if self.image_received:
#			self.cbCameraInfo()

			# Overlay some text onto the image display
#			self.showInfo()

			# show the output frame
			cv2.imshow("RaspicamPreview", self.cv_image)
			cv2.waitKey(1)

		else:
			rospy.logerr("No images recieved")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('raspicam_preview', anonymous=False)
	camera = RaspicamPreview()

	# Camera preview
	while not rospy.is_shutdown():
		camera.preview()
