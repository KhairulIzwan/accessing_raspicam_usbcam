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
		self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback)

		# Allow up to one second to connection
		rospy.sleep(1)

	def callback(self, data):

		# Convert image to OpenCV format
		try:
			cv_image = np.fromstring(data.data, np.uint8)
			cv_image = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)

			# OPTIONAL -- image-rotate """
			cv_image = imutils.rotate(cv_image, angle=-90)

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

		else:
			rospy.logerr("No images recieved")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('raspicam_preview', anonymous=False)
	camera = RaspicamPreview()

	# Camera preview
	while not rospy.is_shutdown():
		camera.preview()
