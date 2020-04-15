#!/usr/bin/env python

################################################################################
## {Description}: Preview an Image from Raspberry Pi Camera (raspicam)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

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

def main(args):
	vn = RaspicamPreview_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] Raspicam_Preview_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] Raspicam_Preview_node [ONLINE]...")
	main(sys.argv)
