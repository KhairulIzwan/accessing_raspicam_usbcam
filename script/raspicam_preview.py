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

class RaspicamPreview:
	def __init__(self):

		self.bridge = CvBridge()

		rospy.on_shutdown(self.shutdown)

		# Subscribe to Image msg
		self.image_topic = "/raspicam_node_robot/image/compressed"
		self.image_sub = rospy.Subscriber(self.image_topic, CompressedImage, self.cbImage)

	def cbImage(self,data):

		try:
			# Convert the raw image to OpenCV format """
			# self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# direct conversion to CV2 ####
			self.cv_image = np.fromstring(msg.data, np.uint8)
			self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

			# OTIONAL -- image-rotate """
			self.cv_image = imutils.rotate(self.cv_image, angle=-90)
			self.cv_image = cv2.flip(self.cv_image,1)
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

		# Get the width and height of the image
		self.cbCameraInfo()

		# Overlay some text onto the image display
		self.showInfo()

		# Refresh the image on the screen
		self.displayImg()

	# Get the width and height of the image
	def cbCameraInfo(self):

		self.imgWidth = rospy.get_param("/raspicam_node_robot/width") 
		self.imgHeight = rospy.get_param("/raspicam_node_robot/height") 

		rospy.set_param("/raspicam_node_robot/raspicam_node_robot/vFlip", True)

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

	# Refresh the image on the screen
	def displayImg(self):

		cv2.imshow("RaspicamPreview", self.cv_image)
		cv2.waitKey(1)

	# Shutdown
	def shutdown(self):

		rospy.logwarn("RaspicamPreview Node [OFFLINE]...")
		cv2.destroyAllWindows()

def main(args):

	# Initializing your ROS Node
	rospy.init_node('raspicam_preview', anonymous=False)
	camera = RaspicamPreview()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.logerr("RaspicamPreview Node [OFFLINE]...")

if __name__ == '__main__':

	main(sys.argv)
