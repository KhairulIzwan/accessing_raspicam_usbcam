#!/usr/bin/env python

################################################################################
## {Description}: Preview an Image from USB type camera
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
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class CameraPreview_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('CameraPreview_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Camera Preview"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/cv_camera/image_raw", 
				Image, self.callback)

		# Subscribe to the camera info topic
		self.imgInfo_sub = rospy.Subscriber("/cv_camera/camera_info", 
				CameraInfo, self.getCameraInfo)

	def callback(self,data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Overlay some text onto the image display
		self.textInfo()

		# Refresh the image on the screen
		self.displayImg()

	# Get the width and height of the image
	def getCameraInfo(self, msg):
		self.image_width = msg.width
		self.image_height = msg.height

	# Convert the raw image to OpenCV format
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format """
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# TODO:
			#self.cv_image = imutils.rotate(self.cv_image, angle=180)
			self.cv_image = cv2.flip(self.cv_image,1)
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

	# Overlay some text onto the image display
	def textInfo(self):
		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

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
			rospy.loginfo("CameraPreview_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

def main(args):
	vn = CameraPreview_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("CameraPreview_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("CameraPreview_node [ONLINE]...")
	main(sys.argv)
