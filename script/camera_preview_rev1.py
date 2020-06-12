#!/usr/bin/env python

################################################################################
## {Description}: Accessing raspicam/usbcam
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class CameraPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False

		rospy.on_shutdown(self.shutdown)

		# Subscribe to Image msg
		self.image_topic = "/cv_camera/image_raw"
		self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cbImage)

		# Subscribe to CameraInfo msg
		self.cameraInfo_topic = "/cv_camera/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(self.cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)

		rospy.logwarn("CameraPreview Node [ONLINE]...")

		# Allow up to one second to connection
		rospy.sleep(1)

	def cbImage(self, msg):

		# Convert image to OpenCV format
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_received = True

	def cbCameraInfo(self, msg):

		# Get CameraInfo
		self.imgWidth = msg.width
		self.imgHeight = msg.height

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
			self.showInfo()

			# show the output frame
			cv2.imshow("CameraPreview", self.cv_image)
			cv2.waitKey(1)

		else:
			rospy.logerr("No images recieved")

	def shutdown(self):
		rospy.logerr("CameraPreview Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_preview', anonymous=False)
	camera = CameraPreview()

	# Camera preview
	while not rospy.is_shutdown():
		camera.preview()
