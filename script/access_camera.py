#!/usr/bin/env python

################################################################################
## {Description}: Accessing raspicam/usbcam
## Original Code from Raspberry Pi for Computer Vision - Hobbyist Bundle
## Re-use and transform into ROS nodes
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from __future__ import division

from imutils.video import VideoStream
import imutils
import cv2
import os
import imutils
import time

# import the necessary ROS packages
import sys
import rospy
import rospkg

from std_msgs.msg import Int32

rospack = rospkg.RosPack()

class AccessCamera_node:
	def __init__(self):
		#global pub

		# Initializing your ROS Node
		rospy.init_node('AccessCamera_node', anonymous=True)

		self.rate = rospy.Rate(5)

		rospy.on_shutdown(self.shutdown)

		# initialize the video stream and allow the cammera sensor to warmup
		rospy.logwarn("Starting video stream...")
		self.vs = VideoStream(src=0).start()
		#self.vs = VideoStream(usePiCamera=True, resolution=(640, 480)).start()

		time.sleep(2.0)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] AccessCamera_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	# grab the frame from the  video stream and resize it to have a
	# maximum width of 400 pixels
	def grabFrame(self):
		while not rospy.is_shutdown():
			self.frame = self.vs.read()
			self.frame = imutils.resize(self.frame, width=400)

			self.preview()

	# show the output image and the final shape count
	def preview(self):

		# show the output frame
		cv2.imshow("Frame", self.frame)
		self.key = cv2.waitKey(1) & 0xFF

		# if the `q` key was pressed, break from the loop
		if self.key == ord("q"):
			self.shutdown()

def main(args):
	#global pub

	vn = AccessCamera_node()

	while not rospy.is_shutdown():
		try:
			vn.grabFrame()
			rospy.spin()

		except KeyboardInterrupt:
			rospy.loginfo("[INFO] AccessCamera_node [OFFLINE]...")

		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] AccessCamera_node [ONLINE]...")
	main(sys.argv)
