#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import atan2, sin, cos
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cv_raw_image = np.zeros((480,640,3), np.uint8)
raw_image = Image()
pose_rel = Odometry()
bin_image = Image()
debug_image = Image()

bridge = CvBridge()

def thresholding():
	global raw_image, bin_image, debug_image, pose_rel, cv_raw_image

	try:
		cv_raw_image = bridge.imgmsg_to_cv2(raw_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	b,g,r = cv2.split(cv_raw_image)
	gray = cv2.cvtColor(cv_raw_image, cv2.COLOR_BGR2GRAY)

	# thresholding
	b_thres = cv2.adaptiveThreshold(b,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
	g_thres = cv2.adaptiveThreshold(g,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
	r_thres = cv2.adaptiveThreshold(r,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
	cv_thres_image = cv2.add(b_thres,g_thres,r_thres)

	#edge
	edges = cv2.Canny(gray,100,200)

	bin_image = bridge.cv2_to_imgmsg(cv_thres_image, "8UC1")
	# bin_image = bridge.cv2_to_imgmsg(edges, "8UC1")

	# rospy.loginfo('%f',1.0)

def corner_detection():
	global raw_image, bin_image, debug_image, pose_rel
	debug_image = bin_image

def pose_solve():
	global raw_image, bin_image, debug_image, pose_rel
	pose_rel = pose_rel

def callback(image):
	global raw_image, bin_image, debug_image, pose_rel
	raw_image = image

def main():
	global raw_image, bin_image, debug_image, pose_rel
	rospy.init_node('window_detect', anonymous=True)

	pub_pose_rel = rospy.Publisher('/pose_rel', Odometry, queue_size=10)
	pub_bin_image = rospy.Publisher('/bin_image', Image, queue_size=10)
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)

	rospy.Subscriber('/cv_camera/image_raw', Image, callback)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		thresholding()
		corner_detection()
		pose_solve()
		pub_bin_image.publish(bin_image)
		pub_debug_image.publish(debug_image)
		pub_pose_rel.publish(pose_rel)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
