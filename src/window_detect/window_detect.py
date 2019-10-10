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

raw_image = Image()
pose_rel = Odometry()
bin_image = Image()
debug_image = Image()

bridge = CvBridge()

def thresholding():
	global raw_image, bin_image, debug_image, pose_rel
	cv_raw_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding="passthrough")
	edges= cv2.Canny(cv_raw_image,100,200)
	# plt.subplot(121),plt.imshow(imgage,cmap = 'gray')
	# plt.title('Original Image'), plt.xticks([]), plt.yticks([])
	# plt.subplot(122),plt.imshow(edges,cmap = 'gray')
	# plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
	# plt.show()

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
