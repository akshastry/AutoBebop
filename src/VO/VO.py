#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import sin, cos, atan2, asin, exp, sqrt
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# camera Parameters
f = 202
B = 0.03

# current state of quad
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0

#Original image in opencv
img_L = np.zeros((480,640,3), np.uint8)
img_R = np.zeros((480,640,3), np.uint8)

#ros Images
left_image = Image()
right_image = Image()
left_featured_image = Image()
right_featured_image = Image()
matched_featured_image = Image()
debug_image = Image()

#relative pose
pose_rel = Odometry()

def pose_estimation():
	global f, B
	global left_image, right_image, img_L, img_R
	global left_featured_image, right_featured_image, matched_featured_image
	global height, width, scale

	try:
		img_L = bridge.imgmsg_to_cv2(left_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	try:
		img_R = bridge.imgmsg_to_cv2(right_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	img_height = len(img_L)
	img_width = len(img_L[0])

	#Resize image
	scale = 1
	width = int(img_L.shape[1] * scale)
	height = int(img_L.shape[0] * scale)
	dim = (width, height) #can also just specify desired dimensions
	frame_L = cv2.resize(img_L, dim, interpolation = cv2.INTER_AREA)
	frame_R = cv2.resize(img_R, dim, interpolation = cv2.INTER_AREA)

	#Convert from BGR to gray colorspace
	frame_L = cv2.cvtColor(frame_L, cv2.COLOR_BGR2GRAY);
	frame_R = cv2.cvtColor(frame_R, cv2.COLOR_BGR2GRAY);

	img1 = frame_L.copy()
	img2 = frame_R.copy()

	# Initiate ORB detector
	orb = cv2.ORB_create()

	# find the keypoints and descriptors with ORB
	kp1, des1 = orb.detectAndCompute(img1,None)
	kp2, des2 = orb.detectAndCompute(img2,None)

	# create BFMatcher object
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

	# Match descriptors.
	matches = bf.match(des1,des2)

	# Sort them in the order of their distance.
	matches = sorted(matches, key = lambda x:x.distance)

	top_matches = matches[:20]

	# draw_params = dict(matchColor = (0,255,0),
	#                    singlePointColor = (255,0,0),
	#                    matchesMask = matchesMask,
	#                    flags = 0)

	# img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

	img3 = cv2.drawMatches(img1,kp1,img2,kp2,top_matches, None, flags=2)
	# img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,flags=2)

	# # Initiate SIFT detector
	# # sift = cv2.SIFT()
	# sift = cv2.xfeatures2d.SIFT_create()

	# # find the keypoints and descriptors with SIFT
	# kp1, des1 = sift.detectAndCompute(img1,None)
	# kp2, des2 = sift.detectAndCompute(img2,None)

	# # BFMatcher with default params
	# bf = cv2.BFMatcher()
	# matches = bf.knnMatch(des1,des2, k=2)

	# # Apply ratio test
	# good = []
	# for m,n in matches:
	#     if m.distance < 0.08*n.distance:
	#         good.append([m])
	#         print(m.trainIdx)

	# # cv2.drawMatchesKnn expects list of lists as matches.
	# img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=2)

	# Initialize lists
	list_kp1 = []
	list_kp2 = []
	list_disp = []
	list_z = []

	# For each match...
	for mat in top_matches:

		# Get the matching keypoints for each of the images
		img1_idx = mat.queryIdx
		img2_idx = mat.trainIdx

		# x - columns
		# y - rows
		# Get the coordinates
		(x1,y1) = kp1[img1_idx].pt
		(x2,y2) = kp2[img2_idx].pt
		if (x2-x1>0.001):
			z = (f*B)/(x2-x1)

		# Append to each list
		list_kp1.append((x1, y1))
		list_kp2.append((x2, y2))
		list_disp.append(x2-x1)
		list_z.append(z)

	if len(list_z)>0:
		mean_z = sum(list_z)/len(list_z)
		print(mean_z)

	matched_featured_image = bridge.cv2_to_imgmsg(img3, "8UC3")

	return

def left_image_assign(image):
	global left_image
	left_image = image

def right_image_assign(image):
	global right_image
	right_image = image

def quaternion_to_euler(w, x, y, z):

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll = atan2(t0, t1)
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch = asin(t2)
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw = atan2(t3, t4)
	# return [yaw, pitch, roll]
	return [roll, pitch, yaw]

def main():
	rospy.init_node('target_detect', anonymous=True)

	pub_pose_rel = rospy.Publisher('/pose_rel_target', Odometry, queue_size=10)
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)
	pub_left_featured_image = rospy.Publisher('/left_featured_image', Image, queue_size=10)
	pub_right_featured_image = rospy.Publisher('/right_featured_image', Image, queue_size=10)
	pub_matched_featured_image = rospy.Publisher('/matched_featured_image', Image, queue_size=10)

	rospy.Subscriber('/duo3d/left/image_rect', Image, left_image_assign)
	rospy.Subscriber('/duo3d/right/image_rect', Image, right_image_assign)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		# try:
		# 	pose_estimation()
		# except:
		# 	rospy.loginfo('Some error ocurred... in target_detect.py')

		pose_estimation()

		pub_matched_featured_image.publish(matched_featured_image)
		pub_debug_image.publish(debug_image)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
