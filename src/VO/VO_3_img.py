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

# time stuff
dt_L = 0.0
t_L_old = 0.0

#Original image in opencv
img_L = np.zeros((480,640,3), np.uint8)
img_R = np.zeros((480,640,3), np.uint8)
frame_L = np.zeros((480,640,1), np.uint8)
frame_R = np.zeros((480,640,1), np.uint8)
frame_L_prev = np.zeros((480,640,1), np.uint8)
frame_R_prev = np.zeros((480,640,1), np.uint8)

#ros Images
left_image = Image()
right_image = Image()
left_featured_image = Image()
right_featured_image = Image()
spatially_matched_featured_image = Image()
temporally_matched_featured_image = Image()
flow_left_image = Image()
debug_image = Image()

#relative pose
pose_rel = Odometry()

def pose_estimation():
	global f, B
	global frame_L, frame_R, frame_L_prev, frame_R_prev
	global left_image, right_image, img_L, img_R
	global left_featured_image, right_featured_image, spatially_matched_featured_image
	global img_L, frame_L, frame_L_prev, dt_L
	global temporally_matched_featured_image, flow_left_image
	global height, width, scale

	frame_L_prev = frame_L
	frame_R_prev = frame_R

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
	# print(img_height)
	# print(img_width)

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
	img3 = frame_L_prev.copy()
	flow_image = img_L.copy()

	# Initiate ORB detector
	orb = cv2.ORB_create()

	# find the keypoints and descriptors with ORB
	kp1, des1 = orb.detectAndCompute(img1,None)
	kp2, des2 = orb.detectAndCompute(img2,None)
	kp3, des3 = orb.detectAndCompute(img3,None)

	# create BFMatcher object
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

	if des3 is not None:
		# Match descriptors.
		matches_sp = bf.match(des1,des2)
		matches_tm = bf.match(des1,des3)

		# Sort them in the order of their distance.
		matches_sp = sorted(matches_sp, key = lambda x:x.distance)
		matches_tm = sorted(matches_tm, key = lambda x:x.distance)

		matches_len = min(len(matches_sp),len(matches_tm))
		matches_sp = matches_sp[:int(0.4*matches_len)]
		matches_tm = matches_tm[:int(0.4*matches_len)]

		# Initialize lists
		list_X1 = []
		list_X2 = []
		list_X3 = []
		list_z = []
		list_vx_img = []
		list_vy_img = []

		# For each match...
		for mat in matches_sp:

			# Get the matching keypoints for each of the images
			img1_idx_sp = mat.queryIdx
			img2_idx_sp = mat.trainIdx

			for mat_tm in matches_tm:
				# Get the matching keypoints for each of the images
				img1_idx_tm = mat_tm.queryIdx
				img3_idx_tm = mat_tm.trainIdx

				if (img1_idx_tm == img1_idx_sp):
					break

			if (img1_idx_tm == img1_idx_sp):
				# x - columns
				# y - rows
				# Get the coordinates
				(x1,y1) = kp1[img1_idx_sp].pt
				(x2,y2) = kp2[img2_idx_sp].pt
				(x3,y3) = kp3[img3_idx_tm].pt

				# print((x1,x2,x3,y1,y2,y3))

				if (x2-x1>0.00001):
					z = (f*B)/(x2-x1)

					vx_img = (x3-x1)/dt_L
					vy_img = (y3-y1)/dt_L

					list_X1.append((x1, y1))
					list_X2.append((x2, y2))
					list_X3.append((x3, y3))
					list_z.append(z)
					list_vx_img.append(vx_img)
					list_vy_img.append(vy_img)

					cv2.arrowedLine(flow_image, (int(x1),int(y1)), (int(x3),int(y3)), (0,0,255), thickness=2, line_type=8, shift=0, tipLength=0.5)
		
		flow_left_image = bridge.cv2_to_imgmsg(flow_image, "8UC3")

		plot_image = cv2.drawMatches(img1,kp1,img2,kp2,matches_sp, None, flags=2)
		spatially_matched_featured_image = bridge.cv2_to_imgmsg(plot_image, "8UC3")

		plot_image = cv2.drawMatches(img1,kp1,img3,kp3,matches_tm, None, flags=2)
		temporally_matched_featured_image = bridge.cv2_to_imgmsg(plot_image, "8UC3")


def left_image_assign(image):
	global left_image
	global t_L_old, dt_L

	left_image = image

	t_L = rospy.get_time()
	dt_L = t_L - t_L_old
	t_L_old = t_L

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
	pub_spatially_matched_featured_image = rospy.Publisher('/spatially_matched_featured_image', Image, queue_size=10)
	pub_temporally_matched_featured_image = rospy.Publisher('/temporally_matched_featured_image', Image, queue_size=10)
	pub_flow_left_image = rospy.Publisher('/flow_left_image', Image, queue_size=10)

	rospy.Subscriber('/duo3d/left/image_rect', Image, left_image_assign)
	rospy.Subscriber('/duo3d/right/image_rect', Image, right_image_assign)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		# try:
		# 	z = height_estimation()
		# 	vel_estimation_left()
		# except:
		# 	rospy.loginfo('Some error ocurred... in target_detect.py')

		pose_estimation()

		pub_spatially_matched_featured_image.publish(spatially_matched_featured_image)
		pub_temporally_matched_featured_image.publish(temporally_matched_featured_image)
		pub_flow_left_image.publish(flow_left_image)
		pub_debug_image.publish(debug_image)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
