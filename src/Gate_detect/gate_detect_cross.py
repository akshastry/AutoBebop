#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import sin, cos, atan2, asin, exp, sqrt
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty, Int32
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from decimal import Decimal

master_mission_no = 0


kP_cross = 0.3
max_bias = .1 #meters

# current state of quad
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0

cross_bias = 0.0

img = np.zeros((480,640,3), np.uint8)
raw_image = Image()
bin_image_cross = Image()

bridge = CvBridge()

pose_gate_in = Odometry()

def remove_distortion(img):
	width  = img.shape[1]
	height = img.shape[0]

	dist_coeffs = np.array([-0.271345, 0.06, -0.000446, -0.000109, 0.0]) #get from camera calibration

	# assume unit matrix for camera
	cam = np.eye(3,dtype=np.float32)

	cam[0,2] = width/2.0  # define center x
	cam[1,2] = height/2.0 # define center y

	focal_length_x = 353.939474 #get from camera calibration
	focal_length_y = 353.169928 #get from camera calibration
	cam[0,0] = focal_length_x        # define focal length x
	cam[1,1] = focal_length_y        # define focal length y

	img = cv2.undistort(img,cam,dist_coeffs)

	return img

def thresholding():
	global raw_image, bin_image_cross, frame
	global img, frame, img_orig, blank_image, img_lines_bin, img_corners
	global height, width, scale

	try:
		img = bridge.imgmsg_to_cv2(raw_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	#img = remove_distortion(img)

	img_orig = img.copy()

	#Resize image
	scale = 1.0
	width = int(img.shape[1] * scale)
	height = int(img.shape[0] * scale)
	dim = (width, height) #can also just specify desired dimensions
	# img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

	blank_image = np.zeros(shape=[height, width, 3], dtype=np.uint8)

	#Convert from BGR to HSV colorspace
	frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV);

	#Guassian blur
	blur_params = (5,5)
	frame = cv2.GaussianBlur(frame,blur_params,cv2.BORDER_DEFAULT)

	#yellow
	lower = (0, 80, 40) #lower threshhold values (H, S, V)
	upper = (80, 255, 250) #upper threshhold values (H, S, V)
	frame = cv2.inRange(frame, lower, upper)


	#Erosion/dilation
	kernel = np.ones((6,6), np.uint8) 
	frame = cv2.erode(frame, kernel, iterations=1)
	kernel = np.ones((2,2), np.uint8) 
	frame = cv2.dilate(frame, kernel, iterations=1) 

	bin_image_cross = bridge.cv2_to_imgmsg(frame, "8UC1")

def getBias():
	global bin_image_cross, height, width, scale, cross_bias, frame

	M = cv2.moments(frame)
	cross_bias_prev = cross_bias
	if M["m00"] > 0:
		cX = int(M["m10"] / M["m00"])
		area_ratio = 1.0*np.count_nonzero(frame)/(height*width)
		#draw circle on troubleshooting image
		frame = cv2.circle(frame, (int(cX), height/2), 5, (255, 255, 255), -1)
		bin_image_cross = bridge.cv2_to_imgmsg(frame, "8UC1")
		#convert cX to normalized image coordinates
		cX = (cX - width/2.0)/width

		if area_ratio > 0.015 and abs(cX) > 0.075: #tune these if needed
			cross_bias = kP_cross*cX
			if abs(cross_bias) > max_bias: #limit bias to max magnitude allowed
				cross_bias = max_bias*np.sign(cross_bias)
		else:
			cross_bias = 0.0
		#print('center = ', cX,'area_ratio = ',area_ratio,'bias = ',cross_bias)
	else:
		cross_bias = 0.0

	#lp filter bias
	B = 1.0
	cross_bias = B*cross_bias + (1.0 - B)*cross_bias_prev
	#final limits on cross bias
	if abs(cross_bias) < .0001: #min bias, so filter doesn't keep decreasing
		cross_bias = 0.0
	if abs(cross_bias) > max_bias: #limit bias to max magnitude allowed
		cross_bias = max_bias*np.sign(cross_bias)

	print(cross_bias)

def callback(image):
	global raw_image
	raw_image = image

def get_master_mission(data):
	global master_mission_no
	master_mission_no = data.data


####################################################################################################################################################



def main():
	global raw_image, bin_image_cross
	rospy.init_node('window_detect_cross', anonymous=True)

	pub_bin_image_cross = rospy.Publisher('/bin_image_cross', Image, queue_size=10)
	pub_cross_bias = rospy.Publisher('/cross_bias', Float64, queue_size=1, latch=True)

	# rospy.Subscriber('/cv_camera/image_raw', Image, callback)
	rospy.Subscriber('/image_raw', Image, callback)
	# rospy.Subscriber('/image_raw_throttle', Image, callback)

	rospy.Subscriber('/master_mission_no', Int32, get_master_mission)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		#if (master_mission_no == 1): #change when done troubleshooting
		if (master_mission_no == 1):
			# try:
			# 	thresholding()
			# 	corners = get_corners()
			# 	flag_publish = pose_solve(corners)
			# 	if (flag_publish):
			# 		pose_display(corners)
			# except:
			# 	rospy.loginfo('Some error ocurred')

			thresholding()
			getBias()

			pub_bin_image_cross.publish(bin_image_cross)
			pub_cross_bias.publish(cross_bias)
	
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
