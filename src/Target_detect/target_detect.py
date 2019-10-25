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


#Original image in opencv
img = np.zeros((480,640,3), np.uint8)

#ros Images
raw_image = Image()
bin_image = Image()
debug_image = Image()

#circle parameters
a= 0.0
cx = 0.0
cy = 0.0

#relative pose
pose_rel = Odometry()

def thresholding():
	global raw_image, bin_image
	global img
	global height, width, scale

	try:
		img = bridge.imgmsg_to_cv2(raw_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	img_height = len(img)
	img_width = len(img[0])


	#Resize image
	scale = 1
	width = int(img.shape[1] * scale)
	height = int(img.shape[0] * scale)
	dim = (width, height) #can also just specify desired dimensions
	frame = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
	print('Image Height:')
	print(height)
	print('Image Width:')
	print(width)


	#Convert from BGR to gray colorspace
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);

	gray_img = frame.copy()

	lower = (150) #lower threshhold values (H, S, V)
	upper = (255) #upper threshhold values (H, S, V)
	frame = cv2.inRange(frame, lower, upper)

	#Erosion/ Dilation
	kernel = np.ones((2,2), np.uint8) 
	frame = cv2.erode(frame, kernel, iterations=3)
	kernel = np.ones((2,2), np.uint8) 
	frame = cv2.dilate(frame, kernel, iterations=3)

	bin_image = bridge.cv2_to_imgmsg(frame, "8UC1")

	return frame

def get_circle(frame):
	global a, cx, cy
	circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,1,1,
                            param1=100,param2=50,minRadius=0,maxRadius=0)
	a = []
	cx = []
	cy = []
	img1 = img.copy()
	if circles is not None:
	    circles = np.uint16(np.around(circles))
	    for i in circles[0,:]:
	        # draw the outer circle
	        cv2.circle(img1,(i[0],i[1]),i[2],(0,255,0),2)
	        # draw the center of the circle
	        cv2.circle(img1,(i[0],i[1]),2,(255,0,0),3)
	        
	        a.append(i[2])
	        cx.append(i[0])
	        cy.append(i[1])


	a = np.average(a)
	cx = np.average(cx)
	cy = np.average(cy)

	img2 = img.copy()
	cv2.circle(img2,(cx.astype(int),cy.astype(int)),a.astype(int),(0,255,0),5)
	cv2.circle(img2,(cx.astype(int),cy.astype(int)),2,(255,0,0),5)

	debug_image = bridge.cv2_to_imgmsg(img2, "8UC3")


def get_pose():
	global a, cx, cy
	#Image coordinates
	u1 = cx
	v1 = cy
	u2 = cx+a
	v2 = cy

	#Real world Info
	R = 0.25

	#Camera Parameters
	fx = 300
	fy = 300
	cam_cx = width/2.0
	cam_cy = height/2.0

	z = R*fx*fy*(1.0/(fx**2*v1**2 - 2.0*fx**2*v1*v2 + fx**2*v2**2 + fy**2*u1**2 - 2*fy**2*u1*u2 + fy**2*u2**2))**(0.5)

	x1 = -R*fy*(cam_cx - u1)*(1.0/(fx**2*v1**2 - 2.0*fx**2*v1*v2 + fx**2*v2**2 + fy**2*u1**2 - 2.0*fy**2*u1*u2 + fy**2*u2**2))**(0.5)

	y1 = -R*fx*(cam_cy - v1)*(1.0/(fx**2*v1**2 - 2.0*fx**2*v1*v2 + fx**2*v2**2 + fy**2*u1**2 - 2.0*fy**2*u1*u2 + fy**2*u2**2))**(0.5)

	rospy.loginfo('x %f \t y %f \t z %f', x1, y1, z)
	pose_rel.header.frame_id = "odom"
	pose_rel.child_frame_id = "base_link"
	pose_rel.header.stamp = rospy.get_rostime()
	pose_rel.pose.pose.position.x = x1
	pose_rel.pose.pose.position.y = y1
	pose_rel.pose.pose.position.z = z
	pose_rel.twist.twist.linear.x = 0.0
	pose_rel.twist.twist.linear.y = 0.0
	pose_rel.twist.twist.linear.z = 0.0
	pose_rel.pose.pose.orientation.w = 1.0
	pose_rel.pose.pose.orientation.x = 0.0
	pose_rel.pose.pose.orientation.y = 0.0
	pose_rel.pose.pose.orientation.z = 0.0
	pub_pose_rel.publish(pose_rel)


def callback(image):
	global raw_image
	raw_image = image

def main():
	global raw_image, bin_image, pub_pose_rel
	rospy.init_node('target_detect', anonymous=True)

	pub_pose_rel = rospy.Publisher('/pose_rel_target', Odometry, queue_size=10)
	pub_bin_image = rospy.Publisher('/bin_image', Image, queue_size=10)
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)

	rospy.Subscriber('/duo3D/image_left_rect', Image, callback) #should be faster than 20Hz or the rate of publishing below, else EKF might get fucked up

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		# try:
		# 	frame = thresholding()
		# 	get_circle(frame)
		# 	get_pose()
		# except:
		# 	rospy.loginfo('Some error ocurred... in target_detect.py')

		frame = thresholding()
		get_circle(frame)
		get_pose()

		pub_bin_image.publish(bin_image)
		pub_debug_image.publish(debug_image)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
