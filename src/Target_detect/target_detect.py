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

master_mission_no = 8

# observation factors for camera (tuning param), should not be necessary if camera is properly calibrated and pnp is working
obs_factor_x = 1.0
obs_factor_y = 1.0
obs_factor_z = 1.0

obs_offset_x = 0.0
obs_offset_y = 0.0
obs_offset_z = 0.0

# current state of quad
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0

#Original image in opencv
img = np.zeros((480,640,3), np.uint8)

#ros Images
raw_image = Image()
bin_image = Image()
circles_image = Image()
circle_avg_image = Image()
debug_image = Image()

#circle parameters
a= 0.0
cx = 0.0
cy = 0.0

#relative pose
pose_rel = Odometry()
pose_target_in = Odometry()

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
	# print('Image Height:')
	# print(height)
	# print('Image Width:')
	# print(width)


	#Convert from BGR to gray colorspace
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);

	gray_img = frame.copy()

	lower = (235) #lower threshhold values (H, S, V)
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
	global img, circles_image, circle_avg_image

	circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,1,1,
                            param1=100,param2=50,minRadius=0,maxRadius=0)

	a = []
	cx = []
	cy = []
	img1 = img.copy()
	if circles is not None:
		print(len(circles))
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(img1,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(img1,(i[0],i[1]),2,(0,0,255),3)

			a.append(i[2])
			cx.append(i[0])
			cy.append(i[1])


		a = np.average(a)
		cx = np.average(cx)
		cy = np.average(cy)

		img2 = img.copy()
		cv2.circle(img2,(cx.astype(int),cy.astype(int)),a.astype(int),(0,255,0),5)
		cv2.circle(img2,(cx.astype(int),cy.astype(int)),2,(0,0,255),5)

		circles_image = bridge.cv2_to_imgmsg(img1, "8UC3")
		circle_avg_image = bridge.cv2_to_imgmsg(img2, "8UC3")

		return True
	else:
		return False

def get_square(frame):
	global a, cx, cy
	global img, circles_image, circle_avg_image

	circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,1,1,
                            param1=100,param2=42,minRadius=0,maxRadius=0)

	a = []
	cx = []
	cy = []
	img1 = img.copy()
	if circles is not None:
		print(len(circles))
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(img1,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(img1,(i[0],i[1]),2,(0,0,255),3)

			a.append(i[2])
			cx.append(i[0])
			cy.append(i[1])


		a = np.average(a)
		cx = np.average(cx)
		cy = np.average(cy)

		img2 = img.copy()
		cv2.circle(img2,(cx.astype(int),cy.astype(int)),a.astype(int),(0,255,0),5)
		cv2.circle(img2,(cx.astype(int),cy.astype(int)),2,(0,0,255),5)

		circles_image = bridge.cv2_to_imgmsg(img1, "8UC3")
		circle_avg_image = bridge.cv2_to_imgmsg(img2, "8UC3")

		return True
	else:
		return False


def get_pose(R):
	global a, cx, cy
	#Image coordinates
	u1 = cx
	v1 = cy
	u2 = cx+a
	v2 = cy

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

# if window relative position and orientation are in camera/body frame
def pose_cam2in():
	global pose_rel, pose_target_in, x, y, z, yaw
	global pub_pose_target_in

	x_obj_rel_c = pose_rel.pose.pose.position.x
	y_obj_rel_c = pose_rel.pose.pose.position.y
	z_obj_rel_c = pose_rel.pose.pose.position.z

	# camera to body frame
	x_obj_rel_b = obs_factor_x * -y_obj_rel_c + obs_offset_x
	y_obj_rel_b = obs_factor_y * -x_obj_rel_c + obs_offset_y
	z_obj_rel_b = obs_factor_z * -z_obj_rel_c + obs_offset_z

	rospy.loginfo('x_b %f \t y_b %f \t z_b %f', x_obj_rel_b, y_obj_rel_b, z_obj_rel_b)

	# body to inertial frame rotation transform
	x_obj_rel_in = x_obj_rel_b*cos(yaw) - y_obj_rel_b*sin(yaw)
	y_obj_rel_in = x_obj_rel_b*sin(yaw) + y_obj_rel_b*cos(yaw)
	z_obj_rel_in = z_obj_rel_b

	# inertial frame shift transform
	x_obj = x + x_obj_rel_in
	y_obj = y + y_obj_rel_in
	z_obj = z + z_obj_rel_in

	pose_target_in.header.frame_id = "odom"
	pose_target_in.child_frame_id = "base_link"
	pose_target_in.header.stamp = rospy.get_rostime()
	pose_target_in.pose.pose.position.x = x_obj
	pose_target_in.pose.pose.position.y = y_obj
	pose_target_in.pose.pose.position.z = z_obj
	pose_target_in.twist.twist.linear.x = 0.0
	pose_target_in.twist.twist.linear.y = 0.0
	pose_target_in.twist.twist.linear.z = 0.0
	pose_target_in.pose.pose.orientation.w = 1.0
	pose_target_in.pose.pose.orientation.x = 0.0
	pose_target_in.pose.pose.orientation.y = 0.0
	pose_target_in.pose.pose.orientation.z = 0.0

	pub_pose_target_in.publish(pose_target_in)

	# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', x_obj, y_obj, z_obj, yaw_obj)
	# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', x_obj_rel_in, y_obj_rel_in, z_obj_rel_in, yaw_obj)

# pose of quad in inertial frame
def quad_pose(data):
	global x, y ,z, vx, vy ,vz, roll, pitch, yaw

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	vz = data.twist.twist.linear.z

	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

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

pub_pose_target_in = rospy.Publisher('/pose_target_in', Odometry, queue_size=10)
def main():
	global raw_image, bin_image, debug_image, pub_pose_rel, circles_image, circle_avg_image
	rospy.init_node('target_detect', anonymous=True)

	pub_pose_rel = rospy.Publisher('/pose_rel_target', Odometry, queue_size=10)
	pub_bin_image = rospy.Publisher('/bin_image', Image, queue_size=10)
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)
	pub_circles_image = rospy.Publisher('/circles_image', Image, queue_size=10)
	pub_circle_avg_image = rospy.Publisher('/circle_avg_image', Image, queue_size=10)

	rospy.Subscriber('/duo3d/left/image_rect', Image, callback) #should be faster than 20Hz or the rate of publishing below, else EKF might get fucked up
	rospy.Subscriber('/pose_in', Odometry, quad_pose)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		try:
			frame = thresholding()

			if (master_mission_no == 5):
				#Real world Info
				R = 0.1
				target_detected = get_circle(frame)

			if (master_mission_no == 8):
				#Real world Info
				R = 0.17
				target_detected = get_square(frame)

			if (target_detected):
				get_pose(R)
				pose_cam2in()
		except:
			rospy.loginfo('Some error ocurred... in target_detect.py')

		# frame = thresholding()
		# circle_detected = get_circle(frame)
		# if (circle_detected):
		# 	get_pose()

		pub_bin_image.publish(bin_image)
		pub_circles_image.publish(circles_image)
		pub_circle_avg_image.publish(circle_avg_image)
		pub_debug_image.publish(debug_image)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
