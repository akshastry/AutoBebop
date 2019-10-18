#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import sin, cos, atan2, asin, exp, sqrt
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

pose_rel = Odometry()

img = np.zeros((480,640,3), np.uint8)
raw_image = Image()
bin_image = Image()
contour_image = Image()
corner_image = Image()
pose_image = Image()
debug_image = Image()

bridge = CvBridge()

def thresholding():
	global raw_image, bin_image
	global img, frame, img_orig, blank_image, img_lines_bin, img_corners
	global height, width

	try:
		img = bridge.imgmsg_to_cv2(raw_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	#Resize image
	scale = .4
	width = int(img.shape[1] * scale)
	height = int(img.shape[0] * scale)
	dim = (width, height) #can also just specify desired dimensions
	frame = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

	img = frame.copy()
	img_orig = frame.copy()
	blank_image = np.zeros(shape=[height, width, 3], dtype=np.uint8)

	#Convert from BGR to HSV colorspace
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);

	#Guassian blur
	blur_params = (5,5)
	frame = cv2.GaussianBlur(frame,blur_params,cv2.BORDER_DEFAULT)

	#yellow
	lower = (0, 90, 70) #lower threshhold values (H, S, V)
	upper = (90, 255, 255) #upper threshhold values (H, S, V)
	frame = cv2.inRange(frame, lower, upper)

	# ##with single G
	# #yellow
	# P_thres = 2

	# frameb = np.zeros(shape=[height,width, 1], dtype=np.uint8)
	# mu = np.array([ 0.0889, 0.4376, 0.7321])
	# sigma = np.array([[0.0010, 0.0012, 0.0047],[0.0012, 0.0040, 0.0060],[0.0047, 0.0060, 0.0252]])
	# sigma_inv = np.linalg.inv(sigma)
	# DEN = ((2*3.14)**(3/2))*sqrt(np.linalg.det(sigma))
	# for i in range(height):
	#     for j in range(width):
	#         x = frame[i][j]/255.0
	#         v = -0.5*np.dot((x-mu),np.matmul(sigma_inv,(x-mu)))
	#         NUM = exp(v)
	#         P = NUM/DEN
	#         if (P>P_thres):
	#             frameb[i][j] = 1
	#         else:
	#             frameb[i][j] = 0
	# frame = frameb

	#Erosion/dilation
	kernel = np.ones((2,2), np.uint8) 
	frame = cv2.erode(frame, kernel, iterations=1)
	kernel = np.ones((3,3), np.uint8) 
	frame = cv2.dilate(frame, kernel, iterations=1) 

	bin_image = bridge.cv2_to_imgmsg(frame, "8UC1")

def get_corners():
	global contour_image, corner_image
	global frame, img_centroids
	global height, width

	#Find contours and save only the biggest one
	_,contours,_ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	frame = np.zeros(shape=[height,width, 1], dtype=np.uint8) 
	if len(contours) > 0:
	    areas = np.array([cv2.contourArea(cnt) for cnt in contours])
	    idxs  = areas.argsort()
	    cntsSorted = [contours[i] for i in idxs]
	    # if areas[idxs[-1]] > 0.0*height*width: #threshold for min contour area
	    frame = cv2.drawContours(frame, [cntsSorted[-1]], 0, 255, thickness=cv2.FILLED)

	#Erosion/dilation on biggest contour binary
	kernel = np.ones((4,4), np.uint8) 
	frame = cv2.erode(frame, kernel, iterations=1)
	kernel = np.ones((4,4), np.uint8)
	frame = cv2.dilate(frame, kernel, iterations=1)

	#Find edges of contour
	_,contours,_ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	
	frame = np.zeros(shape=[height,width, 1], dtype=np.uint8)

	# rospy.loginfo('no of contours %f',len(contours))
	corners = []
	if (len(contours)>0):
		# # approximate the contour
		epsilon = 0.04*cv2.arcLength(contours[0],True)
		contours[0] = cv2.approxPolyDP(contours[0],epsilon,True)
		# convex_hull
		contours[0] = cv2.convexHull(contours[0])
		#approximate the contour
		epsilon = 0.04*cv2.arcLength(contours[0],True)
		contours[0] = cv2.approxPolyDP(contours[0],epsilon,True)
		if (len(contours[0])<5):
			# draw the contour
			frame = cv2.drawContours(frame, [contours[0]], 0, 255, thickness=1)

			if (cv2.arcLength(contours[0], True)>50):
				cluster_mean = contours[0]
				for i in range(len(cluster_mean)):
					e=5
					if (cluster_mean[i][0][0]>e and cluster_mean[i][0][0]<width-e and cluster_mean[i][0][1]>e and cluster_mean[i][0][1]<height-e ):
						corners.append([cluster_mean[i][0][1], cluster_mean[i][0][0]])
	
	corners = np.asarray(corners)

	#Overlay final corners on original image
	img_centroids = img_orig.copy()
	for i in range(len(corners)):
		center = (corners[i][1],corners[i][0])
		cv2.circle(img_centroids, center, 2, [0,255,0], 5)

	contour_image = bridge.cv2_to_imgmsg(frame, "8UC1")
	corner_image = bridge.cv2_to_imgmsg(img_centroids, "8UC3")

	return corners

def pose_solve(cluster_mean):
	global pose_rel
	global camera_matrix, dist_coeffs, image_points, model_points_yellow
	if len(cluster_mean)>3:
		#Order corner points clockwise from top left (origin) 
		#first sort in ascending y values
		idxs  = cluster_mean[:,0].argsort()
		corner_sort_y = [cluster_mean[i,0] for i in idxs]
		corner_sort_x = [cluster_mean[i,1] for i in idxs]
		#now sort x values accordingly
		if corner_sort_x[0]>corner_sort_x[1]:
			hold_x = corner_sort_x[0]
			hold_y = corner_sort_y[0]
			corner_sort_x[0] = corner_sort_x[1]
			corner_sort_x[1] = hold_x
			corner_sort_y[0] = corner_sort_y[1]
			corner_sort_y[1] = hold_y
		if corner_sort_x[3]>corner_sort_x[2]:
			hold_x = corner_sort_x[2]
			hold_y = corner_sort_y[2]
			corner_sort_x[2] = corner_sort_x[3]
			corner_sort_x[3] = hold_x
			corner_sort_y[2] = corner_sort_y[3]
			corner_sort_y[3] = hold_y     
	
		image_points = np.array([   #(x,y)
									(corner_sort_x[0], corner_sort_y[0]),
									(corner_sort_x[1], corner_sort_y[1]),
									(corner_sort_x[2], corner_sort_y[2]),
									(corner_sort_x[3], corner_sort_y[3]),
								], dtype="double")
		model_points_yellow = np.array([ 
										(0.0, 0.0, 0.0),    #TOP LEFT CORNER IS ORIGIN (x,y,z)
										(.84, 0.0, 0.0),
										(.81, -.43, 0.0),
										(.03, -.43, 0.0),
										])

	    #model_points_purple = np.array([
	    #                                (0,0,0),
	    #                                (.82,0,0),
	    #                                (.82,0,.43),
	    #                                (0,0,.43),
	    #                        ])

		focal_length_x = 706.219794 #get from camera calibration
		focal_length_y = 709.519037 #get from camera calibration
		size = frame.shape
		center = (size[1]/2, size[0]/2)
		camera_matrix = np.array(
								[[focal_length_x, 0, center[0]],
								[0, focal_length_y, center[1]],
								[0, 0, 1]], dtype = "double"
								)
		dist_coeffs = np.array([-0.336810, 0.116976, -0.004822, -0.002968, 0.0]) #get from camera calibration

		(success, rotation_pnp, translation_pnp) = cv2.solvePnP(model_points_yellow, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
		#returns a rotation and translation matrix of the extrinsic matrix of the camera 
	    #i.e. rotation of the camera relative to fixed world origin (top left corner of window)
		rotation_pnp_q = euler_to_quaternion(rotation_pnp[0],rotation_pnp[1],rotation_pnp[2])
		pose_rel.header.frame_id = "odom"
		pose_rel.child_frame_id = "base_link"
		pose_rel.header.stamp = rospy.get_rostime()
		pose_rel.pose.pose.position.x = translation_pnp[0]
		pose_rel.pose.pose.position.y = translation_pnp[1]
		pose_rel.pose.pose.position.z = translation_pnp[2]
		pose_rel.twist.twist.linear.x = 0.0
		pose_rel.twist.twist.linear.y = 0.0
		pose_rel.twist.twist.linear.z = 0.0
		pose_rel.pose.pose.orientation.w = rotation_pnp_q[0]
		pose_rel.pose.pose.orientation.x = rotation_pnp_q[1]
		pose_rel.pose.pose.orientation.y = rotation_pnp_q[2]
		pose_rel.pose.pose.orientation.z = rotation_pnp_q[3]

def euler_to_quaternion(roll, pitch, yaw):

        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)

        return [qw, qx, qy, qz]

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

def pose_display(cluster_mean):
	global img_orig, pose_rel, pose_image, img_centroids
	global camera_matrix, dist_coeffs, image_points, model_points_yellow
		#re-project line onto each corner to see 3D orientation found by solvePnP
	# img_centroids = img_orig.copy()
	if len(cluster_mean)>3: 
		translation_pnp = np.array([pose_rel.pose.pose.position.x,pose_rel.pose.pose.position.y,pose_rel.pose.pose.position.z])
		rotation_pnp = np.array(quaternion_to_euler(pose_rel.pose.pose.orientation.w, pose_rel.pose.pose.orientation.x, pose_rel.pose.pose.orientation.y, pose_rel.pose.pose.orientation.z))
		#project a line of length l_test on each corner corner NEED TO ORDER THE CLUSTER_MEAN ARRAY
		l_test = .5
		(gate_origin, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, l_test)]), rotation_pnp, translation_pnp, camera_matrix, dist_coeffs)
		p1 = ( int(image_points[0][0]), int(image_points[0][1]))
		p2 = ( int(gate_origin[0][0][0]), int(gate_origin[0][0][1]))
		img_centroids = cv2.line(img_centroids, p1, p2, (0,255,0), 2)
		
		(gate_origin, jacobian) = cv2.projectPoints(np.array([(model_points_yellow[1][0],model_points_yellow[1][1], l_test)]), rotation_pnp, translation_pnp, camera_matrix, dist_coeffs)
		p1 = ( int(image_points[1][0]), int(image_points[1][1]))
		p2 = ( int(gate_origin[0][0][0]), int(gate_origin[0][0][1]))
		img_centroids = cv2.line(img_centroids, p1, p2, (0,255,0), 2)
		
		(gate_origin, jacobian) = cv2.projectPoints(np.array([(model_points_yellow[2][0],model_points_yellow[2][1], l_test)]), rotation_pnp, translation_pnp, camera_matrix, dist_coeffs)
		p1 = ( int(image_points[2][0]), int(image_points[2][1]))
		p2 = ( int(gate_origin[0][0][0]), int(gate_origin[0][0][1]))
		img_centroids = cv2.line(img_centroids, p1, p2, (0,255,0), 2)
	
		(gate_origin, jacobian) = cv2.projectPoints(np.array([(model_points_yellow[3][0],model_points_yellow[3][1], l_test)]), rotation_pnp, translation_pnp, camera_matrix, dist_coeffs)
		p1 = ( int(image_points[3][0]), int(image_points[3][1]))
		p2 = ( int(gate_origin[0][0][0]), int(gate_origin[0][0][1]))
		img_centroids = cv2.line(img_centroids, p1, p2, (0,255,0), 2)
		
	pose_image = bridge.cv2_to_imgmsg(img_centroids, "8UC3")

def callback(image):
	global raw_image
	raw_image = image

def main():
	global raw_image, bin_image, debug_image, pose_rel
	rospy.init_node('window_detect', anonymous=True)

	pub_pose_rel = rospy.Publisher('/pose_rel_win', Odometry, queue_size=10)
	pub_bin_image = rospy.Publisher('/bin_image', Image, queue_size=10)
	pub_contour_image = rospy.Publisher('/contour_image', Image, queue_size=10)
	pub_corner_image = rospy.Publisher('/corner_image', Image, queue_size=10)
	pub_pose_image = rospy.Publisher('/pose_image', Image, queue_size=10)
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)

	# rospy.Subscriber('/cv_camera/image_raw', Image, callback)
	rospy.Subscriber('/image_raw', Image, callback)
	# rospy.Subscriber('/image_raw_throttle', Image, callback)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		try:
			thresholding()
			corners = get_corners()
			pose_solve(corners)
			pose_display(corners)
		except:
			rospy.loginfo('Some error ocurred')

		# thresholding()
		# corners = get_corners()
		# pose_solve(corners)
		# pose_display(corners)

		pub_bin_image.publish(bin_image)
		pub_contour_image.publish(contour_image)
		pub_corner_image.publish(corner_image)
		pub_pose_image.publish(pose_image)
		pub_debug_image.publish(debug_image)
		pub_pose_rel.publish(pose_rel)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
