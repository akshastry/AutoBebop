#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import sin, cos, atan2, asin
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import hdbscan

pose_rel = Odometry()

img = np.zeros((480,640,3), np.uint8)
raw_image = Image()
bin_image = Image()
contour_image = Image()
line_image = Image()
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
	img_lines_bin = np.zeros(shape=[height, width, 1], dtype=np.uint8) 
	img_corners = np.zeros(shape=[height, width, 1], dtype=np.uint8)

	#Convert from BGR to HSV colorspace
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);

	#Guassian blur
	blur_params = (5,5)
	frame = cv2.GaussianBlur(frame,blur_params,cv2.BORDER_DEFAULT)

	#yellow
	lower = (0, 70, 40) #lower threshhold values (H, S, V)
	upper = (90, 255, 255) #upper threshhold values (H, S, V)
	frame = cv2.inRange(frame, lower, upper)

	#Erosion/dilation
	kernel = np.ones((2,2), np.uint8) 
	frame = cv2.erode(frame, kernel, iterations=1)
	kernel = np.ones((2,3), np.uint8) 
	frame = cv2.dilate(frame, kernel, iterations=1) 

	bin_image = bridge.cv2_to_imgmsg(frame, "8UC1")

def get_contour():
	global contour_image
	global frame 
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

	if (len(contours)>0):
		#convex_hull
		contours[0] = cv2.convexHull(contours[0])
		#approximate the contour
		epsilon = 0.05*cv2.arcLength(contours[0],True)
		contours[0] = cv2.approxPolyDP(contours[0],epsilon,True)
		#draw the contour
		frame = cv2.drawContours(frame, [contours[0]], 0, 255, thickness=1)

		#remove contour lines on borders
		e=20; v=0;
		cv2.line(img_corners, (0,0), (0,height), (v), e, cv2.LINE_AA)
		cv2.line(img_corners, (0,0), (width,0), (v), e, cv2.LINE_AA)
		cv2.line(img_corners, (width,0), (width,height), (v), e, cv2.LINE_AA)
		cv2.line(img_corners, (0,height), (width,height), (v), e, cv2.LINE_AA)

	contour_image = bridge.cv2_to_imgmsg(frame, "8UC1")

def get_lines():
	global line_image, debug_image
	global frame, img_lines_bin, img

	#Find lines on binary image     
	lines = cv2.HoughLines(frame, 0.5, np.pi/180,  threshold=15)  #ADJUST   

	# if lines is not None:
		# rospy.loginfo('no of lines %f',len(lines))

	#Draw lines on blank binary image
	if lines is not None:
	    for i in range(0, len(lines)):
	        rho = lines[i][0][0]
	        theta = lines[i][0][1]
	        a = cos(theta)
	        b = sin(theta)
	        x0 = a * rho
	        y0 = b * rho
	        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
	        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
	        cv2.line(img, pt1, pt2, (0,255,0), 1, cv2.LINE_AA)
	        cv2.line(img_lines_bin, pt1, pt2, (255,255,255), 1, cv2.LINE_AA) 

	#Dilate lines image
	# kernel = np.ones((7,7), np.uint8)
	# img_lines_bin = cv2.dilate(img_lines_bin,kernel,iterations=1)

	line_image = bridge.cv2_to_imgmsg(img_lines_bin, "8UC1")
	# line_image = bridge.cv2_to_imgmsg(img, "8UC3")

	# debug_image = bridge.cv2_to_imgmsg(img, "8UC3")


def corner_detection():
	global corner_image, debug_image
	global img_lines_bin, img_corners, img_centroids
	global height, width

	#Corner detection
	corners = cv2.cornerHarris(img_lines_bin,2,3,0.04)
	img_corners[corners>0.1*corners.max()] = 255

	#remove corners on borders
	e=20; v=0;
	cv2.line(img_corners, (0,0), (0,height), (v), e, cv2.LINE_AA)
	cv2.line(img_corners, (0,0), (width,0), (v), e, cv2.LINE_AA)
	cv2.line(img_corners, (width,0), (width,height), (v), e, cv2.LINE_AA)
	cv2.line(img_corners, (0,height), (width,height), (v), e, cv2.LINE_AA)

	debug_image = bridge.cv2_to_imgmsg(img_corners, "8UC1")

	#get the points
	ret,img_corners = cv2.threshold(img_corners,128,255,cv2.THRESH_BINARY)
	corners_values = np.nonzero(img_corners)

	#HDB Clustering
	if (len(corners_values[0])>1):
	    X = corners_values[0]
	    Y = corners_values[1]
	    Z = np.array([X[0],Y[0]])
	    for i in range(np.size(X)-1):
	        Z = np.vstack((Z,np.array([X[i+1],Y[i+1]])))

	    clusterer = hdbscan.HDBSCAN(min_cluster_size=5)
	    cluster_labels = clusterer.fit_predict(Z)
	    Zp = cluster_labels
	    
	    #If all =-1 assume Only one cluster 
	    n_cluster = len(np.unique(Zp))
	    if (n_cluster == 1):
	        Zp = np.zeros(len(Zp))

	    #remove -1 or unclustered point
	    unclass_ind = []
	    for i in range(len(Zp)):
	        if(Zp[i] == -1):
	            unclass_ind.append(i)

	    Zp = np.delete(Zp,unclass_ind,0)
	    X = np.delete(X,unclass_ind,0)
	    Y = np.delete(Y,unclass_ind,0)
	    
	    n_cluster = len(np.unique(Zp))
	    cluster_mean = np.zeros((n_cluster,2))
	    
	    for i in range(n_cluster):
	        n_el = 0
	        for j in range(len(Zp)):
	            if (Zp[j] == i):
	                n_el = n_el + 1
	                cluster_mean[i,0] = cluster_mean[i,0] + Z[j,0]
	                cluster_mean[i,1] = cluster_mean[i,1] + Z[j,1]

	        cluster_mean[i,0] = cluster_mean[i,0] / n_el
	        cluster_mean[i,1] = cluster_mean[i,1] / n_el
	        
	    cluster_avg_dist = np.zeros((n_cluster))
	    cluster_var = np.zeros((n_cluster,2))
	    
	    for i in range(n_cluster):
	        n_el = 0
	        for j in range(len(Zp)):
	            if (Zp[j] == i):
	                n_el = n_el + 1                
	                cluster_avg_dist[i] = cluster_avg_dist[i] +  (Z[j,0] - cluster_mean[i,0])**2 + (Z[j,0] - cluster_mean[i,0])**2
	                cluster_var[i,0] = cluster_var[i,0] + (Z[j,0] - cluster_mean[i,0] )**2
	                cluster_var[i,1] = cluster_var[i,1] + (Z[j,1] - cluster_mean[i,1] )**2
	                
	        cluster_avg_dist[i] = cluster_avg_dist[i] / n_el
	        cluster_var[i,0] = cluster_var[i,0] / n_el
	        cluster_var[i,1] = cluster_var[i,1] / n_el
	        
	    #sorting
	    for i in range(4):
	        for j in range(len(cluster_avg_dist)-1):
	            if(cluster_avg_dist[j] < cluster_avg_dist[j+1]):
	                temp = cluster_avg_dist[j]
	                cluster_avg_dist[j] = cluster_avg_dist[j+1]
	                cluster_avg_dist[j+1] = temp
	                
	                temp = cluster_mean[j,0]
	                cluster_mean[j,0] = cluster_mean[j+1,0]
	                cluster_mean[j+1,0] = temp
	                
	                temp = cluster_mean[j,1]
	                cluster_mean[j,1] = cluster_mean[j+1,1]
	                cluster_mean[j+1,1] = temp
	    
	    cluster_mean = cluster_mean[-4:,:]        
	            
	elif (len(corners_values[0])==1):
		cluster_mean = corners_values
	else:
	    cluster_mean=[]

	#Overlay final corners on original image
	img_centroids = img_orig.copy()
	for i in range(len(cluster_mean)):
		center = (cluster_mean[i,1].astype(int),cluster_mean[i,0].astype(int))
		cv2.circle(img_centroids, center, 2, [0,255,0], 5)

	corner_image = bridge.cv2_to_imgmsg(img_centroids, "8UC3")
	return cluster_mean

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
	global img_centroids, pose_rel, pose_image
	global camera_matrix, dist_coeffs, image_points, model_points_yellow
		#re-project line onto each corner to see 3D orientation found by solvePnP
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

	pub_pose_rel = rospy.Publisher('/pose_rel', Odometry, queue_size=10)
	pub_bin_image = rospy.Publisher('/bin_image', Image, queue_size=10)
	pub_contour_image = rospy.Publisher('/contour_image', Image, queue_size=10)
	pub_line_image = rospy.Publisher('/line_image', Image, queue_size=10)
	pub_corner_image = rospy.Publisher('/corner_image', Image, queue_size=10)
	pub_pose_image = rospy.Publisher('/pose_image', Image, queue_size=10)
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)

	# rospy.Subscriber('/cv_camera/image_raw', Image, callback)
	# rospy.Subscriber('/image_raw', Image, callback)
	rospy.Subscriber('/image_raw_throttle', Image, callback)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		try:
			thresholding()
			get_contour()
			get_lines()
			corners = corner_detection()
			pose_solve(corners)
			pose_display(corners)
		except:
			rospy.loginfo('Some error ocurred')
		# thresholding()
		# get_contour()
		# get_lines()
		# corners = corner_detection()
		# pose_solve(corners)
		# pose_display(corners)
		pub_bin_image.publish(bin_image)
		pub_contour_image.publish(contour_image)
		pub_line_image.publish(line_image)
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
