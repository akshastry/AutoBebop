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
import hdbscan

img = np.zeros((480,640,3), np.uint8)
raw_image = Image()
pose_rel = Odometry()
bin_image = Image()
contour_image = Image()
line_image = Image()
corner_image = Image()
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
	frame = cv2.dilate(frame, kernel, iterations=1) 

	bin_image = bridge.cv2_to_imgmsg(frame, "8UC1")

def get_contour():
	global contour_image
	global frame 
	global height, width

	#Find contours and save only the biggest one
	_,contours,_ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	frame = np.zeros(shape=[height, width, 1], dtype=np.uint8) 
	areas = np.array([cv2.contourArea(cnt) for cnt in contours])
	idxs  = areas.argsort()
	cntsSorted = [contours[i] for i in idxs]

	# rospy.loginfo('cntsSorted %f',len(cntsSorted))
	if (len(cntsSorted)>=1):
		frame = cv2.drawContours(frame, [cntsSorted[-1]], 0, 255, thickness=cv2.FILLED)

	#Erosion/dilation on biggest contour binary
	kernel = np.ones((4,4), np.uint8) 
	frame = cv2.erode(frame, kernel, iterations=1)
	frame = cv2.dilate(frame, kernel, iterations=1)

	#Find edges of contour
	_,contours,_ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	frame = np.zeros(shape=[height,width, 1], dtype=np.uint8) 

	if (len(contours)>=1):
		frame = cv2.drawContours(frame, [contours[0]], 0, 255, thickness=1)

	contour_image = bridge.cv2_to_imgmsg(frame, "8UC1")

def get_lines():
	global line_image
	global frame, img_lines_bin

	#Find lines on binary image     
	lines = cv2.HoughLines(frame, 0.5, np.pi / 180,  threshold=100)  #ADJUST   

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
	        cv2.line(img_lines_bin, pt1, pt2, (255,255,255), 2, cv2.LINE_AA) 

	#Dilate lines image
	kernel = np.ones((10,10), np.uint8)
	img_lines_bin = cv2.dilate(img_lines_bin,kernel,iterations=1)

	line_image = bridge.cv2_to_imgmsg(img_lines_bin, "8UC1")


def corner_detection():
	global corner_image
	global img_lines_bin, img_corners

	#Corner detection
	corners = cv2.cornerHarris(img_lines_bin,2,3,0.04)
	img_corners[corners>0.1*corners.max()] = 255

	#get the points
	ret,img_corners = cv2.threshold(img_corners,128,255,cv2.THRESH_BINARY)
	corners_values = np.nonzero(img_corners)

	#HDB Clustering
	if (len(corners_values[0])>0):
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
	            
	else:
	    Zp=[]
	    cluster_mean=[]

	#Overlay final corners on original image
	img_centroids = img_orig.copy()
	for i in range(len(cluster_mean)):
	    center = (cluster_mean[i,1].astype(int),cluster_mean[i,0].astype(int))
	    cv2.circle(img_centroids, center, 2, [0,255,0], 5)

	corner_image = bridge.cv2_to_imgmsg(img_centroids, "8UC3")
	return cluster_mean

def pose_solve(corners):
	global pose_rel
	pose_rel = pose_rel

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
	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)

	rospy.Subscriber('/cv_camera/image_raw', Image, callback)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		thresholding()
		get_lines()
		get_contour()
		corners = corner_detection()
		pose_solve(corners)
		pub_bin_image.publish(bin_image)
		pub_contour_image.publish(contour_image)
		pub_line_image.publish(line_image)
		pub_corner_image.publish(corner_image)
		pub_debug_image.publish(debug_image)
		pub_pose_rel.publish(pose_rel)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
