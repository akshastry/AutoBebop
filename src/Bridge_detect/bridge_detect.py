#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import sin, cos, atan2, asin, exp, sqrt
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import traceback

master_mission_no = 0

###############
# Init. Stuff #
###############
bridge = CvBridge()

# observation factors for camera (tuning param), should not be necessary if camera is properly calibrated and pnp is working
obs_factor_x = 1.8
obs_factor_y = 1.8
obs_factor_z = 1.0

obs_offset_x = -0.4
obs_offset_y = 0.0
obs_offset_z = 0.0

#bridge coordinates in body frame
cX_bridge_prev = cY_bridge_prev = xb_past_prev = yb_past_prev = xb_b4_prev = yb_b4_prev = theta_prev = 0.0

# current state of quad
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0

#Original image in opencv
img = np.zeros((480,640,3), np.uint8)

#ros Images
raw_image = Image()
frame = Image()
edges = Image()
dilatedEdges = Image()
bridgemask = Image()
corners = Image()
orig_frame = Image()
filledBridge = Image()

#relative pose
pose_rel = Odometry()
pose_bridge_in = Odometry()


#################
# FUNCTION LIST #
#################

def callback(image):
	global raw_image
	raw_image = image

# if bridge relative position and orientation are in camera/body frame
def pose_cam2in(theta):
	global pose_rel, pose_bridge_in, x, y, z, yaw
	global pub_pose_bridge_in
	#print('yaw = ',yaw)
	rotation_q = euler_to_quaternion(0.0,0.0,yaw - theta)

	x_obj_rel_c = pose_rel.pose.pose.position.x
	y_obj_rel_c = pose_rel.pose.pose.position.y
	z_obj_rel_c = pose_rel.pose.pose.position.z

	# camera to body frame
	x_obj_rel_b = obs_factor_x * -y_obj_rel_c + obs_offset_x
	y_obj_rel_b = obs_factor_y * -x_obj_rel_c + obs_offset_y
	z_obj_rel_b = obs_factor_z * -z_obj_rel_c + obs_offset_z

	# rospy.loginfo('x_b %f \t y_b %f \t z_b %f', x_obj_rel_b, y_obj_rel_b, z_obj_rel_b)

	# body to inertial frame rotation transform
	x_obj_rel_in =  x_obj_rel_b*cos(yaw) - y_obj_rel_b*sin(yaw)
	y_obj_rel_in = x_obj_rel_b*sin(yaw) + y_obj_rel_b*cos(yaw)
	z_obj_rel_in = z_obj_rel_b

	# inertial frame shift transform
	x_obj = x + x_obj_rel_in
	y_obj = y + y_obj_rel_in
	z_obj = z + z_obj_rel_in

	pose_bridge_in.header.frame_id = "odom"
	pose_bridge_in.child_frame_id = "base_link"
	pose_bridge_in.header.stamp = rospy.get_rostime()
	pose_bridge_in.pose.pose.position.x = x_obj
	pose_bridge_in.pose.pose.position.y = y_obj
	pose_bridge_in.pose.pose.position.z = z_obj
	pose_bridge_in.twist.twist.linear.x = 0.0
	pose_bridge_in.twist.twist.linear.y = 0.0
	pose_bridge_in.twist.twist.linear.z = 0.0
	pose_bridge_in.pose.pose.orientation.w = rotation_q[0]
	pose_bridge_in.pose.pose.orientation.x = rotation_q[1]
	pose_bridge_in.pose.pose.orientation.y = rotation_q[2]
	pose_bridge_in.pose.pose.orientation.z = rotation_q[3]

	pub_pose_bridge_in.publish(pose_bridge_in)

	# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', x_obj, y_obj, z_obj, yaw_obj)
	# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', x_obj_rel_in, y_obj_rel_in, z_obj_rel_in, yaw_obj)

def get_pose(cX_bridge,cY_bridge,theta):
	#rospy.loginfo('x %f \t y %f \t z %f', x1, y1, z)
	rotation_q = euler_to_quaternion(0.0,0.0,theta)
	pose_rel.header.frame_id = "odom"
	pose_rel.child_frame_id = "base_link"
	pose_rel.header.stamp = rospy.get_rostime()
	pose_rel.pose.pose.position.x = -cX_bridge
	pose_rel.pose.pose.position.y = -cY_bridge
	pose_rel.pose.pose.position.z = 1.25
	pose_rel.twist.twist.linear.x = 0.0
	pose_rel.twist.twist.linear.y = 0.0
	pose_rel.twist.twist.linear.z = 0.0
	pose_rel.pose.pose.orientation.w = rotation_q[0]
	pose_rel.pose.pose.orientation.x = rotation_q[1]
	pose_rel.pose.pose.orientation.y = rotation_q[2]
	pose_rel.pose.pose.orientation.z = rotation_q[3]
	pub_pose_rel.publish(pose_rel)

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

def euler_to_quaternion(roll, pitch, yaw):

    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    return [qw, qx, qy, qz]

###################################################
#NEW FUNCTIONS UNIQUE TO BRIDGE MISSION

def scaleImage(frame):
    global width, height
    #Resize image
    scale = 1
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height) #can also just specify desired dimensions
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    return frame

def getEdges(frame):
    #find edges in image
    frame = cv2.Canny(frame,0,150) #tune second parameter for more/less agressive detection
    return frame

def firstDilate(frame):
    #dilate/erode edges
    kernel = np.ones((5,5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)
    frame = cv2.bitwise_not(frame)
    # kernel = np.ones((12,12), np.uint8)
    # frame = cv2.erode(frame, kernel, iterations=1)
    # kernel = np.ones((25,25), np.uint8)
    # frame = cv2.dilate(frame, kernel, iterations=1)
    return frame

def featurelessThresh(frame):
    #simple threshhold on featureless areas; if featureless area is too dark, likely not bridge
	frame[np.where(frame == 255)] = orig_frame[np.where(frame == 255)]
    #threshhold for high brightness featurelss areas
	lower = 150 #lower threshhold value
	upper = 255 #upper threshhold value
	frame = cv2.inRange(frame, lower, upper)
   	#dilate/erode edges
	kernel = np.ones((2,2), np.uint8)
	frame = cv2.dilate(frame, kernel, iterations=1) 
	kernel = np.ones((3,3), np.uint8)
	frame = cv2.erode(frame, kernel, iterations=1)
	return frame

def fillBridge(frame,areas,idxs,cntsSorted):
    #draw largest contour and erode
    frame = cv2.drawContours(frame, [cntsSorted[-1]], 0, 255, thickness=cv2.FILLED)
    kernel = np.ones((5,5), np.uint8)
    frame = cv2.erode(frame, kernel, iterations=1)
    #get new eroded contour and fill concave bits
    _,contours,_ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    areas = np.array([cv2.contourArea(cnt) for cnt in contours])
    idxs  = areas.argsort()
    cntsSorted = [contours[i] for i in idxs]
    hull = cv2.convexHull(cntsSorted[-1])
    frame = np.zeros(shape=[height,width, 1], dtype=np.uint8)
    frame = cv2.drawContours(frame, [hull], 0, 255, thickness=cv2.FILLED)   
    return frame, hull, areas, idxs, cntsSorted

def fitPoly(frame,hull):
    #fit polygon
    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, 0.1*peri, True)
    frame = np.zeros(shape=[height,width, 1], dtype=np.uint8)
    frame = cv2.drawContours(frame, [approx], 0, 255, thickness=cv2.FILLED) 
    return frame

def getBridgeCenter(frame):
    #find 4 corners of bridge and sort them clockwise from top left corner
    corners = cv2.goodFeaturesToTrack(frame,4,0.01,80) #pick out 4 most prominant corners
    #draw circles
    for corner in corners:
        x,y = corner.ravel()
        frame = cv2.circle(frame,(x,y),25,255,-1)

    #Order corner points clockwise from top left (origin) 
    #first sort in ascending y values
    idxs  = corners[:,0,1].argsort()
    corner_sort_x = [corners[i,0,0] for i in idxs]
    corner_sort_y = [corners[i,0,1] for i in idxs]
    #now sort x values accordingly
    if len(idxs)>3:
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

    #find center and orientation of bridge
    cX_bridge = int(np.sum(corner_sort_x)/len(idxs))
    cY_bridge = int(np.sum(corner_sort_y)/len(idxs))
    center_bridge = (cX_bridge,cY_bridge)
    if len(idxs)>3:
	    d1 = math.sqrt((corner_sort_x[1] - corner_sort_x[0])**2 + (corner_sort_y[1] - corner_sort_y[0])**2)
	    d2 = math.sqrt((corner_sort_x[3] - corner_sort_x[0])**2 + (corner_sort_y[3] - corner_sort_y[0])**2)
	    d3 = math.sqrt((corner_sort_x[3] - corner_sort_x[2])**2 + (corner_sort_y[3] - corner_sort_y[2])**2)
	    d4 = math.sqrt((corner_sort_x[2] - corner_sort_x[1])**2 + (corner_sort_y[2] - corner_sort_y[1])**2)
    #check if bridge is at top or bottom of frame
    if np.sum(frame[0,:])>0:
    	slope = 1.7
    elif np.sum(frame[height-1,:])>0:
    	slope = 1.7
    #get slope normally if bridge is in frame normally
    else:
	    if d1 > d2: #horizontal
	        slope1 = (corner_sort_y[1] - corner_sort_y[0])/(corner_sort_x[1] - corner_sort_x[0])
	        slope2 = (corner_sort_y[3] - corner_sort_y[2])/(corner_sort_x[3] - corner_sort_x[2])
	        slope  = (slope1 + slope2)/2
	    if d1 < d2: #vertical
	        slope1 = (corner_sort_y[3] - corner_sort_y[0])/(corner_sort_x[3] - corner_sort_x[0])
	        slope2 = (corner_sort_y[2] - corner_sort_y[1])/(corner_sort_x[2] - corner_sort_x[1])
	        slope  = (slope1 + slope2)/2  
    return frame,cX_bridge,cY_bridge,slope

def getBridgeWaypoints(orig_frame,slope,cX_bridge,cY_bridge):
    #draw direction
    signX = 1.0
    signY = 1.0
    slope = slope*-1.0
    #every possible combination of bridge location in frame and its orientation needs a unique path direction
    #still needs work...sometimes fails for weird cases...need to do it based on slope not quadrant
    
    #quadrant 1
    if cY_bridge < height/2 and cX_bridge < width/2:
        if slope > 0:
            signX = 1.0
            signY = -1.0
        if slope < 0:
            signX = 1.0
            signY = -1.0
    #quadrant 2
    if cY_bridge < height/2 and cX_bridge > width/2:
        if slope > 0:
            signX = 1.0
            signY = -1.0
        if slope < 0:
            signX = -1.0
            signY = -1.0
    #quadrant 3
    if cY_bridge > height/2 and cX_bridge < width/2:
        if slope > 0:
            signX = -1.0
            signY = 1.0
        if slope < 0:
            signX = 1.0
            signY = 1.0  
    #quadrant 4
    if cY_bridge > height/2 and cX_bridge > width/2:
        if slope > 0:
            signX = -1.0
            signY = 1.0
        if slope < 0:
            signX = 1.0
            signY = 1.0      
    d = 150 #length of waypoint in pixels
    dx = math.sqrt(d**2/(slope**2 + 1))
    dy = math.sqrt(d**2/(1 + 1/slope**2))
    x_past = cX_bridge + signX*dx
    y_past = cY_bridge + signY*dy
    x_b4 = cX_bridge - signX*dx
    y_b4 = cY_bridge - signY*dy
    arrow = (int(x_past),int(y_past))
    center_bridge = (cX_bridge,cY_bridge)
    orig_frame = cv2.arrowedLine(orig_frame, center_bridge, arrow, (0),thickness=15, line_type=8, shift=0, tipLength=0.5)
    orig_frame = cv2.circle(orig_frame,center_bridge,25,(255,255,255),-1)  
    return orig_frame,x_past,y_past,x_b4,y_b4

def pixel2meters(cX_bridge,cY_bridge,x_past,y_past,x_b4,y_b4):
    scale = 2.1
    #make coordinates relative to center in body frame, scaled to meters

    #Camera Parameters
    fx = 300
    fy = 300
    cam_cx = width/2.0
    cam_cy = height/2.0

    cX_bridge = -(cX_bridge - width/2.0)/width*scale
    cY_bridge = -(cY_bridge - width/2.0)/width*scale
    xb_b4 = -(x_b4 - width/2.0)/width*scale
    yb_b4 = -(y_b4 - height/2.0)/height*scale
    xb_past = -(x_past - width/2.0)/width*scale
    yb_past = -(y_past - height/2.0)/height*scale
    return cX_bridge,cY_bridge,xb_past,yb_past,xb_b4,yb_b4 

def lowPassWaypoints(cX_bridge,cY_bridge,xb_past,yb_past,xb_b4,yb_b4,cX_bridge_prev,cY_bridge_prev,xb_past_prev,yb_past_prev,xb_b4_prev,yb_b4_prev):
    B = 0.5 #lp filter parameter
    cX_bridge = B*cX_bridge + (1.0-B)*cX_bridge_prev
    cY_bridge = B*cY_bridge + (1.0-B)*cY_bridge_prev
    xb_past = B*xb_past + (1.0-B)*xb_past_prev
    yb_past = B*yb_past + (1.0-B)*yb_past_prev
    xb_b4 = B*xb_b4 + (1.0-B)*xb_b4_prev
    yb_b4 = B*yb_b4 + (1.0-B)*yb_b4_prev
    return cX_bridge,cY_bridge,xb_past,yb_past,xb_b4,yb_b4

def lowPassTheta(theta,theta_prev,B):
	# B = 0.5 #lp filter parameter
	theta = B*theta + (1.0 - B)*theta_prev
	return theta


#############
# MAIN LOOP #
#############

def get_master_mission(data):
	global master_mission_no
	master_mission_no = data.data

pub_pose_bridge_in = rospy.Publisher('/pose_bridge_in', Odometry, queue_size=10)
def main():
	global edges, dilatedEdges, bridgemask, filledBridge, corners, orig_frame, pub_pose_rel
	rospy.init_node('bridge_detect', anonymous=True)

	pub_pose_rel = rospy.Publisher('/pose_rel_bridge', Odometry, queue_size=10)
	pub_edges_image = rospy.Publisher('/edges_image', Image, queue_size=10)
	pub_dilatedEdges_image = rospy.Publisher('/dilatedEdges_image', Image, queue_size=10)
	pub_bridgemask_image = rospy.Publisher('/bridgemask_image', Image, queue_size=10)
	pub_filledBridge_image = rospy.Publisher('/filledBridge_image', Image, queue_size=10)
	pub_corners_image = rospy.Publisher('/corners_image', Image, queue_size=10)
	pub_orig_frame_image = rospy.Publisher('/orig_frame_image', Image, queue_size=10)

	rospy.Subscriber('/duo3d/left/image_rect', Image, callback) #should be faster than 20Hz or the rate of publishing below, else EKF might get fucked up
	rospy.Subscriber('/pose_in', Odometry, quad_pose)
	rospy.Subscriber('/master_mission_no', Int32, get_master_mission)

	cX_bridge_prev = cY_bridge_prev = xb_past_prev = yb_past_prev = xb_b4_prev = yb_b4_prev = theta_prev = 0.0

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		if (master_mission_no == 2):

			try: 
				#get image
				try:
					frame = bridge.imgmsg_to_cv2(raw_image, "bgr8")
				except CvBridgeError as e:
					print(e)

				frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				#scale image if needed
				frame = scaleImage(frame)
				#frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
				# frame = cv2.flip(frame,1)
				orig_frame = np.copy(frame)

				#get edges of image
				frame = getEdges(frame)
				edges = np.copy(frame) #for plotting and troubleshooting, comment out in final implimentation
				edges = bridge.cv2_to_imgmsg(edges, "8UC1")
				pub_edges_image.publish(edges)

				#dilate edges to fill in featured areas
				frame = firstDilate(frame)
				dilatedEdges = np.copy(frame) #for plotting and troubleshooting, comment out in final implimentation
				dilatedEdges = bridge.cv2_to_imgmsg(dilatedEdges, "8UC1")
				pub_dilatedEdges_image.publish(dilatedEdges)

				#of featurelss areas, take only areas of high brightness
				frame = featurelessThresh(frame)
				bridgemask = np.copy(frame) #for plotting and troubleshooting, comment out in final implimentation   
				bridgemask = bridge.cv2_to_imgmsg(bridgemask, "8UC1")
				pub_bridgemask_image.publish(bridgemask)

				#do some stuff if we detect something that could be a bridge
				#find all contours of mask
				_,contours,_ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				frame = np.zeros(shape=[height,width, 1], dtype=np.uint8)
				max_contour_ratio = .02  #change based on relative size of bridge in frame 

				if len(contours) > 0:
				    #sort the contours by area
				    areas = np.array([cv2.contourArea(cnt) for cnt in contours])
				    idxs  = areas.argsort()
				    cntsSorted = [contours[i] for i in idxs]
				    
				    #if biggest contour is big enough to be a bridge, do more shit
				    if areas[idxs[-1]] > max_contour_ratio*height*width: 
				        #fill all concave corners of biggest bridge contour
				        frame, hull, areas, idxs, cntsSorted = fillBridge(frame,areas,idxs,cntsSorted)
				        filledBridge = np.copy(frame) #for plotting and troubleshooting, comment out in final implimentation
				        filledBridge = bridge.cv2_to_imgmsg(filledBridge, "8UC1")
				        pub_filledBridge_image.publish(filledBridge)

				        #fit polygon to bridge
				        frame = fitPoly(frame,hull)
				        fittedRectangle = np.copy(frame) #for plotting and troubleshooting, comment out in final implimentation
				        
				        #get center of bridge and its slope
				        corners,cX_bridge,cY_bridge,slope = getBridgeCenter(frame) #corners is image of 4 found corners of bridge, not needed
				        corners = bridge.cv2_to_imgmsg(np.asarray(corners), "8UC1")
				        pub_corners_image.publish(corners)

				        #get waypoint before and on other side of bridge
				        orig_frame,x_past,y_past,x_b4,y_b4 = getBridgeWaypoints(orig_frame,slope,cX_bridge,cY_bridge) #orig_frame is original image with detected center and direction
				        
				        #convert pixel coordinates of two waypoints to displacements w.r.t. body frame
				        cX_bridge,cY_bridge,xb_past,yb_past,xb_b4,yb_b4 = pixel2meters(cX_bridge,cY_bridge,x_past,y_past,x_b4,y_b4 )
				        
				        #lowpass filter waypoint coordinates
				        cX_bridge,cY_bridge,xb_past,yb_past,xb_b4,yb_b4 = lowPassWaypoints(cX_bridge,cY_bridge,xb_past,yb_past,xb_b4,yb_b4,cX_bridge_prev,cY_bridge_prev,xb_past_prev,yb_past_prev,xb_b4_prev,yb_b4_prev)
				        #store previous values 
				        cX_bridge_prev = cX_bridge
				        cY_bridge_prev = cY_bridge
				        xb_past_prev = xb_past
				        yb_past_prev = yb_past
				        xb_b4_prev = xb_b4
				        yb_b4_prev =yb_b4

				        #calculate angle
				        theta = math.atan2((xb_past - xb_b4),(yb_past-yb_b4)) #radians
				        theta = lowPassTheta(theta,theta_prev,.4)
				        theta_prev = theta

				        #pose
				        get_pose(cX_bridge,cY_bridge,theta)
				        pose_cam2in(theta)     

				orig_frame2 = bridge.cv2_to_imgmsg(orig_frame, "8UC1")
				pub_orig_frame_image.publish(orig_frame2)


			except Exception:
				traceback.print_exc()
				# rospy.loginfo('Some error ocurred... in bridge_detect.py')

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass