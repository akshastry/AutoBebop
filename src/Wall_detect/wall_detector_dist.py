#!/usr/bin/env python

import rospy, time, cv2
import numpy as np
from math import sin, cos, atan2, asin, exp, sqrt
from matplotlib import pyplot as plt
from std_msgs.msg import String, Float64, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm, sinm, cosm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

bridge = CvBridge()

# camera Parameters
fx = 743.595409
fy = 750.175831
cx = 800*0.5
cy = 460*0.5
# cx = 357.244818
# cy = 192.270976


## initialization of body to inertial rotation matrix
r_in_b = R.from_euler('zyx', [0, 0, 0], degrees=True)

# current state of quad
pos = np.array([0.0, 0.0, 0.0])
vel = np.array([0.0, 0.0, 0.0])
quat = np.array([0.0, 0.0, 0.0, 1.0])

vel_prev = np.array([0.0, 0.0, 0.0])
pos_prev = np.array([0.0, 0.0, 0.0])

# time stuff
dt_L = 0.0
t_old = 0.0

#Original image in opencv
img = np.zeros((480,640,3), np.uint8)
img_prev = np.zeros((480,640,3), np.uint8)

#ros Images
image = Image()
prev_image = Image()
featured_image = Image()
temporally_matched_featured_image = Image()
flow_image = Image()
wall_image = Image()
debug_image = Image()

#inertial pose of the wall
pose_wall_in = Odometry()

#velocity from VO
vel_VO = Twist()

def pose_estimation(dist):
	global f, B, cx, cy
	global image, prev_image, img, img_prev
	global featured_image
	global img, dt
	global temporally_matched_featured_image, flow_image, wall_image
	global height, width, scale
	global t_old, pos, quat, r_in_b
	global pose_wall_in

	# frame_prev = frame
	# frame_R_prev = frame_R

	try:
		img = bridge.imgmsg_to_cv2(image, "bgr8")
	except CvBridgeError as e:
		print(e)

	try:
		img_prev = bridge.imgmsg_to_cv2(prev_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	img_height = len(img)
	img_width = len(img[0])
	# print(img_height)
	# print(img_width)

	#Resize image
	scale = 1
	width = int(img.shape[1] * scale)
	height = int(img.shape[0] * scale)
	dim = (width, height) #can also just specify desired dimensions
	frame = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
	frame_prev = cv2.resize(img_prev, dim, interpolation = cv2.INTER_AREA)

	#Convert from BGR to gray colorspace
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
	frame_prev = cv2.cvtColor(frame_prev, cv2.COLOR_BGR2GRAY);

	img1 = frame.copy()
	img2 = frame_prev.copy()
	featured_img = img.copy()
	flow_img = img.copy()
	wall_img = img.copy()

	# Initiate ORB detector
	orb = cv2.ORB_create()

	# find the keypoints and descriptors with ORB
	kp1, des1 = orb.detectAndCompute(img1,None)
	kp2, des2 = orb.detectAndCompute(img2,None)

	# create BFMatcher object
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

	# Initialize lists
	list_u = []
	list_v = []
	list_X_t = []
	list_Y_t = []
	list_z = []
	list_vx_img = []
	list_vy_img = []

	if des2 is not None:
		# Match descriptors.
		matches_tm = bf.match(des1,des2)

		# Sort them in the order of their distance.
		matches_tm = sorted(matches_tm, key = lambda x:x.distance)

		matches_len = len(matches_tm)
		matches_tm = matches_tm[:int(0.6*matches_len)]

		# print(dt)

		start = 0
		# For each match...
		for mat in matches_tm:

			# Get the matching keypoints for each of the images
			img1_idx = mat.queryIdx
			img2_idx = mat.trainIdx

			# x - columns
			# y - rows
			# Get the coordinates
			(x1,y1) = kp1[img1_idx].pt
			(x2,y2) = kp2[img2_idx].pt

			# print(x1-x2)
			if (abs(x1-x2)>4):

				vx_img = (x1-x2)/(fx*dt)
				vy_img = (y1-y2)/(fy*dt)

				x = (x1 - cx)/fx
				y = (y1 - cy)/fy

				# Vx = vel[1];
				# Vz = -vel[0]
				dist_img = np.linalg.norm(np.array([[x1/fx-x2/fx],[y1/fy-y2/fy]]))
				z = dist/dist_img
				# rospy.loginfo('dist_img %f \t dist %f \t z %f',dist_img,dist,z)

				if (z>0.2 and z<3.0):
					# print(z)

					X = x*z
					Y = y*z

					list_u.append(x1)
					list_v.append(y1)

					list_X_t.append(X)
					list_Y_t.append(Y)
					list_z.append(z)

					list_vx_img.append(vx_img)
					list_vy_img.append(vy_img)

					z_c = np.clip(int(z*30.0), 0, 120)
					color_hsv = np.uint8([[[z_c,255,255]]]) 
					color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)
					color_plot = (int(color_bgr[0][0][0]),int(color_bgr[0][0][1]),int(color_bgr[0][0][2]))
					# color = (255, 0, 0) 
					# print(color_plot)

					cv2.circle(featured_img,(int(x1),int(y1)),3,color_plot,-1)
					cv2.circle(wall_img,(int(x1),int(y1)),3,color_plot,-1)
					cv2.arrowedLine(flow_img, (int(x2),int(y2)), (int(x1),int(y1)), (0,0,255), thickness=1, line_type=8, shift=0, tipLength=0.5)

		plot_image = cv2.drawMatches(img1,kp1,img2,kp2,matches_tm, None, flags=2)
		temporally_matched_featured_image = bridge.cv2_to_imgmsg(plot_image, "8UC3")

	flow_image = bridge.cv2_to_imgmsg(flow_img, "8UC3")
	featured_image = bridge.cv2_to_imgmsg(featured_img, "8UC3")

	if len(list_z)>60:

		U = np.asarray(list_u)
		V = np.asarray(list_v)

		X_t = np.asarray(list_X_t)
		Y_t = np.asarray(list_Y_t)
		Z = np.asarray(list_z)
		# print(np.mean(Z))

		vx_img = np.asarray(list_vx_img)
		vy_img = np.asarray(list_vy_img)

		X_t = np.reshape(X_t,(-1,1))
		Y_t = np.reshape(Y_t,(-1,1))
		Z = np.reshape(Z,(-1,1))
		# data = np.concatenate((X_t,Y_t,Z), axis = 1)
		# print(data)
		# P,res,inlier_no = ransac(data,3,10,2.0)
		# plot_plane_fit(P, list_X_t, list_Y_t, list_z)
		# roll_p, pitch_p, h = get_rph_from_plane(P)

		# YW_min = np.min(X_t)
		# YW_max = np.max(X_t)

		# ZW_min = np.max(Y_t)
		# ZW_max = np.min(Y_t)

		# print(U)
		U_min = int(np.min(U))
		U_max = int(np.max(U))
		V_min = int(np.min(V))
		V_max = int(np.max(V))

		Wall_Z = np.mean(Z)
		Wall_Upper = -Wall_Z*(V_min-cy)/fy + pos[2] + 0.7
		Wall_Lower = -Wall_Z*(V_max-cy)/fy + pos[2] + 0.7
		Wall_Right = Wall_Z*(U_max-cx)/fx
		Wall_Left = Wall_Z*(U_min-cx)/fx

		Wall_X = -0.5*(Wall_Right+Wall_Left)
		Wall_Y = 0.5*(Wall_Upper+Wall_Lower)
		
		print(-Wall_Z*(V_max-cy)/fy)

		if (Wall_Lower<1.8):
			rospy.loginfo('Go Above and depth %f \t Lower Edge %f \t Upper_Edge %f \t Lateral shift %f',Wall_Z, Wall_Lower, Wall_Upper, Wall_X)
		else:
			rospy.loginfo('Go Below and depth %f \t Lower_Edge %f \t Upper_Edge %f \t Lateral shift %f',Wall_Z, Wall_Lower, Wall_Upper, Wall_X)

		cv2.line(wall_img, (0,V_min), (width,V_min), (0,0,255), 2, cv2.LINE_AA) 
		cv2.line(wall_img, (0,V_max), (width,V_max), (255,0,0), 2, cv2.LINE_AA) 

		cv2.line(wall_img, (U_max,0), (U_max,height), (0,0,255), 2, cv2.LINE_AA) 
		cv2.line(wall_img, (U_min,0), (U_min,height), (255,0,0), 2, cv2.LINE_AA) 

		# plot_points(list_X_t, list_Y_t, list_z)

		wall_image = bridge.cv2_to_imgmsg(wall_img, "8UC3")

def plot_points(list_X_t, list_Y_t, list_z):

	fig = plt.figure()
	# ax = fig.gca(projection='3d')
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(list_X_t, list_Y_t, list_z, color = "r")
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.set_zlim([0,3])

	plt.show()

def ransac(data, min_pts, itern, threshDist):
    d_sh = data.shape
    num_pts = d_sh[0]
    num_param = d_sh[1]
#     bestInNum = 0
    bestParam = np.zeros(num_param)
    best_inlier_num = 0
    for i in range(itern):
        sample = np.random.permutation(data)[:min_pts,:]
        A = sample
        b = -np.ones((len(sample),1))
        x = np.linalg.lstsq(A,b, rcond=None)[0]
        # print(x)
        inlier_pts = []
        num_inlier = 0
        for j in range(num_pts):
			A_pt = data[j,:]
			b_pt = -1.0
			dist = abs(np.dot(A_pt,x)-b_pt)/np.linalg.norm(x)
			# print(dist)
			if(dist <= threshDist):
				inlier_pts.append(data[j,:])
				num_inlier = num_inlier+1
                
#         for j in range(num_inlier):
#             inlier_pts = np.array(inlier_pts)
#             A = np.concatenate((inlier_pts[:,:-1], np.ones((len(inlier_pts),1))), axis = 1)
#             b = inlier_pts[:,-1]
#             x,res,_,_ = np.linalg.lstsq(A,b, rcond=None)
        if(num_inlier > best_inlier_num):
            best_inlier_num = num_inlier
            bestParam = x
            best_inlier_pts = inlier_pts
            
    # print(best_inlier_pts)
    best_inlier_pts = np.array(best_inlier_pts)
    A = best_inlier_pts
    b = -np.ones((len(best_inlier_pts),1))
    x,res,_,_ = np.linalg.lstsq(A,b, rcond=None)
    return [x, res, best_inlier_num]

def get_rph_from_plane(P):
	a = P[0]
	b = P[1]
	c = P[2]

	p1x = np.array([0])
	p1y = np.array([0])
	p1z = -1.0/c

	p2x = -1.0/a
	p2y = np.array([0])
	p2z = np.array([0])

	p3x = np.array([0])
	p3y = -1.0/b
	p3z = np.array([0])

	v1 = np.transpose(np.array([p2x-p1x, p2y-p1y, p2z-p1z]))[0]
	v2 = np.transpose(np.array([p3x-p1x, p3y-p1y, p3z-p1z]))[0]

	# print(v1)
	# print(v2)
	z = np.cross(v1,v2)
	# print(z)
	z_hat = z/np.linalg.norm(z)
	# print(z_hat[0])

	x_hat = v1/np.linalg.norm(v1)

	y = np.cross(z_hat,x_hat)	
	y_hat = y/np.linalg.norm(y)

	Rot = R.from_dcm(np.transpose(np.array([x_hat,y_hat,z_hat])))
	Euler = Rot.as_euler('zyx')

	#camera angles
	pitch = Euler[1]
	roll = Euler[2]

	# transform to body angles
	roll = -pitch
	pitch = -roll

	h = 1.0/np.linalg.norm(P)

	return roll, pitch, h

def plot_plane_fit(P, list_X_t, list_Y_t, list_z):
	a = P[0]
	b = P[1]
	c = P[2]
	## plotting plane
	x = np.linspace(-2,2,2)
	y = np.linspace(-2,2,2)

	X,Y = np.meshgrid(x,y)
	Z = (-1.0 - a*X - b*Y) / c

	fig = plt.figure()
	# ax = fig.gca(projection='3d')
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(list_X_t, list_Y_t, list_z, color = "r")
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.set_zlim([0,3])

	surf = ax.plot_surface(X, Y, Z, alpha=0.5)
	plt.show()

def image_assign(current_image):
	global image, prev_image
	# global t_old, dt

	# prev_image = image
	image = current_image

	# t = rospy.get_time()
	# dt = t - t_old
	# t_old = t

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
	# return [yaw, pitch, roll
	return [roll, pitch, yaw]

def skew(v):
	""" 
	Returns the skew-symmetric matrix of a vector
	Ref: https://github.com/dreamdragon/Solve3Plus1/blob/master/skew3.m

	Also known as the cross-product matrix [v]_x such that 
	the cross product of (v x w) is equivalent to the 
	matrix multiplication of the cross product matrix of 
	v ([v]_x) and w

	In other words: v x w = [v]_x * w
	"""
	sk = np.float32([[0, -v[2], v[1]],
	           		[v[2], 0, -v[0]],
	           		[-v[1], v[0], 0]])

	return sk

def get_odom(data):
	global pos, vel, quat, r_in_b, pos_prev, vel_prev
	global image, prev_image
	global t_old, dt

	pos[0] = data.pose.pose.position.x
	pos[1] = data.pose.pose.position.y
	pos[2] = data.pose.pose.position.z

	vel[0] = data.twist.twist.linear.x
	vel[1] = data.twist.twist.linear.y
	vel[2] = data.twist.twist.linear.z

	quat[3] = data.pose.pose.orientation.w
	quat[0] = data.pose.pose.orientation.x
	quat[1] = data.pose.pose.orientation.y
	quat[2] = data.pose.pose.orientation.z
	r_in_b = R.from_quat(quat)

	# print(vel[1])
	# print(vel_prev[1])
	# rospy.loginfo('v_y %f \t v_y_prev %f',vel[1],vel_prev[1])

	if (np.sign(vel[1])!=np.sign(vel_prev[1])):

		# rospy.loginfo('v_y %f \t pos_y %f \t pos_y_prev %f',vel[1],pos[1],pos_prev[1])

		t = rospy.get_time()
		dt = t - t_old

		dist = np.linalg.norm(pos-pos_prev)
		# print(dist)

		pose_estimation(dist)

		pos_prev = np.copy(pos)

		prev_image = image
		t_old = t

	vel_prev = np.copy(vel)

	# print(vel[1])

	# if (abs(vel[1]) < 0.02):
	# 	# flag_entering = True

	# 	rospy.loginfo('v_y %f \t pos_y %f \t pos_y_prev %f',vel[1],pos[1],pos_prev[1])
	# 	# time.sleep(0.3)

	# 	t = rospy.get_time()
	# 	dt = t - t_old

	# 	dist = np.linalg.norm(pos-pos_prev)
	# 	# print(dist)

	# 	pose_estimation(dist)

	# 	pos_prev = pos

	# 	prev_image = image
	# 	t_old = t

	# 	time.sleep(4.0)

def main():
	rospy.init_node('wall_detect', anonymous=True)

	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)
	pub_featured_image = rospy.Publisher('/featured_image', Image, queue_size=10)
	pub_temporally_matched_featured_image = rospy.Publisher('/temporally_matched_featured_image', Image, queue_size=10)
	pub_flow_image = rospy.Publisher('/flow_image', Image, queue_size=10)
	pub_wall_image = rospy.Publisher('/wall_image', Image, queue_size=10)

	pub_pose_wall_in = rospy.Publisher('/pose_wall_in', Odometry, queue_size=10)

	rospy.Subscriber('/image_raw', Image, image_assign)
	rospy.Subscriber('/bebop/odom', Odometry, get_odom)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# try:
		# 	pose_estimation()
		# except:
		# 	rospy.loginfo('Some error ocurred... in target_detect.py')

		# pose_estimation()
	
		pub_temporally_matched_featured_image.publish(temporally_matched_featured_image)
		pub_featured_image.publish(featured_image)
		pub_flow_image.publish(flow_image)
		pub_wall_image.publish(wall_image)
		pub_debug_image.publish(debug_image)

		pub_pose_wall_in.publish(pose_wall_in)

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
