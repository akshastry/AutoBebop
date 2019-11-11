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
f = 202
B = 0.03002
cx = 640*0.5
cy = 480*0.5

## camera to body rotation matrix
r_b_c = R.from_euler('zyx', [90, 0, 180], degrees=True)

## initialization of body to inertial rotation matrix
r_in_b = R.from_euler('zyx', [0, 0, 0], degrees=True)

# current state of quad
pos = np.array([0.0, 0.0, 0.0])
quat = np.array([0.0, 0.0, 0.0, 1.0])

# time stuff
dt_L = 0.0
t_L_old = 0.0
t_X_old = 0.0

#Original image in opencv
img_L = np.zeros((480,640,3), np.uint8)
img_R = np.zeros((480,640,3), np.uint8)
img_L_prev = np.zeros((480,640,3), np.uint8)

#ros Images
left_image = Image()
right_image = Image()
left_prev_image = Image()
left_featured_image = Image()
right_featured_image = Image()
spatially_matched_featured_image = Image()
temporally_matched_featured_image = Image()
flow_left_image = Image()
debug_image = Image()

#inertial pose
pose_in = Odometry()

#velocity from VO
vel_VO = Twist()

flag_initialize = True

def pose_estimation():
	global f, B, cx, cy
	global left_image, right_image, left_prev_image, img_L, img_R, img_L_prev
	global left_featured_image, right_featured_image, spatially_matched_featured_image
	global img_L, frame_L, frame_L_prev, dt_L
	global temporally_matched_featured_image, flow_left_image
	global height, width, scale
	global t_X_old, pos, quat, r_in_b
	global pose_in, vel_VO

	# frame_L_prev = frame_L
	# frame_R_prev = frame_R

	try:
		img_L = bridge.imgmsg_to_cv2(left_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	try:
		img_R = bridge.imgmsg_to_cv2(right_image, "bgr8")
	except CvBridgeError as e:
		print(e)

	try:
		img_L_prev = bridge.imgmsg_to_cv2(left_prev_image, "bgr8")
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
	frame_L_prev = cv2.resize(img_L_prev, dim, interpolation = cv2.INTER_AREA)

	#Convert from BGR to gray colorspace
	frame_L = cv2.cvtColor(frame_L, cv2.COLOR_BGR2GRAY);
	frame_R = cv2.cvtColor(frame_R, cv2.COLOR_BGR2GRAY);
	frame_L_prev = cv2.cvtColor(frame_L_prev, cv2.COLOR_BGR2GRAY);

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

	# Initialize lists
	list_X1 = []
	list_X2 = []
	list_X3 = []
	list_X_t = []
	list_Y_t = []
	list_z = []
	list_vx_img = []
	list_vy_img = []
	list_A = []
	list_Y = []

	if des2 is not None and des3 is not None:
		# Match descriptors.
		matches_sp = bf.match(des1,des2)
		matches_tm = bf.match(des1,des3)

		# print("Spatially")
		# for mat in matches_sp[:10]:
		# 	img1_idx_sp = mat.queryIdx
		# 	print(img1_idx_sp)
		# print("temporally")
		# for mat in matches_tm[:10]:
		# 	img1_idx_tm = mat.queryIdx
		# 	print(img1_idx_tm)

		# Sort them in the order of their distance.
		matches_sp = sorted(matches_sp, key = lambda x:x.distance)
		matches_tm = sorted(matches_tm, key = lambda x:x.distance)

		matches_len = min(len(matches_sp),len(matches_tm))
		matches_sp = matches_sp[:int(0.5*matches_len)]
		matches_tm = matches_tm[:int(0.5*matches_len)]

		# Sort them in the order of their queryIdx
		matches_sp = sorted(matches_sp, key = lambda x:x.queryIdx)
		matches_tm = sorted(matches_tm, key = lambda x:x.queryIdx)

		start = 0
		# For each match...
		for mat in matches_sp:

			# Get the matching keypoints for each of the images
			img1_idx_sp = mat.queryIdx
			img2_idx_sp = mat.trainIdx

			for mat_tm in matches_tm[start:]:
				# Get the matching keypoints for each of the images
				img1_idx_tm = mat_tm.queryIdx
				img3_idx_tm = mat_tm.trainIdx

				if (img1_idx_tm == img1_idx_sp):
					start = start+1
					break

			if (img1_idx_tm == img1_idx_sp):
				# x - columns
				# y - rows
				# Get the coordinates
				(x1,y1) = kp1[img1_idx_sp].pt
				(x2,y2) = kp2[img2_idx_sp].pt
				(x3,y3) = kp3[img3_idx_tm].pt

				# print((x1,x2,x3,y1,y2,y3))

				if ( abs(y1-y2)<1 and x2-x1>1):
					z = (f*B)/(x2-x1)

					vx_img = (x1-x3)/(f*dt_L)
					vy_img = (y1-y3)/(f*dt_L)

					x = (x1 - cx)/f
					y = (y1 - cy)/f
					X = x*z
					Y = y*z
					f1 = (-1.0/z, 0.0, x/z, x*y, -(1.0+x*x), y)
					f2 = (0.0, -1.0/z, y/z, 1.0+y*y, -x*y, -x)

					# if (len(X1)==0):
					# 	X1 = np.array([[x1,y1]])
					# else:
					# 	X1 = np.concatenate((X1,np.array([[x1,y1]])),axis=0)
					# # print(X1)
					list_X1.append((x1, y1))
					list_X2.append((x2, y2))
					list_X3.append((x3, y3))
					list_X_t.append(X)
					list_Y_t.append(Y)
					list_z.append(z)
					list_vx_img.append(vx_img)
					list_vy_img.append(vy_img)

					list_A.append(f1)
					list_A.append(f2)
					list_Y.append(vx_img)
					list_Y.append(vy_img)

					cv2.arrowedLine(flow_image, (int(x3),int(y3)), (int(x1),int(y1)), (0,0,255), thickness=1, line_type=8, shift=0, tipLength=0.5)


		flow_left_image = bridge.cv2_to_imgmsg(flow_image, "8UC3")

		plot_image = cv2.drawMatches(img1,kp1,img2,kp2,matches_sp, None, flags=2)
		spatially_matched_featured_image = bridge.cv2_to_imgmsg(plot_image, "8UC3")

		plot_image = cv2.drawMatches(img1,kp1,img3,kp3,matches_tm, None, flags=2)
		temporally_matched_featured_image = bridge.cv2_to_imgmsg(plot_image, "8UC3")

		if list_X1:

			X1 = np.asarray(list_X1)
			X2 = np.asarray(list_X2)
			X3 = np.asarray(list_X3)
			X_t = np.asarray(list_X_t)
			Y_t = np.asarray(list_Y_t)
			Z = np.asarray(list_z)
			vx_img = np.asarray(list_vx_img)
			vy_img = np.asarray(list_vy_img)

			A = np.asarray(list_A)
			Y = np.asarray(list_Y)

			X_t = np.reshape(X_t,(-1,1))
			Y_t = np.reshape(Y_t,(-1,1))
			Z = np.reshape(Z,(-1,1))
			data = np.concatenate((X_t,Y_t,Z), axis = 1)
			# print(data)
			P,res,inlier_no = ransac(data,3,10,1.0)
			# print(P)
			# print(inlier_no*1.0/X_t.shape[0]*1.0)
			# plot_plane_fit(P, list_X_t, list_Y_t, list_z)
			roll_p, pitch_p, h = get_rph_from_plane(P)
			# r_in_b = R.from_euler('zyx', [0, pitch, roll]) * r_b_c.inv() 
			# Euler = r_in_b.as_euler('zyx')
			# pitch = Euler[1]
			# roll = Euler[2]
			# print(Euler)
			# print(h)

			# print(A)
			# print(Y)
			V,residuals,_,_ = np.linalg.lstsq(A, Y, rcond=None)

			if (residuals<100.0):
				# print(residuals)
			### RANSAC

				t_X = rospy.get_time()
				d_t_X = (t_X - t_X_old)
				t_X_old = t_X

				V_ang_cam = V[3:6]
				V_ang_b = r_b_c.apply(V_ang_cam)
				delta_r_in_b = R.from_dcm(expm(d_t_X*skew(V_ang_b)))
				r_in_b = delta_r_in_b*r_in_b
				Euler = r_in_b.as_euler('zyx')
				yaw = Euler[0]
				pitch = Euler[1]
				roll = Euler[2]
				beta = 1.0
				r_in_b = R.from_euler('zyx', [yaw, beta*pitch_p+(1.0-beta)*pitch, beta*roll_p+(1.0-beta)*roll])

				V_lin_cam = V[:3]
				V_lin_b = r_b_c.apply(V_lin_cam)
				V_lin_in = r_in_b.apply(V_lin_b)

				pos = pos + d_t_X*V_lin_in[:3]
				pos[2] = beta*h + (1.0-beta)*pos[2]
				# print(pos)
				quat = r_in_b.as_quat()
				# print(quat)

				pose_in.header.frame_id = "odom"
				pose_in.child_frame_id = "base_link"
				pose_in.header.stamp = rospy.get_rostime()
				pose_in.pose.pose.position.x = pos[0]
				pose_in.pose.pose.position.y = pos[1]
				pose_in.pose.pose.position.z = pos[2]
				pose_in.twist.twist.linear.x = V_lin_in[0]
				pose_in.twist.twist.linear.y = V_lin_in[1]
				pose_in.twist.twist.linear.z = V_lin_in[2]
				pose_in.pose.pose.orientation.w = quat[3]
				pose_in.pose.pose.orientation.x = quat[0]
				pose_in.pose.pose.orientation.y = quat[1]
				pose_in.pose.pose.orientation.z = quat[2]
				pose_in.twist.twist.angular.x = V_ang_b[0]
				pose_in.twist.twist.angular.y = V_ang_b[1]
				pose_in.twist.twist.angular.z = V_ang_b[2]

				vel_VO.linear.x = V_lin_in[0]
				vel_VO.linear.y = V_lin_in[1]
				vel_VO.linear.z = V_lin_in[2]
				vel_VO.angular.x = V_ang_b[0]
				vel_VO.angular.y = V_ang_b[1]
				vel_VO.angular.z = V_ang_b[2]

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

def left_image_assign(current_image):
	global left_image, left_prev_image
	global t_L_old, dt_L

	left_prev_image = left_image
	left_image = current_image

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

def get_first_odom_val(data):
	global pos, quat, r_in_b, flag_initialize
	if (flag_initialize == True):
		pos[0] = data.pose.pose.position.x
		pos[1] = data.pose.pose.position.y
		pos[2] = data.pose.pose.position.z
		quat[3] = data.pose.pose.orientation.w
		quat[0] = data.pose.pose.orientation.x
		quat[1] = data.pose.pose.orientation.y
		quat[2] = data.pose.pose.orientation.z
		r_in_b = R.from_quat(quat)
		flag_initialize = False

def main():
	global t_X_old, t_L_old
	rospy.init_node('target_detect', anonymous=True)

	pub_debug_image = rospy.Publisher('/debug_image', Image, queue_size=10)
	pub_left_featured_image = rospy.Publisher('/left_featured_image', Image, queue_size=10)
	pub_right_featured_image = rospy.Publisher('/right_featured_image', Image, queue_size=10)
	pub_spatially_matched_featured_image = rospy.Publisher('/spatially_matched_featured_image', Image, queue_size=10)
	pub_temporally_matched_featured_image = rospy.Publisher('/temporally_matched_featured_image', Image, queue_size=10)
	pub_flow_left_image = rospy.Publisher('/flow_left_image', Image, queue_size=10)

	pub_pose_in_VO = rospy.Publisher('/pose_in_VO_lst', Odometry, queue_size=10)
	pub_vel_VO = rospy.Publisher('/vel_VO', Twist, queue_size=10)

	rospy.Subscriber('/duo3d/left/image_rect', Image, left_image_assign)
	rospy.Subscriber('/duo3d/right/image_rect', Image, right_image_assign)

	## only for taking the first position and orientation so that the origin of the odom(ground truth) matches tha origin of the pose from VO
	rospy.Subscriber('bebop/odom', Odometry, get_first_odom_val)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if (flag_initialize==False):
			# try:
			# 	pose_estimation()
			# except:
			# 	rospy.loginfo('Some error ocurred... in target_detect.py')

			pose_estimation()
		
			pub_spatially_matched_featured_image.publish(spatially_matched_featured_image)
			pub_temporally_matched_featured_image.publish(temporally_matched_featured_image)
			pub_flow_left_image.publish(flow_left_image)
			pub_debug_image.publish(debug_image)

			pub_pose_in_VO.publish(pose_in)
			pub_vel_VO.publish(vel_VO)

		else:
			t_X_old = rospy.get_time()
			t_L_old = rospy.get_time()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
