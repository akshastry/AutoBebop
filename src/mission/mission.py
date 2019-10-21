#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos, sqrt, asin, acos, atan
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

wpt_dist = 2.1

obs_factor_x = 0.5
obs_factor_y = 0.75
obs_factor_z = 1.0
obs_factor_yaw = 1.0

x_offset = -0.2
y_offset = -0.24
z_offset = -0.17

Hz = 10.0

flag_window_detected = False
flag_window_detected_first = False
flag_mission_window = False
flag_search_mode = True
flag_user_input = False
flag_land = False
flag_track_yaw = 1.0

t = t_search = t_search_start = t_window_detect = 0.0

phase = 0.0

x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0
y_obj_rel_b = x_obj_rel_b = 0.0
xd = yd = zd = yawd = 0.0
x_obj = y_obj = z_obj = 0.0
roll_obj = pitch_obj = yaw_obj = 0.0
xd = 0.0
yd = 0.0
zd = 0.0
yawd = 0.0

r_ac = 0.06
v_ac = 0.1
yaw_ac = 0.02

vel = 0.1

pose_in = Odometry()
pose_d_in = Odometry()
pose_win_in = Odometry()
pose_win_b = Odometry()

def waypoint_gen():
	global pose_d_in
	global t, t_search_start, t_search
	global x_search, y_search
	global phase
	global flag_search_mode, flag_mission_window, flag_window_detected, flag_land, flag_track_yaw
	global xd, yd, zd, yawd, x, y, z, vx, vy ,vz, y_obj_rel_b, x_obj_rel_b, wpt_dist
	global flag_user_input

	if (flag_mission_window):
		if (flag_window_detected):
			# check whether the orientation is okay
			print(wpt_dist)
			
			if ((yawd - yaw)**2 > yaw_ac**2 or ((x-xd)**2 + (y-yd)**2 + (z-zd)**2) > r_ac**2 or vx**2 + vy**2 + vz**2 > v_ac**2):

				xd = x_obj - wpt_dist*cos(yaw_obj)
				yd = y_obj - wpt_dist*sin(yaw_obj)
				zd = z_obj + 0.03
				yawd = yaw_obj + flag_track_yaw*atan2(y_obj_rel_b, x_obj_rel_b)

				rospy.loginfo('wpt xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)

			else:
				
				if (wpt_dist > 1.6):

					wpt_dist = wpt_dist - 0.2

					xd = x_obj - wpt_dist*cos(yaw_obj)
					yd = y_obj - wpt_dist*sin(yaw_obj)
					zd = z_obj + 0.03
					yawd = yaw_obj + flag_track_yaw*atan2(y_obj_rel_b, x_obj_rel_b)
					flag_track_yaw = 0.0

					rospy.loginfo('wpt xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)

				else:

					xd = x_obj + 1.0*cos(yaw_obj)
					yd = y_obj + 1.0*sin(yaw_obj)
					zd = z_obj + 0.05
					yawd = yaw_obj

					flag_mission_window = False
					
					rospy.loginfo('go through wpt xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)
					#print(flag_mission_window)
					#if (flag_user_input==False):
					#	usr_in = raw_input('Check the wpt :')
					#	flag_user_input = True

	if (flag_search_mode):
		#search for the window
		xd = x_search
		yd = y_search

		dt = 1/Hz
		t_search = t - t_search_start
		# rospy.loginfo('t_search %f', t_search)

		Amplitude_z = 0.01*t_search
		Amplitude_yaw = 0.01*t_search

		DEN = sqrt((Amplitude_z*sin(phase))**2 + (Amplitude_z*cos(phase))**2)
		# rospy.loginfo('DEN %f', DEN)

		if (DEN>0.0001):
			phase = phase + dt*(vel/DEN)
			# rospy.loginfo('phase %f',phase)

		zd = Amplitude_z*cos(phase) + 0.5
		yawd = Amplitude_yaw*sin(phase)

		# zd = 0.5
		# yawd = 0.0

		rospy.loginfo('xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)

	if (flag_mission_window==False and flag_search_mode==False and flag_land==False):
		if ((x-xd)**2 + (y-yd)**2 + (z-zd)**2 < (2*r_ac)**2 and vx**2 + vy**2 + vz**2 < (2*v_ac)**2):
			pub_l.publish()
			flag_land = True

	pose_d_in.header.frame_id = "odom"
	pose_d_in.child_frame_id = "base_link"
	pose_d_in.header.stamp = rospy.get_rostime()
	pose_d_in.pose.pose.position.x = xd
	pose_d_in.pose.pose.position.y = yd
	pose_d_in.pose.pose.position.z = zd
	pose_d_in.twist.twist.linear.x = 0.0
	pose_d_in.twist.twist.linear.y = 0.0
	pose_d_in.twist.twist.linear.z = 0.0
	pose_d_in.pose.pose.orientation.w = cos(0.5*yawd)
	pose_d_in.pose.pose.orientation.x = 0.0
	pose_d_in.pose.pose.orientation.y = 0.0
	pose_d_in.pose.pose.orientation.z = sin(0.5*yawd)

# # if window position is in inertial frame
# def window_feedback(data):
# 	global x_obj, y_obj ,z_obj, roll_obj, pitch_obj, yaw_obj
# 	global flag_window_detected, flag_window_detected_first, flag_mission_window, flag_search_mode
# 	global t, t_window_detect

# 	flag_window_detected = True
# 	t_window_detect = t

# 	#Starting the window mission on first detection of the window
# 	if (flag_window_detected_first == False):
# 		flag_window_detected_first == True
# 		flag_mission_window = True
# 		flag_search_mode = False

# 	if (flag_mission_window):
# 		x_obj = data.pose.pose.position.x + x
# 		y_obj = data.pose.pose.position.y + y
# 		z_obj = data.pose.pose.position.z + z

# 		q0 = data.pose.pose.orientation.w
# 		q1 = data.pose.pose.orientation.x
# 		q2 = data.pose.pose.orientation.y
# 		q3 = data.pose.pose.orientation.z
# 		roll_obj, pitch_obj, yaw_obj = quaternion_to_euler(q0, q1, q2, q3)

# 		yaw_obj = yaw_obj + yaw

# if window relative position and orientation are in camera/body frame
def window_feedback(data):
	global pose_win_in, pose_win_b, pub_pose_win_in, y_obj_rel_b, x_obj_rel_b

	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	_,yaw_obj_rel,_ = quaternion_to_euler(q0, q1, q2, q3)

	x_obj_rel_c = data.pose.pose.position.x
	y_obj_rel_c = data.pose.pose.position.y
	z_obj_rel_c = data.pose.pose.position.z

	x_obj_rel_b = obs_factor_x * z_obj_rel_c + x_offset
	y_obj_rel_b = obs_factor_y * -x_obj_rel_c + y_offset
	z_obj_rel_b = obs_factor_z * -y_obj_rel_c + z_offset

	x_obj_rel_in =  x_obj_rel_b*cos(yaw) - y_obj_rel_b*sin(yaw)
	y_obj_rel_in = x_obj_rel_b*sin(yaw) + y_obj_rel_b*cos(yaw)
	z_obj_rel_in = z_obj_rel_b

	x_obj = x + x_obj_rel_in
	y_obj = y + y_obj_rel_in
	z_obj = z + z_obj_rel_in

	yaw_obj = yaw - obs_factor_yaw * yaw_obj_rel

	pose_win_in.header.frame_id = "odom"
	pose_win_in.child_frame_id = "base_link"
	pose_win_in.header.stamp = rospy.get_rostime()
	pose_win_in.pose.pose.position.x = x_obj
	pose_win_in.pose.pose.position.y = y_obj
	pose_win_in.pose.pose.position.z = z_obj
	pose_win_in.twist.twist.linear.x = 0.0
	pose_win_in.twist.twist.linear.y = 0.0
	pose_win_in.twist.twist.linear.z = 0.0
	pose_win_in.pose.pose.orientation.w = cos(0.5*yaw_obj)
	pose_win_in.pose.pose.orientation.x = 0.0
	pose_win_in.pose.pose.orientation.y = 0.0
	pose_win_in.pose.pose.orientation.z = sin(0.5*yaw_obj)

	pub_pose_win_in.publish(pose_win_in)

	# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', x_obj, y_obj, z_obj, yaw_obj)
	# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', x_obj_rel_in, y_obj_rel_in, z_obj_rel_in, yaw_obj)

def window_feedback_filtered(data):
	global pose_win_b
	global x_obj, y_obj ,z_obj, roll_obj, pitch_obj, yaw_obj, yaw, y_obj_rel_b, x_obj_rel_b
	global flag_window_detected, flag_window_detected_first, flag_mission_window, flag_search_mode
	global t, t_window_detect

	flag_window_detected = True
	t_window_detect = t

	#Starting the window mission on first detection of the window
	if (flag_window_detected_first == False):
		flag_window_detected_first = True
		flag_mission_window = True
		flag_search_mode = False

	if (flag_mission_window):
		x_obj = data.pose.pose.position.x
		y_obj = data.pose.pose.position.y
		z_obj = data.pose.pose.position.z
		vx_obj = data.twist.twist.linear.x
		vy_obj = data.twist.twist.linear.y
		vz_obj = data.twist.twist.linear.z

		q0 = data.pose.pose.orientation.w
		q1 = data.pose.pose.orientation.x
		q2 = data.pose.pose.orientation.y
		q3 = data.pose.pose.orientation.z
		roll_obj, pitch_obj, yaw_obj = quaternion_to_euler(q0, q1, q2, q3)

		pose_win_b.header.frame_id = "odom"
		pose_win_b.child_frame_id = "base_link"
		pose_win_b.header.stamp = rospy.get_rostime()
		pose_win_b.pose.pose.position.x = -x_obj_rel_b
		pose_win_b.pose.pose.position.y = -y_obj_rel_b
		pose_win_b.pose.pose.position.z = z_obj
		pose_win_b.twist.twist.linear.x = 0.0
		pose_win_b.twist.twist.linear.y = 0.0
		pose_win_b.twist.twist.linear.z = 0.0
		pose_win_b.pose.pose.orientation.w = cos(0.5*yaw)
		pose_win_b.pose.pose.orientation.x = 0.0
		pose_win_b.pose.pose.orientation.y = 0.0
		pose_win_b.pose.pose.orientation.z = sin(0.5*yaw)


def quad_feedback(data):
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

pub_pose_win_in = rospy.Publisher('/pose_win_in', Odometry, queue_size=10)
pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1)

def main():
	global t, t_search_start
	global x ,y, x_search, y_search
	global flag_search_mode, flag_mission_window, flag_window_detected
	global pose_d_in, pose_win_in, pose_win_b

	rospy.init_node('mission', anonymous=True)

	pub_pose_d_in = rospy.Publisher('/pose_d_in', Odometry, queue_size=1)
	pub_pose_win_b = rospy.Publisher('/pose_win_b', Odometry, queue_size=1)
	pub_to = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
	# pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1)

	#takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	rospy.Subscriber('/pose_win_in_filtered', Odometry, window_feedback_filtered)
	rospy.Subscriber('/pose_rel_win', Odometry, window_feedback)
	rospy.Subscriber('/pose_in', Odometry, quad_feedback)
	time.sleep(1.0)
	rate = rospy.Rate(Hz) # 10hz

	#search initialization step
	t0 = rospy.get_time()
	t_search_start = rospy.get_time() - t0
	flag_search_mode = True
	x_search = x
	y_search = y
	while not rospy.is_shutdown():
		t = rospy.get_time() - t0
		# try:
		# 	waypoint_gen()
		# except:
		# 	rospy.loginfo('Some error ocurred')

		waypoint_gen()
		flag_window_detected = False

		pub_pose_d_in.publish(pose_d_in)
		pub_pose_win_b.publish(pose_win_b)
		
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
