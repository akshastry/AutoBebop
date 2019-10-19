#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos, sqrt, asin, acos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

Hz = 10.0

flag_window_detected = False
flag_window_detected_first = False
flag_mission_window = False
flag_search_mode = True

t = t_search = t_search_start = t_window_detect = 0.0

phase = 0.0

x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0
x_obj = y_obj = z_obj = 0.0
roll_obj = pitch_obj = yaw_obj = 0.0
xd = 0.0
yd = 0.0
zd = 0.0
yawd = 0.0

r_ac = 0.05
v_ac = 0.1
yaw_ac = 0.01

vel = 0.1

pose_in = Odometry()
pose_d_in = Odometry()

def waypoint_gen():
	global pose_d_in
	global t, t_search_start, t_search
	global x_search, y_search
	global phase
	global flag_search_mode, flag_mission_window, flag_window_detected

	if (flag_mission_window):
		if (flag_window_detected):
			# check whether the orientation is okay
			if ((yaw_obj - yaw)**2 > yaw_ac**2):
				xd = x_obj - 1.0*cos(yaw_obj)
				yd = y_obj - 1.0*sin(yaw_obj)
				zd = z_obj
				yawd = yaw_obj
			else:
				xd = x_obj + 1.0*cos(yaw_obj)
				yd = y_obj + 1.0*sin(yaw_obj)
				zd = z_obj
				yawd = yaw_obj
				flag_mission_window = False #End the window mission

	if (flag_search_mode):
		#search for the window
		xd = x_search
		yd = y_search

		dt = 1/Hz
		t_search = t - t_search_start
		# rospy.loginfo('t_search %f', t_search)

		Amplitude_z = 0.01*t_search
		Amplitude_yaw = 0.02*t_search

		DEN = sqrt((Amplitude_z*sin(phase))**2 + (Amplitude_z*cos(phase))**2)
		# rospy.loginfo('DEN %f', DEN)

		if (DEN>0.0001):
			phase = phase + dt*(vel/DEN)
			# rospy.loginfo('phase %f',phase)

		zd = Amplitude_z*cos(phase)
		yawd = Amplitude_yaw*sin(phase)
		rospy.loginfo('zd %f \t yawd %f', zd, yawd)

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

def window_feedback(data):
	global x_obj, y_obj ,z_obj, roll_obj, pitch_obj, yaw_obj
	global flag_window_detected, flag_window_detected_first, flag_mission_window, flag_search_mode
	global t, t_window_detect

	flag_window_detected = True
	t_window_detect = t

	#Starting the window mission on first detection of the window
	if (flag_window_detected_first == False):
		flag_window_detected_first == True
		flag_mission_window = True
		flag_search_mode = False

	if (flag_mission_window):
		x_obj = data.pose.pose.position.x + x
		y_obj = data.pose.pose.position.y + y
		z_obj = data.pose.pose.position.z + z

		q0 = data.pose.pose.orientation.w
		q1 = data.pose.pose.orientation.x
		q2 = data.pose.pose.orientation.y
		q3 = data.pose.pose.orientation.z
		roll_obj, pitch_obj, yaw_obj = quaternion_to_euler(q0, q1, q2, q3)

		yaw_obj = yaw_obj + yaw

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

def main():
	global t, t_search_start
	global x ,y, x_search, y_search
	global flag_search_mode, flag_mission_window, flag_window_detected

	rospy.init_node('mission', anonymous=True)

	pub_pose_d_in = rospy.Publisher('/pose_d_in', Odometry, queue_size=1)
	pub_to = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
	pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1)

	#takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	rospy.Subscriber('/pose_rel_win_filtered', Odometry, window_feedback)
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
		
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
