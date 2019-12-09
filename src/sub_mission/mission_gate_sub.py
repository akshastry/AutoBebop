#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos, sqrt, asin, acos, atan
from std_msgs.msg import String, Float64, Empty, Int32
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

master_mission_no = 0

flag_initialized = False
flag_land = False
pose_d_in = Odometry()

Hz = 10.0

# search parameters
vel = 0.1
t = t_search = t_search_start = t_window_detect = 0.0
phase = 0.0

# convergence radii
r_ac = 0.05
v_ac = 0.08

# current state of quad
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0

# desired state values, current state should move towards this
xd = 0.0
yd = 0.0
zd = 0.0
yawd = 0.0

# target pose in inertial frame (from EKF)
x_obj = y_obj = z_obj = yaw_obj = 0.0

# initial target location and covariance
x_srch = 0.0
y_srch = 0.0
cox_x = 0.0
cov_y = 0.0

mission_no = 0

A = 0.3
T = 10.0
omega = 2.0*3.14/T

def estimate():
	print("Estimating Gate position...")
	global xd, yd, zd, yawd, x_target, y_target
	global t, t_search_start, t_search
	global phase

	xd = x_srch
	yd = y_srch

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

	zd = Amplitude_z*cos(phase) + 0.3
	yawd = Amplitude_yaw*sin(phase)
	
	# rospy.loginfo('xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)

def converge():
	print("converging to Gate...")
	global mission_no, xd, yd, zd, yawd, x, y, z
	global r_ac, v_ac, x_obj, y_obj, yaw_obj

	wpt_dist = 1.0

	xd = x_obj - wpt_dist*cos(yaw_obj)
	yd = y_obj - wpt_dist*sin(yaw_obj)
	zd = z_obj + 0.05
	yawd = yaw_obj + 0*(3.14/180)
	# yawd = yaw_obj + atan2(y_obj - y, x_obj - x)

	# rospy.loginfo('xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)

	if( ((x-xd)**2 + (y-yd)**2 + (z-zd)**2) < 1.0*r_ac**2 and (vx**2 + vy**2 + vz**2) < 1.0*v_ac**2):
		rospy.loginfo('Go to x %f \t y %f \t z %f', x_obj + wpt_dist*cos(yaw_obj), y_obj + wpt_dist*sin(yaw_obj), z_obj)
		usr_in = raw_input('Should I cross? :( :')
		if(usr_in=="1"):
			mission_no = 3
		else:
			mission_no = 2

def cross():
	print("crossing the gate...")
	global mission_no, xd, yd, yawd, zd, x, y, z
	global r_ac, v_ac, x_obj, y_obj, yaw_obj, master_mission_no

	wpt_dist = 0.5
	
	yaw_tr_offset = 10*(3.14/180)
	xd = x_obj + wpt_dist*cos(yaw_obj+yaw_tr_offset)
	yd = y_obj + wpt_dist*sin(yaw_obj+yaw_tr_offset)
	zd = z_obj + 0.05
	yawd = yaw_obj + 0*(3.14/180)

	# rospy.loginfo('xd %f \t yd %f \t zd %f \t yawd %f', xd, yd, zd, yawd)

	if( ((x-xd)**2 + (y-yd)**2 + (z-zd)**2) < 4*r_ac**2 and (vx**2 + vy**2 + vz**2) < 4*v_ac**2):
		# mission_no = 3
		master_mission_no = 2
		pub_master_mission.publish(master_mission_no)
		print(master_mission_no)

def land():
	print("landing...")
	global mission_no, flag_land, pub_l
	flag_land = True
	mission_no = 4

	pub_l.publish()
	time.sleep(1.0)
	pub_l.publish()
	

def default():
	print("default mission called")
	global xd, yd, zd, x, y, z
	
	xd = x
	yd = y
	zd = z

switcher = {
	0: estimate,
	1: converge,
	2: cross,
	3: land
}

def waypoint_gen():
	global mission_no

	func = switcher.get(mission_no, default)
	func()


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

def target_feedback(data):
	global x_obj, y_obj, z_obj, yaw_obj, mission_no

	if(mission_no == 0):
		mission_no = 1

	if(mission_no == 1):
		x_obj = data.pose.pose.position.x
		y_obj = data.pose.pose.position.y
		z_obj = data.pose.pose.position.z
		q0 = data.pose.pose.orientation.w
		q1 = data.pose.pose.orientation.x
		q2 = data.pose.pose.orientation.y
		q3 = data.pose.pose.orientation.z
		_,_,yaw_obj = quaternion_to_euler(q0, q1, q2, q3)
		
def get_master_mission(data):
	global master_mission_no, flag_initialized, x_srch, y_srch

	master_mission_no = data.data

	if (master_mission_no==1 and flag_initialized==False):
		x_srch = x
		y_srch = y
		flag_initialized = True

def pub_waypoint():
	global xd, yd, zd, yawd, pose_d_in, pub_pose_d_in
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

	pub_pose_d_in.publish(pose_d_in)


pub_pose_d_in = rospy.Publisher('/pose_d_in', Odometry, queue_size=1)
pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1, latch=True)
pub_master_mission = rospy.Publisher('/master_mission_no', Int32, queue_size=1, latch=True)

def main():
	global t, t_search_start
	rospy.init_node('mission_gate', anonymous=True)

	rospy.Subscriber('/pose_gate_in_filtered', Odometry, target_feedback)
	rospy.Subscriber('/pose_in', Odometry, quad_pose)
	rospy.Subscriber('/master_mission_no', Int32, get_master_mission)

	# time.sleep(1.0)
	rate = rospy.Rate(Hz) # 10hz

	t0 = rospy.get_time()
	#search initialization ste
	while not rospy.is_shutdown():
		t = rospy.get_time() - t0

		# try:
		#	if(flag_land == False):
		#		waypoint_gen()
		#		pub_waypoint()
		# except:
		# 	rospy.loginfo('Some error ocurred')

		if (master_mission_no == 1):
			if(flag_land == False):
				waypoint_gen()
				pub_waypoint()
		else:
			t_search_start = rospy.get_time() - t0
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

