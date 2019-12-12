#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos, asin, acos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

flag_initialize=False
flag_land = True

# to account for odometry drift on different surfaces
K_surface = 1.0

# P gains
kpx = 0.35
kpy = 0.35
kpz = 0.5
kp_yaw = 0.45

# D gains
kdx = 0.5
kdy = 0.5
kdz = 0.0

# Initial state values from bebop/odom
t0 = x0 = y0 = z0 = yaw0 = 0.0

# Current state values obtained after subtractiong initial values(x0, y0, z0, yaw0) from bebop/odom
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0

# yaw acceptance radius
yaw_ac = 2.0*(3.14/180.0)

# desired state values, current state should move towards this
xd = 0.0
yd = 0.0
zd = 0.0
yawd = 0.0*(3.14/180.0)

# To publish
ctrl = Twist()
pose_in = Odometry()

Trust_R = 2.0*0.55

A = 0.3
T = 10.0
omega = 1.0*3.14/T

def control():
	global ctrl, t, t0
	global x, y ,z, vx, vy ,vz, roll, pitch, yaw
	global xd, yd, zd, yawd

	t = rospy.get_time() - t0

	########### for recording bag purposes ############
	# yd = A * sin(omega*t)
	# if (yd>0.0):
	# 	yd = A
	# else:
	# 	yd = -A

	# yd = A * cos(omega*t) - A
	# zd = 0.25
	# yawd = 0.0 * 1.0 * A * sin(omega*t)

	errx = xd - x
	erry = yd - y

	if (errx**2 + erry**2 > Trust_R**2):
		heading = atan2(erry,errx)
		errx = Trust_R*cos(heading)
		erry = Trust_R*sin(heading)

	# if (errx > 0.5):
	# 	errx = 0.5
	# if (erry > 0.5):
	# 	erry = 0.5

	# if (errx < -0.5):
	# 	errx = -0.5
	# if (erry < -0.5):
	# 	erry = -0.5

	if ( zd > 2.0):
		zd = 2.0
		rospy.loginfo('Ceiling reached')
	if ( zd < -0.4):
		zd = -0.4
		rospy.loginfo('Floor reached')
	errz = zd - z

	errxb = errx*cos(yaw) + erry*sin(yaw)
	erryb = -errx*sin(yaw) + erry*cos(yaw)
	errzb = errz
	ux = kpx * (errxb) - kdx * vx
	uy = kpy * (erryb) - kdy * vy
	uz = kpz * (errzb) - kdz * vz
	# ux = kpx * (errx) - kdx * vx
	# uy = kpy * (erry) - kdy * vy
	# uz = kpz * (errz) - kdz * vz

	# rospy.loginfo('xd %f \t x %f \t ux %f',xd,x,ux)
	# rospy.loginfo('yd %f \t y %f \t uy %f',yd,y,uy)
	# rospy.loginfo('zd %f \t z %f \t uz %f',zd,z,uz)


	#if (ux > 0.3):
	#	ux = 0.3
	#elif (ux < -0.3):
	#	ux = -0.3

	#if (uy > 0.3):
	#	uy = 0.3
	#elif (uy < -0.3):
	#	uy = -0.3

	#if (uz > 0.3):
	#	uz = 0.3
	#elif (uz < -0.3):
	#	uz = -0.3

	#if (yawd > 0.5*3.14):
	#	yawd = 0.5*3.14
	#if (yawd < -0.5*3.14):
	#	yawd = -0.5*3.14

	ctrl.angular.x = 0.0
	ctrl.angular.y = 0.0
	ctrl.angular.z = kp_yaw * (yawd - yaw)

	# rospy.loginfo('yawd %f \t yaw %f',yawd,yaw)

	ctrl.linear.x = ux
	ctrl.linear.y = uy
	ctrl.linear.z = uz

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

def feedback(data):
	global pose_in
	global t0, x0, y0, z0, yaw0, x, y ,z, vx, vy ,vz, roll, pitch, yaw
	global flag_initialize

	if (flag_initialize==True):
		t0 = rospy.get_time()
		x0 = data.pose.pose.position.x
		y0 = data.pose.pose.position.y
		z0 = data.pose.pose.position.z
		q0 = data.pose.pose.orientation.w
		q1 = data.pose.pose.orientation.x
		q2 = data.pose.pose.orientation.y
		q3 = data.pose.pose.orientation.z
		roll0, pitch0, yaw0 = quaternion_to_euler(q0, q1, q2, q3)
		flag_initialize = False
		rospy.loginfo('Initialized Control with x: %f \t y: %f \t z: %f \t yaw: %f',x0,y0,z0,yaw0)

	xm = data.pose.pose.position.x*K_surface - x0
	ym = data.pose.pose.position.y*K_surface - y0
	z = data.pose.pose.position.z*K_surface - z0

	x = xm*cos(yaw0) + ym*sin(yaw0)
	y = -xm*sin(yaw0) + ym*cos(yaw0)

	vx = data.twist.twist.linear.x*K_surface
	vy = data.twist.twist.linear.y*K_surface
	vz = data.twist.twist.linear.z*K_surface

	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

	yaw = yaw - yaw0
	q0, q1, q2, q3 = euler_to_quaternion(roll, pitch, yaw)

	pose_in.header.frame_id = "odom"
	pose_in.child_frame_id = "base_link"
	pose_in.header.stamp = rospy.get_rostime()
	pose_in.pose.pose.position.x = x
	pose_in.pose.pose.position.y = y
	pose_in.pose.pose.position.z = z
	pose_in.twist.twist.linear.x = vx
	pose_in.twist.twist.linear.y = vy
	pose_in.twist.twist.linear.z = vz
	pose_in.pose.pose.orientation.w = q0
	pose_in.pose.pose.orientation.x = q1 
	pose_in.pose.pose.orientation.y = q2
	pose_in.pose.pose.orientation.z = q3

def reference(data):
	global xd, yd, zd, yawd

	xd = data.pose.pose.position.x
	yd = data.pose.pose.position.y
	zd = data.pose.pose.position.z

	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	rolld, pitchd, yawd = quaternion_to_euler(q0, q1, q2, q3)

def callback_land(data):
	global flag_land

	flag_land = True

	print("Landing mode enabled: Disabling control")
	print(flag_land)

def callback_takeoff(data):
	global t0, x0, y0, z0, yaw0
	global flag_initialize
	global flag_land

	print("Flying mode enabled: Enabling control in 5 secs")

	time.sleep(5.0)
	flag_land = False
	
	print(flag_land)

	# t0 = rospy.get_time()
	# x0 = x
	# y0 = y
	# z0 = z
	# yaw0 = yaw
	flag_initialize = True
	# rospy.loginfo('Initialized Control with x: %f \t y: %f \t z: %f \t yaw: %f',x0,y0,z0,yaw0)

def main():
	global t, t0, flag_land

	rospy.init_node('control', anonymous=True)

	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
	# pub_to = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
	# pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1)
	pub_pose_in = rospy.Publisher('/pose_in', Odometry, queue_size=1)

	# # takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	rospy.Subscriber('/pose_d_in', Odometry, reference)
	rospy.Subscriber('/bebop/odom', Odometry, feedback)
	rospy.Subscriber('/bebop/land', Empty, callback_land)
	rospy.Subscriber('/bebop/takeoff', Empty, callback_takeoff)
	# time.sleep(1.0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		t = rospy.get_time() - t0
		# try:
		# 	control()
		# except:
		# 	rospy.loginfo('Some error ocurred')

		control()

		pub_pose_in.publish(pose_in)

		if(flag_land==False):
			pub.publish(ctrl)
		
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
