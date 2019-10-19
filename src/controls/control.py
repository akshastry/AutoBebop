#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos, asin, acos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

flag_initialize=True
flag_land = False

K_surface = 1.0

kpx = 0.3
kpy = 0.3
kpz = 0.5
kp_yaw = 0.2

kdx = 0.4
kdy = 0.4
kdz = 0.0

t0 = x0 = y0 = z0 = yaw0 = 0.0
x = y = z = vx = vy = vz = roll = pitch = yaw = 0.0
yaw_ac = 2.0*(3.14/180.0)

xd = 0.0
yd = 0.0
zd = 0.0
yawd = 0.0*(3.14/180.0)

ctrl = Twist()
pose_in = Odometry()

def control():
	global ctrl
	global x, y ,z, vx, vy ,vz, roll, pitch, yaw
	global xd, yd, zd, yawd

	errx = xd - x
	erry = yd - y
	if ( zd > 2.0):
		zd = 2.0
		rospy.loginfo('Ceiling reached')
	if ( zd < -0.25):
		zd = -0.25
		rospy.loginfo('Floor reached')
	errz = zd - z

	errxb = errx*cos(yaw) + erry*sin(yaw)
	erryb = -errx*sin(yaw) + erry*cos(yaw)
	errzb = errz
	ux = kpx * (errxb) - kdx * vx
	uy = kpy * (erryb) - kdy * vy
	uz = kpz * (errzb) - kdz * vz

	if (yawd > 0.5*3.14):
		yawd = 0.5*3.14
	if (yawd < -0.5*3.14):
		yawd = -0.5*3.14

	ctrl.angular.x = 0.0
	ctrl.angular.y = 0.0
	ctrl.angular.z = kp_yaw * (yawd - yaw)

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

	x = data.pose.pose.position.x*K_surface - x0
	y = data.pose.pose.position.y*K_surface - y0
	z = data.pose.pose.position.z*K_surface - z0
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

def main():
	global t, t0, flag_land

	rospy.init_node('control', anonymous=True)

	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
	pub_to = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
	pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1)
	pub_pose_in = rospy.Publisher('/pose_in', Odometry, queue_size=1)

	# takeoff
	time.sleep(2.0)
	pub_to.publish()
	time.sleep(5.0)

	rospy.Subscriber('/pose_d_in', Odometry, reference)
	rospy.Subscriber('/bebop/odom', Odometry, feedback)
	rospy.Subscriber('/bebop/land', Empty, callback_land)
	time.sleep(1.0)

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