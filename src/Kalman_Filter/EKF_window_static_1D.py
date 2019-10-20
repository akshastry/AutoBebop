#!/usr/bin/env python

import rospy
from math import atan2, sin, cos, asin, acos
from nav_msgs.msg import Odometry


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




# psi_qI = 0.0
# X_qI = np.array([0.0,0.0,0.0])


# state vector
x = y = z = psi = 0.0

# Covariances
Q_x = 0.0
Q_y = 0.0
Q_z = 0.0
Q_psi = 0.0

R_x = 10**(-3)
R_y = 10**(-3)
R_z = 10**(-3)
R_psi = 10**(-3)

P_x = 10**(-1)
P_y = 10**(-1)
P_z = 10**(-1)
P_psi = 10**(-1)


def EKF_predict():
	global P_x, P_y, P_z, P_psi, Q_x, Q_y, Q_z, Q_psi
	
	P_x = P_x + Q_x
	P_y = P_y + Q_y
	P_z = P_z + Q_z
	P_psi = P_psi + Q_psi

	
def EKF_update(Z_x, Z_y, Z_z, Z_psi):
	global x, y, z, psi, P_x, P_y, P_z, P_psi, R_x, R_y, R_z, R_psi


	K_x = P_x/(P_x + R_x)
	K_y = P_y/(P_y + R_y)
	K_z = P_z/(P_z + R_z)
	K_psi = P_psi/(P_psi + R_psi)

	x = x + K_x * (Z_x - x)
	y = y + K_y * (Z_y - y)
	z = z + K_z * (Z_z - z)
	psi = psi + K_psi * (Z_psi - psi)

	P_x = (1 - K_x) * P_x
	P_y = (1 - K_y) * P_y
	P_z = (1 - K_z) * P_z
	P_psi = (1 - K_psi) * P_psi


def win_callback(data):
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

	# EKF_predict()
	EKF_update(x, y, z, yaw)


def main():
	# global psi_qI
	global x,y,z, psi

	rospy.init_node('EKF', anonymous=True)

	pub = rospy.Publisher('/pose_win_in_filtered', Odometry, queue_size=1)

	# # takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	# rospy.Subscriber('/bebop/odom', Odometry, odom_callback)
	rospy.Subscriber('/pose_win_in', Odometry, win_callback)

	# time.sleep(1.0)

	rate = rospy.Rate(5) # 10hz

	while not rospy.is_shutdown():
		# t = rospy.get_time() - t0
		# try:
		# 	control()
		# except:
		# 	rospy.loginfo('Some error ocurred')

		EKF_predict()

		q0, q1, q2, q3 = euler_to_quaternion(0, 0, psi)

		pose_EKF = Odometry()
		pose_EKF.header.frame_id = "odom"
		pose_EKF.child_frame_id = "base_link"
		pose_EKF.header.stamp = rospy.get_rostime()
		pose_EKF.pose.pose.position.x = x
		pose_EKF.pose.pose.position.y = y
		pose_EKF.pose.pose.position.z = z
		pose_EKF.twist.twist.linear.x = 0
		pose_EKF.twist.twist.linear.y = 0
		pose_EKF.twist.twist.linear.z = 0
		pose_EKF.pose.pose.orientation.w = q0
		pose_EKF.pose.pose.orientation.x = q1 
		pose_EKF.pose.pose.orientation.y = q2
		pose_EKF.pose.pose.orientation.z = q3
		pub.publish(pose_EKF)

		rate.sleep()


	# 	control()

	# 	pub_pose_in.publish(pose_in)

	# 	if(flag_land==False):
	# 		pub.publish(ctrl)
		
	# 	rate.sleep()






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


