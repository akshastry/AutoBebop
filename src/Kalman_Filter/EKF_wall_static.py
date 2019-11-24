#!/usr/bin/env python

import rospy
from math import atan2, sin, cos, asin, acos
import numpy as np
from nav_msgs.msg import Odometry

q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0


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
X_k = np.zeros(3)

# Covariances
Q = np.zeros((3,3))
Q[0,0] = 10**(-4)
Q[1,1] = 10**(-4)
Q[2,2] = 10**(-4)
# Q[3,3] = 10**(-7)

R = np.zeros((3,3))
R[0,0] = 10**(-1)
R[1,1] = 10**(-1)
R[2,2] = 10**(-1)
# R[3,3] = 10**(-3)

P = np.zeros((3,3))
P[0,0] = 10**(1)
P[1,1] = 10**(1)
P[2,2] = 10**(1)
# P[3,3] = 10**(0)


def EKF_predict():
	global X_k, P, Q
	
	P = P + Q #assuming F is identity
	
def EKF_update(Z_k):
	global X_k, P, R

	y_tilda = Z_k - X_k

	S = P + R
	K = np.matmul(P, np.linalg.inv(S))

	X_k = X_k + np.transpose(np.matmul(K, np.reshape(y_tilda, (-1, 1))))[0]
	P = np.matmul(np.eye(3) - K, P)


def target_callback(data):
	global q0, q1, q2, q3
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z

	Z_k = np.array([x, y, z])

	# EKF_predict()
	EKF_update(Z_k)


def main():
	# global psi_qI
	global X_k
	global q0, q1, q2, q3

	rospy.init_node('EKF', anonymous=True)

	pub = rospy.Publisher('/pose_wall_in_filtered', Odometry, queue_size=1)

	# # takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	# rospy.Subscriber('/bebop/odom', Odometry, odom_callback)
	rospy.Subscriber('/pose_wall_in', Odometry, target_callback)

	# time.sleep(1.0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		# t = rospy.get_time() - t0
		# try:
		# 	control()
		# except:
		# 	rospy.loginfo('Some error ocurred')

		EKF_predict()

		pose_EKF = Odometry()
		pose_EKF.header.frame_id = "odom"
		pose_EKF.child_frame_id = "base_link"
		pose_EKF.header.stamp = rospy.get_rostime()
		pose_EKF.pose.pose.position.x = X_k[0]
		pose_EKF.pose.pose.position.y = X_k[1]
		pose_EKF.pose.pose.position.z = X_k[2]
		pose_EKF.twist.twist.linear.x = 0
		pose_EKF.twist.twist.linear.y = 0
		pose_EKF.twist.twist.linear.z = 0
		pose_EKF.pose.pose.orientation.w = q0
		pose_EKF.pose.pose.orientation.x = q1
		pose_EKF.pose.pose.orientation.y = q2
		pose_EKF.pose.pose.orientation.z = q3
		pose_EKF.pose.covariance = np.array([P[0,0],0,0,0,0,0,0,P[1,1],0,0,0,0,0,0,P[2,2],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
		
		print(np.linalg.norm(pose_EKF.pose.covariance))
		if (np.linalg.norm(pose_EKF.pose.covariance)<0.006):
			pub.publish(pose_EKF)
			rospy.loginfo('Go to x %f \t y %f \t z %f', X_k[0], X_k[1], X_k[2])

		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


