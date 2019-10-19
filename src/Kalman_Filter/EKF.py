#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry


x_prev = 0.0
y_prev = 0.0
z_prev = 0.0

# rotation matrix from quad to camera frame
R_CQ = np.zeros((3,3))
R_CQ[0,1] = -1
R_CQ[1,2] = -1
R_CQ[2,0] = 1

# state vector
X_k = np.zeros(4)

# Covariances
Q = np.zeros((4,4))
Q[0,0] = 10**(-4)
Q[1,1] = 10**(-4)
Q[2,2] = 10**(-4)
Q[3,3] = 10**(-4)

R = np.zeros((4,4))
R[0,0] = 10**(-3)
R[1,1] = 10**(-3)
R[2,2] = 10**(-3)
R[3,3] = 10**(-3)

P = np.zeros((4,4))
P[0,0] = 10**(-1)
P[1,1] = 10**(-1)
P[2,2] = 10**(-1)
P[3,3] = 10**(-1)

# dh/dx is constant identity
H = np.eye(4) 

def EKF_predict(delX, psi):
	global X_k, P, Q

	X_k1 = np.zeros(4)

    R_k = np.matmul(np.matmul(R_WC(X_k[3]), R_CQ), R_QI(psi))
    # print(np.transpose(np.matmul(R_k, np.reshape(V_k, (-1, 1))) * dt))

    X_k1[0:3] = X_k[0:3] + np.transpose(np.matmul(R_k, np.reshape(delX, (-1, 1))))[0]
    X_k1[3] = X_k[3]

    #calculate F = df/dx
    F = np.zeros((4,4))
    F[0:3,0:3] = np.eye(3)
    R_kF = np.matmul( np.matmul( delR_WC(X_k[3]), R_CQ), R_QI(psi))
    F[0:3,3] = np.transpose(np.matmul(R_kF, np.reshape(delX, (-1, 1)) ))[0]
    F[3,3] = 1

    P = np.matmul(F, np.matmul(P, np.transpose(F))) + Q

    X_k = X_k1

def EKF_update(Z_k):
	global X_k, P, H, R

    y_tilda = Z_k - np.transpose(np.matmul(H, np.reshape(X_k, (-1, 1))))[0]

    S = np.matmul(H, np.matmul(P, np.transpose(H))) + R
    K = np.matmul(P, np.matmul(np.transpose(H), np.linalg.inv(S)))

    X_k1 = X_k1 + np.transpose(np.matmul(K, np.reshape(y_tilda, (-1, 1))))[0]
    P = np.matmul(np.eye(4) - np.matmul(K,H), P)

    X_k = X_k1

def odom_callback(data):
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

	delx = x - x_prev
	dely = y - y_prev
	delz = z - z_prev

	delX = np.array([delx, dely, delz])

	EKF_predict(delX, yaw)

	x_prev = x
	y_prev = y
	z_prev = z

def win_callback(data):
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

	Z_k = np.array([x, y, z, roll])

	KF_update(Z_k)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


def main():

	rospy.init_node('EKF', anonymous=True)

	pub = rospy.Publisher('/pose_rel_win_filtered', Odometry, queue_size=1)

	# # takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	rospy.Subscriber('/bebop/odom', Odometry, odom_callback)
	rospy.Subscriber('/pose_rel_win', Odometry, win_callback)

	# time.sleep(1.0)

	rate = rospy.Rate(5) # 10hz

	while not rospy.is_shutdown():
		# t = rospy.get_time() - t0
		# try:
		# 	control()
		# except:
		# 	rospy.loginfo('Some error ocurred')

		q0, q1, q2, q3 = euler_to_quaternion(0, 0, X_k[3])

		pose_EKF = Odometry()
		pose_EKF.header.frame_id = "pose_rel_win_filtered"
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
		pub.publish(pose_EKF)


	# 	control()

	# 	pub_pose_in.publish(pose_in)

	# 	if(flag_land==False):
	# 		pub.publish(ctrl)
		
	# 	rate.sleep()

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



def R_QI(psi):
	mat = np.zeros((3,3))
	mat[2,2] = 1
	mat[0,0] = cos(psi)
	mat[0,1] = sin(psi)
	mat[1,0] = -sin(psi)
	mat[1,1] = cos(psi)

	return mat

def R_WC(psi):
	mat = np.zeros((3,3))
	mat[2,2] = 1
	mat[0,0] = cos(psi)
	mat[0,1] = -sin(psi)
	mat[1,0] = sin(psi)
	mat[1,1] = cos(psi)

	return mat

def delR_WC(psi):
	mat = np.zeros((3,3))
	mat[0,0] = -sin(psi)
	mat[0,1] = -cos(psi)
	mat[1,0] = cos(psi)
	mat[1,1] = -sin(psi)

	return mat