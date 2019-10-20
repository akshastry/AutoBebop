#!/usr/bin/env python

import rospy, time
import numpy as np
from math import atan2, sin, cos, asin, acos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry



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

def EKF_predict(V_k, dt):
	global X_k

	X_k1 = np.zeros(4)

    R_k = np.matmul(R_WC(X_k[3]), R_CQ)
    # print(np.transpose(np.matmul(R_k, np.reshape(V_k, (-1, 1))) * dt))

    X_k1[0:3] = X_k[0:3] + np.transpose(np.matmul(R_k, np.reshape(V_k, (-1, 1))) * dt)[0]
    X_k1[3] = X_k[3]

    #calculate F = df/dx
    F = np.zeros((4,4))
    F[0:3,0:3] = np.eye(3)
    F[0:3,3] = np.transpose(np.matmul(np.matmul(delR_WC(X_k[3]), R_CQ), np.reshape(V_k, (-1, 1))) * dt)[0]
    F[3,3] = 1

    P = np.matmul(F, np.matmul(P, np.transpose(F))) + Q

    X_k = X_k1

def EKF_update(Z_k):

    y_tilda = Z_k - np.transpose(np.matmul(H, np.reshape(X_k1, (-1, 1))))[0]

    S = np.matmul(H, np.matmul(P, np.transpose(H))) + R
    K = np.matmul(P, np.matmul(np.transpose(H), np.linalg.inv(S)))

    X_k1 = X_k1 + np.transpose(np.matmul(K, np.reshape(y_tilda, (-1, 1))))[0]
    P = np.matmul(np.eye(4) - np.matmul(K,H), P)

    X_k = X_k1

def odom_callback(data):
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	vz = data.twist.twist.linear.z

	V_k = np.array([v_x, v_y, v_z])
	dt = rospy

	EKF_predict(V_k, dt)




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


def main():
	global t, t0, flag_land

	rospy.init_node('EKF', anonymous=True)

	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
	pub_to = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
	pub_l = rospy.Publisher('/bebop/land', Empty, queue_size=1)
	pub_pose_in = rospy.Publisher('/pose_in', Odometry, queue_size=1)

	# takeoff
	time.sleep(2.0)
	pub_to.publish()
	time.sleep(5.0)

	rospy.Subscriber('/bebop/odom', Odometry, odom_callback)

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