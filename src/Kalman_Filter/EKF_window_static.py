#!/usr/bin/env python

import rospy
from math import atan2, sin, cos, asin, acos
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Empty, Int32

master_mission_no = 0

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
X_k = np.zeros(4)

# Covariances
Q = np.zeros((4,4))
Q[0,0] = 10**(-3)
Q[1,1] = 10**(-3)
Q[2,2] = 10**(-3)
Q[3,3] = 10**(-3)

R = np.zeros((4,4))
R[0,0] = 10**(-2)
R[1,1] = 10**(-2)
R[2,2] = 10**(-2)
R[3,3] = 10**(-2)

P = np.zeros((4,4))
P[0,0] = 10**(2)
P[1,1] = 10**(2)
P[2,2] = 10**(2)
P[3,3] = 10**(2)


def EKF_predict():
	global X_k, P, Q
	
	P = P + Q #assuming F is identity
	
def EKF_update(Z_k):
	global X_k, P, R

	y_tilda = Z_k - X_k

	S = P + R
	K = np.matmul(P, np.linalg.inv(S))

	X_k = X_k + np.transpose(np.matmul(K, np.reshape(y_tilda, (-1, 1))))[0]
	P = np.matmul(np.eye(4) - K, P)


def win_callback(data):
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z
	q0 = data.pose.pose.orientation.w
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	roll, pitch, yaw = quaternion_to_euler(q0, q1, q2, q3)

	Z_k = np.array([x, y, z, yaw])

	# EKF_predict()
	EKF_update(Z_k)


def get_master_mission(data):
	global master_mission_no
	# print(data.data)
	master_mission_no = data.data

def main():
	# global psi_qI
	global X_k

	rospy.init_node('EKF', anonymous=True)

	pub = rospy.Publisher('/pose_gate_in_filtered', Odometry, queue_size=1)

	# # takeoff
	# time.sleep(2.0)
	# pub_to.publish()
	# time.sleep(5.0)

	# rospy.Subscriber('/bebop/odom', Odometry, odom_callback)
	rospy.Subscriber('/pose_gate_in', Odometry, win_callback)

	rospy.Subscriber('/master_mission_no', Int32, get_master_mission)

	# time.sleep(1.0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():

		if (master_mission_no == 1):
			# t = rospy.get_time() - t0
			# try:
			# 	control()
			# except:
			# 	rospy.loginfo('Some error ocurred')

			EKF_predict()

			q0, q1, q2, q3 = euler_to_quaternion(0, 0, X_k[3])

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
			pose_EKF.pose.covariance = np.array([P[0,0],0,0,0,0,0,0,P[1,1],0,0,0,0,0,0,P[2,2],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,P[3,3]])
			
			# print(np.linalg.norm(pose_EKF.pose.covariance))
			if (np.linalg.norm(pose_EKF.pose.covariance)<0.02):
				# rospy.loginfo('x %f \t y %f \t z %f \t yaw %f', X_k[0], X_k[1], X_k[2], X_k[3])
				pub.publish(pose_EKF)

		rate.sleep()
		


	# 	control()

	# 	pub_pose_in.publish(pose_in)

	# 	if(flag_land==False):
	# 		pub.publish(ctrl)
		
		






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


