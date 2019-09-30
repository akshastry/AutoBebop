#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

flag_initialize=True
flag_land = False

K_surface = 0.75

waypointx = [0, 0, 0, 0, 1]
waypointy = [0, 1, 0, 0, 0]
waypointz = [0, 1, 2, 1, 0]
waypoint_T = [2, 2, 2, 2, 2]
wpt_idx = 0
r_ac = 0.2
v_ac = 0.1

kpx = 0.1
kpy = 0.1
kpz = 0.5
kp_psi = 0.5

kdx = 0.1
kdy = 0.1
kdz = 0.0
kd_psi = 0.1

t0 = 0.0
x0 = 0.0
y0 = 0.0
z0 = 0.0
xd = 0.0
yd = 0.0
zd = 0.0
psid = 0.0

A = 0.5
T = 5.0
omega = 2*3.14/T

ctrl = Twist()
pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
pub_to = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
pub_l = rospy.Publisher('bebop/land', Empty, queue_size=1)

def diamond():
    rospy.init_node('lander', anonymous=True)
    time.sleep(2.0)
    pub_to.publish()
    time.sleep(5.0)
    rospy.Subscriber('bebop/odom', Odometry, callback)
    rospy.Subscriber('bebop/land', Empty, callback_land)
    rate = rospy.Rate(10) # 10hz

    rospy.spin()

def min_acc_traj(x0, xf, t0, tf):
	traj = (tf**3)*(xf/(t0**2 - 2*t0*tf + tf**2) - (2*x0)/(t0**3 - 3*t0**2*tf + 3*t0*tf**2 - tf**3)) + tf*((xf*(tf**2 + 2*t0*tf))/(t0**2 - 2*t0*tf + tf**2) - (6*t0*tf*x0)/(t0**3 - 3*t0**2*tf + 3*t0*tf**2 - tf**3)) - tf**2*((xf*(t0 + 2*tf))/(t0**2 - 2*t0*tf + tf**2) - (3*x0*(t0 + tf))/(t0**3 - 3*t0**2*tf + 3*t0*tf**2 - tf**3)) + (x0*(- tf**3 + 3*t0*tf**2))/(t0**3 - 3*t0**2*tf + 3*t0*tf**2 - tf**3) - (t0*tf**2*xf)/(t0**2 - 2*t0*tf + tf**2)
	return traj

def min_snap_traj(x0, xf, t0, tf):
    traj = tf**5*((xf*(t0**2 + 4*t0*tf + 2*tf**2))/(2*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4)) + (84*x0*(t0**2 + 3*t0*tf + tf**2))/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7)) + tf**7*((20*x0)/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7) + xf/(6*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4))) - tf**4*((xf*(t0**3 + 12*t0**2*tf + 18*t0*tf**2 + 4*tf**3))/(6*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4)) + (35*x0*(t0**3 + 9*t0**2*tf + 9*t0*tf**2 + tf**3))/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7)) + tf**3*((xf*(4*t0**3*tf + 18*t0**2*tf**2 + 12*t0*tf**3 + tf**4))/(6*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4)) + (140*x0*(t0**3*tf + 3*t0**2*tf**2 + t0*tf**3))/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7)) - tf**6*((70*x0*(t0 + tf))/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7) + (xf*(3*t0 + 4*tf))/(6*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4))) - tf**2*((210*x0*(t0**3*tf**2 + t0**2*tf**3))/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7) + (xf*(2*t0**3*tf**2 + 4*t0**2*tf**3 + t0*tf**4))/(2*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4))) + tf*((xf*(4*t0**3*tf**3 + 3*t0**2*tf**4))/(6*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4)) + (140*t0**3*tf**3*x0)/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7)) + (x0*(- 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7))/(- t0**7 + 7*t0**6*tf - 21*t0**5*tf**2 + 35*t0**4*tf**3 - 35*t0**3*tf**4 + 21*t0**2*tf**5 - 7*t0*tf**6 + tf**7) - (t0**3*tf**4*xf)/(6*(t0**4 - 4*t0**3*tf + 6*t0**2*tf**2 - 4*t0*tf**3 + tf**4))
    return traj

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + '  x:%f, y:%f, z:%f', data ().pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
     global flag_land
     global ctrl
     global flag_initialize
     global t0, x0, y0, z0, xd, yd, zd, psid, wpt_idx

     if (flag_initialize==True):
	t0 = rospy.get_time()
     	x0 = data.pose.pose.position.x
     	y0 = data.pose.pose.position.y
     	z0 = data.pose.pose.position.z
	flag_initialize = False

#     rospy.loginfo('flag_initialize: %r', flag_initialize)
     t = rospy.get_time() - t0
	
     if (wpt_idx+1<len(waypointx)):
        xd	= min_acc_traj(waypointx[wpt_idx], waypointx[wpt_idx+1], t, t+waypoint_T[wpt_idx])
        yd	= min_acc_traj(waypointy[wpt_idx], waypointy[wpt_idx+1], t, t+waypoint_T[wpt_idx])
        zd	= min_acc_traj(waypointz[wpt_idx], waypointz[wpt_idx+1], t, t+waypoint_T[wpt_idx])
    else:
        xd = waypointx[wpt_idx]
        yd = waypointy[wpt_idx]
        zd = waypointz[wpt_idx]

     errx = K_surface * xd - data.pose.pose.position.x + x0
     erry = K_surface * yd - data.pose.pose.position.y + y0
     errz = K_surface * zd - data.pose.pose.position.z + z0
     vx = data.twist.twist.linear.x
     vy = data.twist.twist.linear.y
     vz = data.twist.twist.linear.z

#     rospy.loginfo('%f',errx**2 + erry**2 + errz**2)
     if (errx**2 + erry**2 + errz**2 < r_ac**2 and vx**2 + vy**2 + vz**2 < v_ac**2):
     	wpt_idx = wpt_idx + 1
     
#     rospy.loginfo('vel_x: %f', data.twist.twist.linear.x)
     ux = kpx * (errx) - kdx * vx
     uy = kpy * (erry) - kdy * vy
     uz = kpz * (errz) - kdz * vz

     q0 = data.pose.pose.orientation.w
     q1 = data.pose.pose.orientation.x
     q2 = data.pose.pose.orientation.y
     q3 = data.pose.pose.orientation.z
     psi = atan2(2*(q0*q3 + q1*q2), 1-2*(q2**2 + q3**2))
     ctrl.angular.x = 0.0
     ctrl.angular.y = 0.0
     ctrl.angular.z = kp_psi * (psid - psi)

#    ctrl.linear.x = ux * cos(psi) + uy * sin(psi)
#    ctrl.linear.y = -ux * sin(psi) + uy * cos(psi)
#    ctrl.linear.z = uz

     ctrl.linear.x = ux
     ctrl.linear.y = uy
     ctrl.linear.z = uz

     if(flag_land==False):
     	pub.publish(ctrl)

def callback_land(data):
     global flag_land
     flag_land = True

if __name__ == '__main__':
    try:
        diamond()
    except rospy.ROSInterruptException:
        pass

