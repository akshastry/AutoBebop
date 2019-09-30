#!/usr/bin/env python

import rospy, time
from math import atan2, sin, cos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

flag_initialize=True
flag_land = False

K_surface = 0.75

#waypointx = [0, 0.75, 0.75, 0.75*2, 0.75*2, 0.75*3]
#waypointy = [0, 0.75, 0.75, 0.75*2, 0.75*2, 0.75*3]
#waypointz = [0.75, 0.75, 0.75*2, 0.75*2, 0.75*3, 0.75*3]

waypointx = [0, 0.75, 0.75, 0.75*2, 0.75*2, 0.75*3, 0.75*3, 0.75*4]
waypointy = [0, 0.75, 0.75, 0.75*2, 0.75*2, 0.75*3, 0.75*3, 0.75*4]
waypointz = [0, 0.0, 0.75*1, 0.75*1, 0.75*2, 0.75*2, 0.75*3, 0.75*3]

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

def steps2():
    rospy.init_node('lander', anonymous=True)
    time.sleep(2.0)
    pub_to.publish()
    time.sleep(5.0)
    rospy.Subscriber('bebop/odom', Odometry, callback)
    rospy.Subscriber('bebop/land', Empty, callback_land)
    rate = rospy.Rate(10) # 10hz

    rospy.spin()

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + '  x:%f, y:%f, z:%f', data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
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
	
     xd	= waypointx[wpt_idx]
     yd	= waypointy[wpt_idx]
     zd	= waypointz[wpt_idx]

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
        steps2()
    except rospy.ROSInterruptException:
        pass
