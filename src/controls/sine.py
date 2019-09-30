#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from math import atan2, sin, cos
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance
from nav_msgs.msg import Odometry

kpx = 0.1
kpy = 0.1
kpz = 0.05
kp_psi = 0.1

ctrl = Twist()
pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
#pub_to = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
pub_l = rospy.Publisher('bebop/land', Empty, queue_size=1)

flag_land = False

t0 = 0.0
A = 0.5
T = 1.0
omega = 2*3.14/T
def sine():
    
    rospy.init_node('lander', anonymous=True)
    t0 = rospy.get_time()    
    rospy.Subscriber('bebop/odom', Odometry, callback)
    rospy.Subscriber('bebop/land', Empty, callback_land)
    rate = rospy.Rate(10) # 10hz

    rospy.spin()

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + '  x:%f, y:%f, z:%f', data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
     
#     rospy.loginfo('flag_land: %r', flag_land)
     t = rospy.get_time() - t0
	
     xd = 0.0 * sin(omega * t)
     yd = 0.0
     zd = 1.0
     psid = 0.0

     ux = kpx * (xd - data.pose.pose.position.x)
     uy = kpy * (yd - data.pose.pose.position.y)
     uz = kpz * (zd - data.pose.pose.position.z)

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

#     if(flag_land==False):
#     	pub.publish(ctrl)

def callback_land(data):
     rospy.loginfo('flag_land: %r', flag_land)
     flag_land = True

if __name__ == '__main__':
    try:
        sine()
    except rospy.ROSInterruptException:
        pass
