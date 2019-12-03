#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, sqrt, asin, acos, atan

x=y=z=q0=q1=q2=q3=yaw=0.0

#skeleton source: https://github.com/cse481wi18/cse481wi18/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers                                                           
def show_gate_in_rviz(marker_publisher, tlc_z):
    pi = 3.14159265359
    q_val = 0.70710678118
    q_l_r = euler_to_quaternion(0.0, 0.5*pi, yaw+0.5*pi)
    marker1 = Marker( #left vertical pillar
    			type = 3,
    			id = 69, #haha
    			#lifetime = rospy.Duration(100.0),
                pose = Pose(Point(x-(0.81*0.5)*cos(yaw-0.5*pi), y-(0.81*0.5)*sin(yaw-0.5*pi), z/2), Quaternion(0, 0, 0, -1)),
    			# pose = Pose(Point(-0.81/2, 0.0, tlc_z/2), Quaternion(0, 0, 0, -1)),
    			scale = Vector3(0.062, 0.062, tlc_z),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='odom'),
    			#lifetime=rospy.Duration(60.0)
                )
    marker2 = Marker( #right vertical pillar - green
    			type = 3,
    			id = 70, 
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(x+(0.81*0.5)*cos(yaw-0.5*pi), y+(0.81*0.5)*sin(yaw-0.5*pi), z/2), Quaternion(0, 0, 0, -1)),
    			scale = Vector3(0.062, 0.062, tlc_z),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='odom'),
    			#lifetime=rospy.Duration(60.0)
                )
    marker3 = Marker( #top piece
    			type = 3,
    			id = 71, 
    			#lifetime = rospy.Duration(100.0),
    			# pose = Pose(Point(x, y, z+0.43*0.5), Quaternion(q_val, 0, 0, q_val)),
                pose = Pose(Point(x, y, z+0.43*0.5), Quaternion(q_l_r[1],q_l_r[2],q_l_r[3],q_l_r[0])),
    			scale = Vector3(0.062, 0.062, 0.87),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='odom'),
    			#lifetime=rospy.Duration(60.0)
                )
    marker4 = Marker( #bottom piece
    			type = 3,
    			id = 72, 
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(x, y, z-0.43*0.5), Quaternion(q_l_r[1],q_l_r[2],q_l_r[3],q_l_r[0])),
    			scale = Vector3(0.062, 0.062, 0.87),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='odom'),
    			#lifetime=rospy.Duration(60.0)
                )

    marker_publisher.publish(marker1)
    marker_publisher.publish(marker2)
    marker_publisher.publish(marker3)
    marker_publisher.publish(marker4)

def window_feedback(data):
    global x, y, z, q0, q1, q2, q3, yaw

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z

    q0 = data.pose.pose.orientation.w
    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    _,_,yaw = quaternion_to_euler(q0, q1, q2, q3)

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

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():

    rospy.init_node('my_node')
    wait_for_time()
    marker_publisher = rospy.Publisher('gate_marker', Marker, queue_size=5)
    rospy.Subscriber('/pose_win_in_filtered', Odometry, window_feedback)
    # rospy.Subscriber('/pose_win_in', Odometry, window_feedback)
    while not rospy.is_shutdown():
        #make the gate and publish it 
        
        rospy.sleep(0.1)
        show_gate_in_rviz(marker_publisher, tlc_z = 0.8) #tlz is estimated z coord of top left corner in world frame
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass