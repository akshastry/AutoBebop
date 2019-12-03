#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, sqrt, asin, acos, atan

x=y=z=0.0
rotation_q = [0,0,0,0]

def show_bullseye_in_rviz(marker_publisher):

    marker1 = Marker( 
    			type = 1, #mesh resource type
    			id = 69, #haha
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(x, y, z), Quaternion(-rotation_q[1],rotation_q[2],-rotation_q[0],rotation_q[3])),
    			scale = Vector3(1.1,0.8,0.05),
    			color = ColorRGBA(0.7, 0.5, 0.3, 0.8), 
                header = Header(frame_id = 'odom')
                )
    marker2 = Marker( 
                type = 1, #mesh resource type
                id = 70, #haha
                #lifetime = rospy.Duration(100.0),
                pose = Pose(Point(x, y, z), Quaternion(0,0,0,1)),#-rotation_q[1],rotation_q[2],-rotation_q[0],rotation_q[3])),
                scale = Vector3(0.55,800,0.005),
                color = ColorRGBA(0.0, 0.0, 0.9, 0.5), 
                header = Header(frame_id = 'odom')
                )
    marker_publisher.publish(marker1)
    marker_publisher.publish(marker2)

def bullseye_feedback(data):
    global x, y, z, rotation_q

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    rotation_q[0] = data.pose.pose.orientation.w 
    rotation_q[1] = data.pose.pose.orientation.x
    rotation_q[2] = data.pose.pose.orientation.y
    rotation_q[3] = data.pose.pose.orientation.z


def wait_for_time():                                              
    #Wait for simulated time to begin.                                                                         
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('my_node')
    wait_for_time()
    marker_publisher = rospy.Publisher('bullseye_marker', Marker, queue_size=5)
    rospy.Subscriber('/pose_bridge_in', Odometry, bullseye_feedback) #subscribe to bullseye pos topic 

    while not rospy.is_shutdown():
        #make the bullseye marker and publish it 
        rospy.sleep(0.1)
        show_bullseye_in_rviz(marker_publisher) 
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass