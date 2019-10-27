#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
from math import atan2, sin, cos, sqrt, asin, acos, atan

x=y=z=0.0


def show_bullseye_in_rviz(marker_publisher,scale):

    marker1 = Marker( 
    			type = 10, #mesh resource type
                mesh_resource = "package://beginner_tutorial/bullseye.dae", #change to filepath of bullseye.dae located in bebot package
    			id = 69, #haha
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(x - scale/2, y + scale/2, z), Quaternion(.7071, 0, 0, .7071)),
    			scale = Vector3(scale, scale, scale),
    			color = ColorRGBA(0.0, 0.0, 0.0, 1.0), 
                header = Header(frame_id = 'odom')
                )

    marker_publisher.publish(marker1)


def bullseye_feedback(data):
    global x, y, z

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z


def wait_for_time():                                              
    #Wait for simulated time to begin.                                                                         
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    scale = 0.5 #diameter of largest circle on landing bullseye
    rospy.init_node('my_node')
    wait_for_time()
    marker_publisher = rospy.Publisher('bullseye_marker', Marker, queue_size=5)
    rospy.Subscriber('/pose_win_in_filtered', Odometry, bullseye_feedback) #subscribe to bullseye pos topic 

    while not rospy.is_shutdown():
        #make the bullseye marker and publish it 
        rospy.sleep(0.1)
        show_bullseye_in_rviz(marker_publisher,scale) 
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass