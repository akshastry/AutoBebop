#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
 
#skeleton source: https://github.com/cse481wi18/cse481wi18/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers                                                           
def show_gate_in_rviz(marker_publisher, tlc_z):
    marker1 = Marker( #left vertical pillar
    			type = 3,
    			id = 69, #haha
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(-0.81/2, 0.0, tlc_z/2), Quaternion(0, 0, 0, -1)),
    			scale = Vector3(0.062, 0.062, tlc_z),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='base_link'),
    			#lifetime=rospy.Duration(60.0)
                )
    marker2 = Marker( #right vertical pillar - green
    			type = 3,
    			id = 70, 
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(0.81/2, 0.0, tlc_z/2), Quaternion(0, 0, 0, -1)),
    			scale = Vector3(0.062, 0.062, tlc_z),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='base_link'),
    			#lifetime=rospy.Duration(60.0)
                )
    marker3 = Marker( #top piece
    			type = 3,
    			id = 71, 
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(0.0, 0, tlc_z), Quaternion(0, 0.7071, 0, 0.7071)),
    			scale = Vector3(0.062, 0.062, 0.87),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='base_link'),
    			#lifetime=rospy.Duration(60.0)
                )
    marker4 = Marker( #bottom piece
    			type = 3,
    			id = 72, 
    			#lifetime = rospy.Duration(100.0),
    			pose = Pose(Point(0.0, 0, tlc_z-0.43), Quaternion(0, 0.7071, 0, 0.7071)),
    			scale = Vector3(0.062, 0.062, 0.87),
    			color = ColorRGBA(0.8, 0.75, 0.2, 1.0),
    			header=Header(frame_id='base_link'),
    			#lifetime=rospy.Duration(60.0)
                )

    marker_publisher.publish(marker1)
    marker_publisher.publish(marker2)
    marker_publisher.publish(marker3)
    marker_publisher.publish(marker4)

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('my_node')
    wait_for_time()

    while not rospy.is_shutdown():
        #make the gate and publish it 
        marker_publisher = rospy.Publisher('gate_marker', Marker, queue_size=5)
        rospy.sleep(0.5)
        show_gate_in_rviz(marker_publisher, tlc_z = 1.0) #tlz is estimated z coord of top left corner in world frame

#call this script every time a new gate position is generated so that z can be updated

# if __name__ == '__main__':
#   main()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass