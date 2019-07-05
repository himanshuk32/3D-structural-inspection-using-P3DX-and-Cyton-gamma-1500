#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time

rospy.init_node('line_pub_example')
pub_line_min_dist = rospy.Publisher('visualization_marker', Marker, queue_size=1)
rospy.loginfo('Publishing example line')

marker = Marker()
marker.header.frame_id = "/base_footprint"
marker.type = marker.LINE_STRIP
marker.action = marker.ADD

# marker scale
marker.scale.x = 0.03
marker.scale.y = 0.03
marker.scale.z = 0.03

# marker color
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 0.0
marker.color.b = 0.0

# marker orientaiton
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

# marker position
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.z = 0.0

j = 0
marker.points = []
    
while not rospy.is_shutdown():
    
    for k in range(1000):
        # first point
	first_line_point = Point()
	first_line_point.x = j
	first_line_point.y = 2.0
	first_line_point.z = 0.5
	marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = j+0.005
	second_line_point.y = 2.005
	second_line_point.z = 0.505
	marker.points.append(second_line_point)
    j = j + 0.01
    # Publish the Marker
    pub_line_min_dist.publish(marker)

    time.sleep(0.5)
