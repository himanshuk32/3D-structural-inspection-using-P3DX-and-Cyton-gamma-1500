#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
from math import *

pi = 3.141592635

rospy.init_node('point_publish')
pub_line_min_dist = rospy.Publisher('visualization_marker', Marker, queue_size = 1)
marker = Marker()
marker.header.frame_id = "/chassis"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.id = 0

# marker scale
marker.scale.x = 0.3
marker.scale.y = 0.3
marker.scale.z = 0.3

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

marker.points = []

f = 0.0
    
while not rospy.is_shutdown():
	
	marker.header.stamp = rospy.get_rostime()
	
	for i in range(100):
		y = 0.5 * sin(f + i / 100.0 * 2 * pi)
		z = 0.5 * cos(f + i / 100.0 * 2 * pi)
		p = Point()
		p.x = i - 50
		p.y = y
		p.z = z
		marker.points.append(p)
	pub_line_min_dist.publish(marker)
	f = f + 0.04
	time.sleep(0.5)
