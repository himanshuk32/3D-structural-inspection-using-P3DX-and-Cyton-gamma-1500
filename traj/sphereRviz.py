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
marker.scale.x = 0.01
marker.scale.y = 0.01
marker.scale.z = 0.01

# marker color
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0

# marker orientaitn
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

# marker position
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.z = 0.0

marker.points = []

theta = 0.0	
phi = 0.0

while not rospy.is_shutdown():
	
	marker.header.stamp = rospy.get_rostime()
	
	x = 0.5 * cos(theta) * cos(phi)	
	y = 0.5 * cos(theta) * sin(phi)
	z = 0.5 * sin(theta)
	
	p = Point()
	p.x = x + 1
	p.y = y + 1
	p.z = z + 1
	marker.points.append(p)
	
	theta = theta + 0.02
	if theta >= 2*pi:
		phi = phi + 0.02
		theta = 0.0
	
	pub_line_min_dist.publish(marker)
	#time.sleep(0.1)
