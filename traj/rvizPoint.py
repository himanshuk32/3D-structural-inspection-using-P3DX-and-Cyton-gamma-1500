#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
from math import *
import random

pi = 3.141592635

rospy.init_node('point_publish')
pub_line_min_dist = rospy.Publisher('visualization_marker', Marker, queue_size = 1)
marker = Marker()
marker.header.frame_id = "/chassis"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.id = 0

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

marker.points = []

f = 0.25
g = 0.75
h = 0.8
sign = 1

while not rospy.is_shutdown():
	
	marker.header.stamp = rospy.get_rostime()
	
	p = Point()
	p.x = h
	p.y = g
	p.z = f
	marker.points.append(p)
	pub_line_min_dist.publish(marker)

	f = f + random.choice([0.072, 0.056, 0.055])*sign
	if f > 0.81:
		sign = -1
		g = g + random.choice([0.055, 0.065, 0.045, 0.06])
	if f < 0.45:
		sign = 1
		g = g + random.choice([0.055, 0.065, 0.045, 0.06])
	
	if g > 1.5:
		h = h + 0.75
	time.sleep(1.5)
