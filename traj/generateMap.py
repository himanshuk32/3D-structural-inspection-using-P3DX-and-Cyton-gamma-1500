#!/usr/bin/env python

# imports

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time 
import numpy
import sys
import copy
import matplotlib.pyplot as plt
import os
from math import *

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

pi = 3.141592635

debug = False
simulating = True
mobile = False

robotBaseName = "Pioneer 3DX"
rospy.init_node('generateMap',anonymous=True)
group = moveit_commander.MoveGroupCommander("cyton1500_group")

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

pointFromGlobalFOR_x = 0
pointFromGlobalFOR_y = 0
pointFromGlobalFOR_z = 0

def toFloat(number,noOfDigits):
	return float(int(number*pow(10,noOfDigits))/(pow(10,noOfDigits)+0.0))


while not rospy.is_shutdown():
	
	manipCurrentPose = group.get_current_pose().pose
	manipAng_z = manipCurrentPose.orientation.z
	manipAng_x = manipCurrentPose.orientation.x	
	manipAng_y = manipCurrentPose.orientation.y	
	manipPos_x = manipCurrentPose.position.x
	manipPos_y = manipCurrentPose.position.y
	manipPos_z = manipCurrentPose.position.z

	laserMessage = rospy.wait_for_message('/fused_bot/QS18VP6LLP_laser/scan', LaserScan, timeout = 0.5)
	odomMessage = rospy.wait_for_message('/fused_bot/odom', Odometry, timeout = 0.5)

	if laserMessage.ranges[0] < 10:
		pointFromLaser_x = - manipPos_x + laserMessage.ranges[0]*cos(manipAng_x)
		pointFromLaser_y = - manipPos_y + laserMessage.ranges[0]*cos(manipAng_y)
		pointFromLaser_z = - manipPos_z + laserMessage.ranges[0]*cos(manipAng_z)
		pointFromGlobalFOR_x = pointFromLaser_x + odomMessage.pose.pose.position.x
		pointFromGlobalFOR_y = pointFromLaser_y + odomMessage.pose.pose.position.y
		pointFromGlobalFOR_z = pointFromLaser_z + odomMessage.pose.pose.position.z
	
	
	marker.header.stamp = rospy.get_rostime()
	xx = pointFromGlobalFOR_x
	yy = pointFromGlobalFOR_y
	zz = pointFromGlobalFOR_z
	print " manipPos_x = ",toFloat(manipPos_x,3)," manipPos_y = ",toFloat(manipPos_y,2)," manipPos_z = ",toFloat(manipPos_z,3)," pointFromLaser_x = ",toFloat(pointFromLaser_x,3)," pointFromLaser_y = ",toFloat(pointFromLaser_y,3)," pointFromLaser_z = ",toFloat(pointFromLaser_z,3)," X = ", toFloat(xx,3), " Y = ",toFloat(yy,3), " Z = ",toFloat(zz,3)
	#print " X = ", xx, " Y = ",yy, " Z = ",zz
	p = Point()
	p.x = xx
	p.y = yy
	p.z = zz
	marker.points.append(p)
	
	pub_line_min_dist.publish(marker)

	time.sleep(0.1)
