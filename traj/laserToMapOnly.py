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
import tf

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
rospy.init_node('laser2Map',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("cyton1500_group")

defaultPose = [-0.00798111114703, -0.00240289023425, 0.973910607185, 3.55992334659e-05, 2.53591989399e-05, 0.707060385227, 0.707153172752]
manipPose1 = [0.441024004555, -0.0157218288562, 0.402612554019, 0.527145535299, 0.477387854476, 0.486045011226, 0.50791600494]
manipPose2 = [0.01, -0.3, 0.55, 0, 0, 0, 0.707]


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
marker.color.b = 1.0

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

manipPos_x = 0.0
manipPos_y = 0.0
manipPos_z = 0.0
gamma = 0.0

botX = 0.0
botY = 0.0
deltaa = 0.0

laserDist = 0.0
xx = yy = zz = 0.0

def toFloat(number,noOfDigits):
	return float(int(number*pow(10,noOfDigits))/(pow(10,noOfDigits)+0.0))


############################ starting of main ###################################


while not rospy.is_shutdown():
	
	odomMessage = rospy.wait_for_message('/fused_bot/odom', Odometry, timeout = 0.5)

	odomQuaternion = (
	    odomMessage.pose.pose.orientation.x,
	    odomMessage.pose.pose.orientation.y,
	    odomMessage.pose.pose.orientation.z,
	    odomMessage.pose.pose.orientation.w)
	odomEuler = tf.transformations.euler_from_quaternion(odomQuaternion)
	botX = odomMessage.pose.pose.position.x
	botY = odomMessage.pose.pose.position.y
	deltaa = odomEuler[2]
	if deltaa < 0.0:
		deltaa = 2*pi + deltaa

	manipCurrentPose = group.get_current_pose().pose
	manipQuaternion = (
	    manipCurrentPose.orientation.x,
	    manipCurrentPose.orientation.y,
	    manipCurrentPose.orientation.z,
	    manipCurrentPose.orientation.w)
	manipEuler = tf.transformations.euler_from_quaternion(manipQuaternion)
	gamma = manipEuler[2]
	manipPos_x = manipCurrentPose.position.x
	manipPos_y = manipCurrentPose.position.y
	manipPos_z = manipCurrentPose.position.z

	laserMessage = rospy.wait_for_message('/fused_bot/QS18VP6LLP_laser/scan', LaserScan, timeout = 0.5)
	if laserMessage.ranges[0] <= 2.0:
		laserDist = laserMessage.ranges[0]
		xx = manipPos_x*cos(deltaa) - manipPos_y*sin(deltaa) + laserDist*sin(deltaa+gamma)
		yy = manipPos_x*sin(deltaa) + manipPos_y*cos(deltaa) - laserDist*cos(deltaa+gamma)
		zz = manipPos_z 
	
		marker.header.stamp = rospy.get_rostime()
		#print " X = ", xx, " Y = ",yy, " Z = ",zz
		p = Point()
		p.x = xx
		p.y = yy
		p.z = zz
		if abs(manipEuler[1]) <= 0.015 and abs(manipEuler[0]) <= 0.015 and abs(manipPos_x - 0.01) <= 0.005 and abs(manipPos_y + 0.3) <= 0.01 :
			marker.points.append(p)
			pub_line_min_dist.publish(marker)
		
	time.sleep(0.01)
