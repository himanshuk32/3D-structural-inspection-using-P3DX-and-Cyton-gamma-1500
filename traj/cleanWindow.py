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
import enum

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


debug = False
simulating = True
mobile = False

robotBaseName = "Pioneer 3DX"
rospy.init_node('generateMap3',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("cyton1500_group")

defaultPose = [-0.00798111114703, -0.00240289023425, 0.973910607185, 3.55992334659e-05, 2.53591989399e-05, 0.707060385227, 0.707153172752]
manipPose1 = [0.441024004555, -0.0157218288562, 0.402612554019, 0.527145535299, 0.477387854476, 0.486045011226, 0.50791600494]
manipPose2 = [0.01, -0.3, 0.81, 0, 0, 0, 0.707]


pub_line_min_dist = rospy.Publisher('visualization_marker', Marker, queue_size = 1)
marker = Marker()
marker.header.frame_id = "/chassis"
marker.type = marker.POINTS
marker.action = marker.ADD
marker.id = 0

# marker scale
marker.scale.x = 0.05
marker.scale.y = 0.05
marker.scale.z = 0.05

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

pi = 3.141592635

velPub = rospy.Publisher('/fused_bot/cmd_vel', Twist,queue_size=10)

n = 0
sign = 1
directionOfRotation = -1

def toFloat(number,noOfDigits):
	return float(int(number*pow(10,noOfDigits))/(pow(10,noOfDigits)+0.0))

def stopBot():
        velPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

def moveBot(linear,angular):
        velPub.publish(Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular)))

def moveStraightForTime(linear, time, startTime):
	moveBot(linear,0)
	if rospy.get_rostime().secs - startTime > time:
		stopBot()
		print " straight return 1"
		return 1
	print " straight return 0"
	return 0

def moveStraightForDistance(linear, reqDist, startDist, odom):
	moveBot(linear,0)
	currentVector = pow((odom[0]*odom[0] + odom[1]*odom[1]),0.5)
	if abs(currentVector - startDist) > reqDist - 0.05:
		stopBot()
		print " straight return 1"
		return 1
	print " straight return 0"
	return 0

def setManipPose(pose, whichPose):
	[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] = whichPose

def rotateBotAngle(angle, currentAngle):
	global directionOfRotation
	if currentAngle <= angle - 0.05:
		moveBot(0,directionOfRotation*0.55)
		return 0
	stopBot()
	return 1

def getLaserData(topic):
	laserMessage = rospy.wait_for_message(topic, LaserScan, timeout = 0.5)
	if laserMessage.ranges[0] > 2.5:	
		return -5
	return laserMessage.ranges[0]
	
def getOdomData(topic):
	odomMessage = rospy.wait_for_message(topic, Odometry, timeout = 0.5)

	odomQuaternion = (
	    odomMessage.pose.pose.orientation.x,
	    odomMessage.pose.pose.orientation.y,
	    odomMessage.pose.pose.orientation.z,
	    odomMessage.pose.pose.orientation.w)
	odomEuler = tf.transformations.euler_from_quaternion(odomQuaternion)
	robotX = odomMessage.pose.pose.position.x
	robotY = odomMessage.pose.pose.position.y
	robotYaw = odomEuler[2]
	if robotYaw < 0.0:
		robotYaw = 2*pi + robotYaw
	return [robotX, robotY, robotYaw]

def getManipData(groupp):
	manippCurrentPose = groupp.get_current_pose().pose
	manippQuaternion = (
	    manippCurrentPose.orientation.x,
	    manippCurrentPose.orientation.y,
	    manippCurrentPose.orientation.z,
	    manippCurrentPose.orientation.w)
	manippEuler = tf.transformations.euler_from_quaternion(manippQuaternion)
	manippYaw = manippEuler[2]
	manippPos_x = manippCurrentPose.position.x
	manippPos_y = manippCurrentPose.position.y
	manippPos_z = manippCurrentPose.position.z
	return [manippPos_x, manippPos_y, manippPos_z, manippYaw] 

def laserDataTF(laserDistance, botPose, manipPose):
	if laserDistance > 0.0:
		xx = manipPose[0]*cos(deltaa) - manipPose[1]*sin(deltaa) + laserDistance*sin(deltaa+gamma) + botPose[0]
		yy = manipPose[0]*sin(deltaa) + manipPose[1]*cos(deltaa) - laserDistance*cos(deltaa+gamma) + botPose[1]
		zz = manipPose[2] - 0.2
		return [xx, yy, zz]
	return [-5,-5,-5]

def mapOnRViz(markerr, point):
	markerr.header.stamp = rospy.get_rostime()
	p = Point()
	p.x = point[0]
	p.y = point[1]
	p.z = point[2]
	markerr.points.append(p)
	pub_line_min_dist.publish(markerr)

def moveManip(groupp, height):
	global n
	global sign
	pose = Pose()
	setManipPose(pose, manipPose2)
	
	if height >= 0.75:
		sign = 1
	if height <= 0.55:
		sign = -1
	n = n + sign
	pose.position.z = pose.position.z + 0.025*n		
	groupp.set_pose_target(pose)
	groupp.go()		
	print "z ",pose.position.z," n ",n
	if((pose.position.z >= height and sign == 1) or (pose.position.z <= height and sign == -1)):
		return 1
	return 0
	

class state(enum.Enum):
	rotate = 1
	moveManipulator = 2
	moveRobotStraight = 4
	
upwardHeight = 0.81
downHeight = 0.55
fsmFlag = 0
randomFlag = 1
manipFlag = 0 #1 is for up, 0 is for down
botReqYaw = 2*pi
botReqYaw2 = 0.0
statee = state.moveManipulator
prevState = state.rotate
heightt = downHeight
startTime = 0
timeeee = 0.5

def finiteStateMachine(laser, odom, manip):
	
	global statee
	global randomFlag
	global manipFlag
	global startTime
	global timeeee
	global botReqYaw
	global botReqYaw2
	
	if statee == state.rotate:
		fsmFlag = rotateBotAngle(botReqYaw, odom[2])
		if laser > 0.0:
			botReqYaw2 = odom[2]
			laserAtYaw2 = laser
		if fsmFlag:
			statee = state.moveManipulator
			manipFlag = 1
			if randomFlag == 1:
				statee = state.rotate
				directionOfRotation = 1
				if botReqYaw2 - botReqYaw >= pi:
					directionOfRotation = -1
				botReqYaw = botReqYaw2
				randomFlag = 2
			if randomFlag == 2:
				statee = state.moveRobotStraight
				startDist = pow((odom[0]*odom[0] + odom[1]*odom[1]),0.5)				
				randomFlag = 3
			
	elif statee == state.moveManipulator:
		if manipFlag:
			heightt = upwardHeight
		else:
			heightt = downHeight
		fsmFlag = moveManip(group,heightt)
		if fsmFlag:
			statee = state.moveRobotStraight
			startTime = rospy.get_rostime().secs
			timeeee = 0.15
			
	elif statee == state.moveRobotStraight:	
		fsmFlag = moveStraightForDist(0.5,1,startDist,odom)
		if fsmFlag:
			statee = state.moveManipulator
			manipFlag = 1 - manipFlag
	print "state  	", statee	
	
	
while not rospy.is_shutdown():
	
	laserDist = getLaserData('/fused_bot/QS18VP6LLP_laser/scan')
	odommPose = getOdomData('/fused_bot/odom')
	manippPose = getManipData(group)
	
	finiteStateMachine(laserDist, odommPose, manippPose)
	tfPoint = laserDataTF(laserDist, odommPose, manippPose)
	mapOnRViz(marker, tfPoint)
	
	time.sleep(0.1)
