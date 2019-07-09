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

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from math import *
pi = 3.141592635

debug = False
simulating = True
mobile = False

robotBaseName = "Pioneer 3DX"
rospy.init_node('manipCurrentPose',anonymous=True)
group = moveit_commander.MoveGroupCommander("cyton1500_group")

def quaternionToEuler(quaternionPose):
	quaternion = (
	    quaternionPose.orientation.x,
	    quaternionPose.orientation.y,
	    quaternionPose.orientation.z,
	    quaternionPose.orientation.w)	
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler

while not rospy.is_shutdown():
	
	manipCurrentPose = group.get_current_pose().pose
	manipEuler = quaternionToEuler(manipCurrentPose)
	manippRoll = manipEuler[0]*180/pi
	manippPitch = manipEuler[1]*180/pi
	manippYaw = manipEuler[2]*180/pi
	manipPos_x = manipCurrentPose.position.x
	manipPos_y = manipCurrentPose.position.y
	manipPos_z = manipCurrentPose.position.z

	#print " manipulator's  X = ",manipCurrentPose.position.x,"  Y = ",manipCurrentPose.position.y, "  Z = ",manipCurrentPose.position.z
	print "  angle X = ",manippRoll," angle Y = ",manippPitch, " angle Z = ",manippYaw 
	time.sleep(0.1)
