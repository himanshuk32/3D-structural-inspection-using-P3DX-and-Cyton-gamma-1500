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

defaultPoseP = [-0.00798111114703, -0.00240289023425, 0.973910607185]
defaultPoseQ = [3.55992334659e-05, 2.53591989399e-05, 0.707060385227, 0.707153172752]

robotBaseName = "Pioneer 3DX"
rospy.init_node('setManipPose',anonymous=True)
group = moveit_commander.MoveGroupCommander("cyton1500_group")

def quaternionToEuler1(quaternionPose):
	quaternion = (
	    quaternionPose.orientation.x,
	    quaternionPose.orientation.y,
	    quaternionPose.orientation.z,
	    quaternionPose.orientation.w)	
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler

def quaternionToEuler2(quaternionPose):
	quaternion = (
	    quaternionPose[0],
	    quaternionPose[1],
	    quaternionPose[2],
	    quaternionPose[3])	
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler
	
def eulerToQuaternion(eulerPose):
	euler = (
	    eulerPose[0],
	    eulerPose[1],
	    eulerPose[2])
	quaternion = tf.transformations.quaternion_from_euler(euler)
	return quaternion	
98*
def setManipPose(pose, whichPose):
	[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] = whichPose


#manipReqPoseQ = [0.0, 0.0, 0.0, 0.707]
manipReqPoseP = [0.01, 0.3, 0.55]
manipReqPoseE = [0.0, 90.0, 90.0]

manipReqPoseQ = eulerToQuaternion(manipReqPoseE)

manipReqPose = [manipReqPoseP[0],manipReqPoseP[1],manipReqPoseP[2],manipReqPoseQ[0],manipReqPoseQ[1],manipReqPoseQ[2],manipReqPoseQ[3]]

while not rospy.is_shutdown():
	
	group.get_current_pose()
	pose = Pose()
	setManipPose(pose, manipReqPose)
	group.set_pose_target(pose)
	group.go()	
	time.sleep(0.1)
