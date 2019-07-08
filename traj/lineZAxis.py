#!/usr/bin/env python

# imports /home/himanshu/catkin_ws/src/P3DX-and-Cyton-gamma-1500/Walden95-pioneer-3dx-and-cyton-gamma-1500-mobile-manipulator-50bd44e00ebf/base-arm_fusion/p3dx_cyton1500_fusion/xacro

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

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

velPub = rospy.Publisher('/fused_bot/cmd_vel', Twist,queue_size=10)

def stopBot():
        velPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

def setManipPose(pose, whichPose):
	[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] = whichPose

debug = False
simulating = True
mobile = False

robotBaseName = "Pioneer 3DX"
rospy.init_node('lineZAxis',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("cyton1500_group")

defaultPose = [-0.00798111114703, -0.00240289023425, 0.973910607185, 3.55992334659e-05, 2.53591989399e-05, 0.707060385227, 0.707153172752]
manipPose1 = [0.441024004555, -0.0157218288562, 0.402612554019, 0.527145535299, 0.477387854476, 0.486045011226, 0.50791600494]
manipPose2 = [0.01, 0.3, 0.55, 0, 0, 0, 0.707]

pose = Pose()
setManipPose(pose, manipPose1)
posee = defaultPose
time.sleep(0.75)
print "outside loop"
n = -1
sign = 1
while not rospy.is_shutdown():
	if pose.position.z > 0.81:
	    sign = -1
	if pose.position.z < 0.55:
	    sign =  1
	n = n + sign
	group.get_current_pose()
	pose = Pose()
	setManipPose(pose, manipPose2)
	pose.position.z = pose.position.z + 0.075*n
	group.set_pose_target(pose)
	group.go()
        time.sleep(1.5)

