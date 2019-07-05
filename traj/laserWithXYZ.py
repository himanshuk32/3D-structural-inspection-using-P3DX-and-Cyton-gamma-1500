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
rospy.init_node('observePoseAndLaser',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("cyton1500_group")

while not rospy.is_shutdown():
	
	group.get_current_pose()
	laserMessage = rospy.wait_for_message('/fused_bot/QS18VP6LLP_laser/scan', LaserScan, timeout = 0.1)
	odomMessage = rospy.wait_for_message('/fused_bot/odom', Odometry, timeout = 0.1)
	print "laserData  = ", laserMessage.ranges[0], "X = ", odomMessage.pose.pose.position.x, "Y = ", odomMessage.pose.pose.position.y
	time.sleep(0.1)
