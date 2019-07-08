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

pi = 3.141592635

debug = False
simulating = True
mobile = False

robotBaseName = "Pioneer 3DX"
rospy.init_node('manipCurrentPose',anonymous=True)
group = moveit_commander.MoveGroupCommander("cyton1500_group")

while not rospy.is_shutdown():
	
	manipCurrentPose = group.get_current_pose().pose
	ang_z = float((manipCurrentPose.orientation.z*180.0)/pi)
	ang_x = float((manipCurrentPose.orientation.x*180.0)/pi)	
	ang_y = float((manipCurrentPose.orientation.y*180.0)/pi)	
	print " manipulator's  X = ",manipCurrentPose.position.x,"  Y = ",manipCurrentPose.position.y, "  Z = ",manipCurrentPose.position.z
	#print "  angle X = ",ang_x," angle Y = ",ang_y, " angle Z = ",ang_z 
	time.sleep(0.1)
