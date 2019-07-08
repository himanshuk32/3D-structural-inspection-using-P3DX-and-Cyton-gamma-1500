#!/usr/bin/env python

# imports

import rospy
import geometry_msgs.msg
import time
import numpy
import sys
import os
import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from math import *

rospy.init_node('botPose',anonymous=True)

pi = 3.1415926

def callBack(odomMessage):
	quaternion = (
	    odomMessage.pose.pose.orientation.x,
	    odomMessage.pose.pose.orientation.y,
	    odomMessage.pose.pose.orientation.z,
	    odomMessage.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	alpha = yaw*180.0/pi
	print " botX = ", odomMessage.pose.pose.position.x, " botY = ", odomMessage.pose.pose.position.y, " yaw = ", alpha



odomSubscriber = rospy.Subscriber('/fused_bot/odom', Odometry, callBack)
rospy.spin()
