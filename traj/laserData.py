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
import os

from sensor_msgs.msg import LaserScan
def laserCallback(msg):
    print float((int(msg.ranges[0]*1000))/1000.0)

rospy.init_node('laserData',anonymous=True)

sub = rospy.Subscriber('/fused_bot/QS18VP6LLP_laser/scan', LaserScan, laserCallback)
rospy.spin()
