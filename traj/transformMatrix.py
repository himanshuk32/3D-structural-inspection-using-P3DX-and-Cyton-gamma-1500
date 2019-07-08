#!/usr/bin/env python

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
rospy.init_node('generateMap',anonymous=True)
group = moveit_commander.MoveGroupCommander("cyton1500_group")


class matrix:

    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise (ValueError, "Invalid size of matrix")
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise (ValueError, "Invalid size of matrix")
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise (ValueError, "Matrices must be of equal dimensions to add")
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise (ValueError, "Matrices must be of equal dimensions to subtract")
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise (ValueError, "Matrices must be m*n and n*p to multiply")
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i]) ** 2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise (ValueError, "Matrix not positive-definite")
                res.value[i][i] = (d)**(1/2.0)
            for j in range(i + 1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                    res.value[i][j] = (self.value[i][j] - S) / res.value[i][i]
                except:
                    raise (ValueError, "Zero diagonal")
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k] * res.value[j][k] for k in range(j + 1, self.dimx)])
            res.value[j][j] = 1.0 / tjj ** 2 - S / tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum(
                    [self.value[i][k] * res.value[k][j] for k in range(i + 1, self.dimx)]) / self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)



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
