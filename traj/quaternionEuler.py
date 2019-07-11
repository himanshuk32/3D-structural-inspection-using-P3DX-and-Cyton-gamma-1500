#!/usr/bin/env python

# imports

import rospy
import geometry_msgs.msg
import numpy
import sys
import copy
import os
import tf

from math import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

odomMessage = Pose()
odomMessage.orientation.x = 0
odomMessage.orientation.y = 0
odomMessage.orientation.z = 0
odomMessage.orientation.w = 0.707

odomQuaternion = (
    odomMessage.orientation.x,
    odomMessage.orientation.y,
    odomMessage.orientation.z,
    odomMessage.orientation.w)
odomEuler = tf.transformations.euler_from_quaternion(odomQuaternion)
print "roll ", odomEuler[0], "  pitch ", odomEuler[1], "  yaw  ", odomEuler[2]
