# The following is a very crappy demo which is mostly hard-coded and dosen't interface with the diffriantal drive with its python interface

import numpy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import time
import matplotlib.pyplot as plt

import os
import time

debug = False
simulating = True
mobile = False

baseSpeed = 0.5

defaultPose = [-0.00798111114703, -0.00240289023425, 0.973910607185, 3.55992334659e-05, 2.53591989399e-05, 0.707060385227, 0.707153172752]
searchPose_Top0 = [0.441024004555, -0.0157218288562, 0.402612554019, 0.527145535299, 0.477387854476, 0.486045011226, 0.50791600494]
searchPose_Transition0 = [0.453105823538, 0.152369814563, 0.401565616327, 0.0373625654534, 0.710454970009, -0.0127965415602, 0.702633633422]

robotBaseName = "Pioneer 3DX"
rospy.init_node('controlCyton',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("cyton1500_group")

os.system("rostopic pub -1 fused_bot/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.5]'")
time.sleep(0.15)
os.system("rostopic pub -1 fused_bot/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
speed = 0.5;

while not rospy.is_shutdown():
	group.get_current_pose()
	pose = Pose()
	[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] = searchPose_Top0
	group.set_pose_target(pose)
	group.go()
	time.sleep(1.00)
	
	[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] = searchPose_Transition0
	group.set_pose_target(pose)
	group.go()
	time.sleep(1.00)
	
	[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] = defaultPose
	group.set_pose_target(pose)
	group.go()
	time.sleep(1.00)

        os.system("rostopic pub -1 fused_bot/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.5]'")
        time.sleep(0.05)
	os.system("rostopic pub -1 fused_bot/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
	
	time.sleep(1.00)




    
