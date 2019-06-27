#!/usr/bin/env python

# imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time

# parameters
topic_cmd_vel = '/fused_bot/cmd_vel'
update_interval = 0.1 # [s]

vel_lin = input("Enter the linear velocity (in m/s): ")	
radius = input("Enter the radius (in metre): ")
vel_ang = vel_lin/radius

# launch node and create a publisher
rospy.init_node('Circle')
pub = rospy.Publisher(topic_cmd_vel, Twist,queue_size=10)

# loop until shutdown
while not rospy.is_shutdown():

	# publish the defined linear and angular velocity
	pub.publish(Twist(Vector3(vel_lin, 0, 0), Vector3(0, 0, vel_ang)))
	
	# sleep the defined interval
	time.sleep(update_interval)

