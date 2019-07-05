#!/usr/bin/env python
import getch
import roslib#; roslib.load_manifest('traj')
import rospy


from geometry_msgs.msg import Twist

KEY_UP = 65
KEY_DOWN = 66
KEY_RIGHT = 67
KEY_LEFT = 68
USER_QUIT = 100

MAX_FORWARD = 0.75
MAX_LEFT = 0.25
MIN_FORWARD = -0.75
MIN_LEFT = -0.25

forward = 0.0
left = 0.0
keyPress = 0

while(keyPress != USER_QUIT):
	pub = rospy.Publisher('/fused_bot/cmd_vel', Twist)
	rospy.init_node('userToRosAria')

	twist = Twist()

	keyPress = getch.getArrow()

	if((keyPress == KEY_UP) and (forward <= MAX_FORWARD)):
		forward += 0.05
	elif((keyPress == KEY_DOWN) and (forward >= MIN_FORWARD)):
		forward -= 0.05
	elif((keyPress == KEY_LEFT) and (left <= MAX_LEFT)):
		left += 0.05
	elif((keyPress == KEY_RIGHT) and (left >= MIN_LEFT)):
		left -= 0.05

	# max backward/forward speed is 1.2 m/s
	if(forward > MAX_FORWARD):
		forward = MAX_FORWARD
	elif(forward < MIN_FORWARD):
		forward = MIN_FORWARD

	if(left > MAX_LEFT):
		left = MAX_LEFT
	elif(left < MIN_LEFT):
		left = MIN_LEFT

	twist.linear.x = forward
	twist.angular.z = left
	pub.publish(twist)
	rospy.sleep(0.01)


pub = rospy.Publisher('/fused_bot/cmd_vel', Twist)
rospy.init_node('userToRosAria')
twist = Twist()
pub.publish(twist)
exit()
