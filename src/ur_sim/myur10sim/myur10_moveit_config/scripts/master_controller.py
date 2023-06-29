#!/usr/bin/env python

# Master controller node
# A test node to understand the functionality of intercommunicating nodes in ROS when
# controlling a UR10 robot arm.

import rospy
from std_msgs.msg import Float64
import math
import random

RATE = 0.2	# Publishing rate per second - i.e. publish once every 5 seconds

if __name__ == '__main__':
	# Initiate node
	rospy.init_node('publsher')

	# Set the publishing settings for this ROS node
	pub = rospy.Publisher('x_value', Float64, queue_size=1)
	rate = rospy.Rate(RATE)
	step = 0

	 
	while not rospy.is_shutdown():
		# Generate random x value
		x_value = random.uniform(0, 1.5)

		# Publish the value
		pub.publish(x_value)
		step = step + 1

		# Wait to match the rate
		rate.sleep()

