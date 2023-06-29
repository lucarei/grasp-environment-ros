#!/usr/bin/env python


# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64

x_value = 0

# Create a ROS node called 'move_group_python_interface'
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface',anonymous=True)

# Setup the moveit control parameters
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group_display_planned_path', moveit_msgs.msg.DisplayTrajectory)

while not rospy.is_shutdown():
	# Create callback which is automatically called when x_value data is receieved
	def callback(data):
		global x_value
		x_value = data.dataprint("Moving x_position to:", x_value)
				
	# Get data from the publishing node for x_value
	rospy.Subscriber("x_value", Float64, callback)	

	# Choose end effector pose position in cartesian coordinates
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1.0
	pose_target.position.x = x_value
	pose_target.position.y = 0
	pose_target.position.z = 1.0
	group.set_pose_target(pose_target)

	# Call the planner to compute and execute the plan
	plan = group.go(wait=True)
	group.stop()	# Stop any residual movement
	group.clear_pose_targets()	# Clear pose targets

