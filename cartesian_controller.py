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



	# Choose end effector pose position in cartesian coordinates
pose_target = geometry_msgs.msg.Pose()
#red line -> x, gli oggetti sono sull'asse X con Y=0 rispetto al world
pose_target.position.x =0.99
pose_target.position.y = 0.43
pose_target.position.z = 0.5

pose_target.orientation.x =  -0.0115700064242
pose_target.orientation.y =   0.0423873654921
pose_target.orientation.z =   0.987913591677
pose_target.orientation.w =    -0.148647844176


group.set_pose_target(pose_target)

	# Call the planner to compute and execute the plan
plan = group.go(wait=True)
group.stop()	# Stop any residual movement
group.clear_pose_targets()	# Clear pose targets
