#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# init moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

# instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# instantiate a MoveGroupCommander object
group_name="manipulator"
move_group=moveit_commander.MoveGroupCommander(group_name)

# Create DisplayTrajectory ROS publishder wihich is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planmned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

# Get basic information
planning_frame = move_group.get_planning_frame()
print('Planning frame: %s' % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "======== End effector link: %s" % eef_link

# We can get a list of all groups in the robot:
group_names = robot.get_group_names()
print "======== Available Planning Groups:", robot.get_group_names()

# Sometimes for debuggins it is usefulm to print the entire state of the robot
print "========Printing robot state"
print robot.get_current_state()
print "========"

# We can get the joint values from the group and sdjust some of the values:
joint_current = move_group.get_current_joint_values()

print('Current Joint values:', joint_current)
print('Enter joint angles in radians')
joint_goal = [int(input("Enter_angle: ")) for i in range(6)]
print('New goals for the robot:', joint_goal)

# The go command can be called with joint values, poses, or without any parameters if
# you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ''stop()'' to ensure no residual movement
move_group.stop()
