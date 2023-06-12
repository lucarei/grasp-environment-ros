#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import pi
import copy
import moveit_msgs.msg

if __name__ == '__main__':

  rospy.init_node("move_group_BIOBLU_picknPLACE", anonymous=True)
  
  moveit_commander.roscpp_initialize(sys.argv)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group_name = "manipulator"
  group = moveit_commander.MoveGroupCommander(group_name)
  display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

  planning_frame = group.get_planning_frame()
  print(planning_frame)

  print(robot.get_current_state())

  joint_goal = group.get_current_joint_values()

  joint_goal[3]=0

  group.set_joint_value_target(joint_goal)
  plan2 = group.plan()
  print("============ Waiting while RVIZ displays plan2...")
  rospy.sleep(5)

  print("Sending goal...")
  group.go(joint_goal, wait=True)
  print("Goal successfull!")

  group.stop()






