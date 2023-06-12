#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
import math
import tf2_ros
from tf2_geometry_msgs import PoseStamped 

#for cartesian controller
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group_display_planned_path', moveit_msgs.msg.DisplayTrajectory)


def callback(data):
    rospy.loginfo("I heard")
    data.markers[0].pose.position.x

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    p=PoseStamped()
    p.header.frame_id="camera_rgb_optical_frame"
    p.pose.position.x=data.markers[0].pose.position.x
    p.pose.position.y=data.markers[0].pose.position.y
    p.pose.position.z=data.markers[0].pose.position.z

    p.pose.orientation.x=data.markers[0].pose.orientation.x
    p.pose.orientation.y=data.markers[0].pose.orientation.y
    p.pose.orientation.z=data.markers[0].pose.orientation.z
    p.pose.orientation.w=data.markers[0].pose.orientation.w
    print(p)
    c=1
    rate = rospy.Rate(10.0)
    while c==1:
        try:
            #sembra funzionare
            #se rimuovi la posa da "p", lui usa tuto 0 e quindi ti trova la posizione camera rispetto al world
            #usiamo world perche il controllo  fatto in world
            trans = tfBuffer.transform(p, "world", rospy.Duration(1))
            print(trans)
            c=2
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rate.sleep()
    print(trans.pose.position)
    pose_target = geometry_msgs.msg.Pose()
    #red line -> x, gli oggetti sono sull'asse X con Y=0 rispetto al world
    pose_target.position.x = trans.pose.position.x
    pose_target.position.y = trans.pose.position.y
    pose_target.position.z = trans.pose.position.z+0.3

    pose_target.orientation.x =  trans.pose.orientation.x
    pose_target.orientation.y =  trans.pose.orientation.y
    pose_target.orientation.z =  trans.pose.orientation.z
    pose_target.orientation.w =  trans.pose.orientation.w
    group.set_pose_target(pose_target)

	# Call the planner to compute and execute the plan
    plan = group.go(wait=True)
    group.stop()	# Stop any residual movement
    group.clear_pose_targets()	# Clear pose targets

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()