## Setting up an Amazon picking challenge simulation with a UR10 robot arm in ROS / Gazebo.

### Step 1 - Install ROS, Gazebo and the Universal Robots packages


### Step 2 - Build the simulation

Follow this tutorial: https://www.youtube.com/watch?v=ayp87SjrwPc

### Step 3 - Setup MoveIt

To install MoveIt use the following with your ROS distro inserted:
```
$ sudo apt install ros-<distro>-moveit
```

Then do to the `catkin_ws/src/amazonsim/` folder and start the MoveIt setup assistant

```
$ roslaunch moveit_setup_assistant setup_assistant.launch
```

Follow the setup instructions in this video: https://www.youtube.com/watch?v=j6bBxfD_bYs

You should now have the following files (see repo files to compare) in the newly setup MoveIt config folder.

```
controllers.yaml
joint_names.yaml
myur10_moveit_controller_manager.launch.xml
myur10_planning_execution.launch
```

To launch the RViz window which runs the planner, we first launch the Gazebo simulation to startup the ROS nodes and topics for our simulation, then we launch the MoveIt MotionPlanner.
```
$ roslaunch myur10sim_description myur10.launch
$ roslaunch myur10_moveit_config myur10_planning_execution.launch
```

This launches both the Gazebo simulation and the RViz planning tool. To do a quick test that all is working, 'Add' the RobotModeln and 'Add' MotionPlanning to RViz. Set two states to move between (e.g. AllZeros to Home), hit Plan, and then hit Execute. When Executing, the robot in both RViz and Gazebo should move.

### Step 4 - Control the robot

So we now want to control the robot using python node. To do so we setup a python launch file and a python ros node using rospy. This is done using the moveit package and based on the following documentation: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

Setup the python launch file by creating `myur10_move_group_python.launch` in the `myur10_moveit_config/launch` folder (see GitHub files for the content).

Setup the python file by creating a scripts folder under `myur10_moveit_config` within which the file `cartesian_controller.py` should be created (see GitHub files for the content). This can be used to control the robot in the cartesian plane. Create another file called 'master_controller.py' and again take the code from the github files. This one creates a node which publishes random x_coordinates for the robot to end effector to move to.

Now we can run that python file to control the robot (note there are some dependant python packages that may need installing - pip install.... etc).

In two seperate terminal windows run the following (note a launch file could be built to action this from a single terminal):

```
$ rosrun myur10_moveit_config master_controller.py

$ rosrun myur10_moveit_config cartesian_controller.py
```

Switching back to our Gazebo and RViz windows, we will now see the robot planning and then moving to the random positions. Note that if there is no possible path due to an dd randomised x_value, the movement will be aborted.

