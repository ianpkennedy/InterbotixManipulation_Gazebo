### Part I


In part I of the homework, a flip node flips a user constructed robot using a differential gazebo plugin within custom world (in the worlds folder) built in gazebo. It publishes to cmd_vel, and switches polarity at fixed intervals. The robot description is ddrive.urdf.xacro. Physical parameters for the robot are loaded from  /config/param.yaml  .  view.rviz provides a basic view of the robot, and view2.rviz provides an odom view of the robot with odometry. 

To view the robot in rviz, launch the following:

`roslaunch diff_drive ddrive_rviz.launch`

To view the robot with a joint_state_publisher_gui, type the following:

`roslaunch diff_drive ddrive_rviz.launch gui:=true`


In order to see the robot in gazebo do some flips and view robot odometry in Rviz, do the following in a terminal. Be sure to unpause gazebo so that the simulation and rviz window run the following, which will load a custom gazebo world environment for the robot (from /worlds/ddrive.world):

`roslaunch diff_drive ddrive.launch`


A video of the robot performing the task in gazebo can be seen here: 
https://drive.google.com/file/d/1yHBLn0aciDDX4AcN6VDNNo0hOAP1hzuf/view?usp=sharing


https://github.com/ianpkennedy/ROS_InterbotixManipulation_Gazebo/blob/main/diff_drive/example.gif

![alt-text](https://github.com/ianpkennedy/ROS_InterbotixManipulation_Gazebo/blob/main/diff_drive/example.gif)

Sources:
For testing out vehicle dynamics in gazebo, http://wiki.ros.org/teleop_twist_keyboard (accessed 10/28)

 https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py (accessed 10/28)
