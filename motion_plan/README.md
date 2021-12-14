### Part II

Ian Kennedy

This package deals with the motion planning used for operating an Interbotix px100 robot in the pick and place of an object while avoiding a box looking item. A preloaded list of working waypoints are preloaded, reflecing the geometry of the world provided in the world folder. The mover node provides reset, step, and follow services to reset the robot and planning scene, add waypoints to the trajectory, and run the trajectory for the pick and place operation. THe interbotix_ros_core , interbotic_ros_toolboxes and interbotix_ros_manipulators packages are required for this part. Unpause the gazebo simulation to be able to view the simulation when running gazebo. 


Some tests can be run from the top level of the workspace using:


`catkin_make run_tests`


In order to run the algorithm on a real robot, use the following command:

`roslaunch motion_plan arm.launch use_sim:=false use_actual:=true robot_model:=px100`


To run the algorithm in gazebo, perform the following:

`roslaunch motion_plan arm.launch use_sim:=true use_gazebo:=true robot_model:=px100 dof:=4`


To just run it with a fake moveit node, do the following:

`roslaunch motion_plan arm.launch use_sim:=true use_fake:=true robot_model:=px100 dof:=4`


A video of the robot running in gazebo can be seen here:

https://drive.google.com/file/d/17PfJBCpiLhK1-lG74WCfkLxjOEsUvfwg/view?usp=sharing

A video of the robot performing in real life:

https://drive.google.com/file/d/1HljNYAiFwejft1ccZNIAGVfBpaTXv_GH/view?usp=sharing




Sources:

        http://gazebosim.org/tutorials?tut=model_editor (accessed 10/29)


        http://wiki.ros.org/rviz/DisplayTypes/TF (accessed 10/30)