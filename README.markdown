# Gazebo + MoveIt!
* Please use version 1.1.10
* http://wiki.ros.org/ur_gazebo
  * To launch the simulated arm and a controller for it, run:
    `$ roslaunch ur_gazebo ur10.launch`
  * `$ roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true`
  * To control the simulated arm from RViz, also run:
    `$ roslaunch ur10_moveit_config moveit_rviz.launch config:=true`
