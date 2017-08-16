# Gazebo + MoveIt!
* Please use version 1.1.10
* http://wiki.ros.org/ur_gazebo
  * To launch the simulated arm and a controller for it, run:
    `$ roslaunch ur_gazebo ur10.launch`
  * `$ roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true`
  * To control the simulated arm from RViz, also run:
    `$ roslaunch ur10_moveit_config moveit_rviz.launch config:=true`

# Move the Arm in Simulation
1 You can move the arm directly by instantiating an *actionlib* client, and send joint commands as `control_msgs.msg.FollowJointTrajectoryGoal` to *actionlib* server in namespace `/arm_controller/follow_joint_trajectory`, simulate the motion in *gazebo*, view the motion in *rviz*.
  * `./universal_robot/tests/simulate_ur10_by_sending_joint_command_via_action_client.py`

2 You can also move the arm by interacting with `moveit_commander.MoveGroupCommander` and send requests as *end effector transformation*, *goal pose (joint-space goal)*, or *cartesian path* to robot controller, simulate the motion in *gazebo*, and view the motion in *rviz*.
  * `./universal_robot/tests/simulate_ur10_by_sending_joint_command_via_MoveIt.py`


# Issues
* If you encounter a **PATH_TOLERANCE_VIOLATED** or a **GOAL_TOLERANCE_VIOLATED**, refer to the path and goal constraints which are configured in `./universal_robot/ur_gazebo/controller/arm_controller_ur10.yaml`.
