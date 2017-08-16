#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

# http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html#planning-to-a-pose-goal
# Please do these before running this script:
# roslaunch ur_gazebo ur10.launch
# roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
# roslaunch ur10_moveit_config moveit_rviz.launch config:=true

def move_group_python_interface_tutorial():

    # 1. Setup
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # XXX: you have to launch move_group to continue
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    # arm_group = moveit_commander.MoveGroupCommander("arm")
    # gripper_group = moveit_commander.MoveGroupCommander("endeffector")

    display_trajectory_publisher = rospy.Publisher(\
            '/move_group/display_planned_path',\
            moveit_msgs.msg.DisplayTrajectory,\
            queue_size=20)
    # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    # print "============ Waiting for RVIZ..."
    # rospy.sleep(10)
    # print "============ Starting tutorial "

    # 2. Getting Basic Information
    print "============ Reference frame: %s" % arm_group.get_planning_frame()
    print "============ End effector: %s" % arm_group.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    # 3. Plan
    # (1) Planning to an end effector transformation
    """
    arm_group.clear_pose_targets()
    print "============ Generating plan 1"
    cur_robot_pose = arm_group.get_current_pose().pose
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = -7.0921250634e-07
    pose_target.orientation.y = -7.06001727103e-07
    pose_target.orientation.z = 0.707173275016
    pose_target.orientation.w = 0.707040281103
    pose_target.position.x = -1.91382481231e-05
    pose_target.position.y = 0.256141003146
    pose_target.position.z = 1.42729999842

    arm_group.set_pose_target(pose_target)
    # Now, we call the planner to compute the plan and visualize it 
    # if successful Note that we are just planning, 
    # not asking move_group to actually move the robot
    plan1 = arm_group.plan()
    # print "============ Waiting while RVIZ displays plan1..."
    # rospy.sleep(5)
    # You can ask RVIZ to visualize a plan (aka trajectory) for you. 
    # But the arm_group.plan() method does this automatically 
    # so this is not that useful here 
    # (it just displays the same trajectory again).
    # print "============ Visualizing plan1"
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan1)
    # display_trajectory_publisher.publish(display_trajectory);
    # print "============ Waiting while plan1 is visualized (again)..."
    # rospy.sleep(5)

    # Execution
    # Use the following line when working with a real robot
    # arm_group.go(wait=True)
    # Use execute instead if you would like the robot to follow
    # the plan that has already been computed
    ret_val = arm_group.execute(plan1)
    if ret_val == True:
        print "Success"
    else:
        print "Fail"
    """

    # (2) Planning to a joint-space goal
    """
    arm_group.clear_pose_targets()
    print "============ Generating plan to a desired pose for the end-effector"
    group_variable_values = arm_group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values
    group_variable_values[1] = group_variable_values[1]+0.5
    arm_group.set_joint_value_target(copy.deepcopy(group_variable_values))
    plan2 = arm_group.plan()
    # arm_group.go(wait=True)
    ret_val = arm_group.execute(plan2)
    if ret_val == True:
        print "Success"
    else:
        print "Fail"
    """
 

    # (3) Cartesian Paths
    """
    # You can plan a cartesian path directly by specifying a list of waypoints 
    # for the end-effector to go through.
    waypoints = []

    # start with the current pose
    starting_point = arm_group.get_current_pose().pose

    # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/
    # XXX: We cannot include the current pose in waypoints,
    # otherwise an error of "Trajectory message contains waypoints 
    # that are not strictly increasing in time" will raise.
    # waypoints.append(arm_group.get_current_pose().pose)

    # first orient gripper and move backward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = starting_point.orientation.w
    wpose.orientation.x = starting_point.orientation.x
    wpose.orientation.y = starting_point.orientation.y
    wpose.orientation.z = starting_point.orientation.z
    wpose.position.x = starting_point.position.x - 0.1
    wpose.position.y = starting_point.position.y
    wpose.position.z = starting_point.position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move up
    wpose.position.z += 0.10
    waypoints.append(copy.deepcopy(wpose))

    # third move back again
    wpose.position.x -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    # fourth move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    # We want the cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in cartesian
    # translation.  We will specify the jump threshold as 0.0, effectively
    # disabling it.
    (plan3, fraction) = arm_group.compute_cartesian_path(\
            # waypoints to follow
            waypoints,\
            # eef_step
            0.01,\
            # jump_threshold
            0.0)
    print "============ Waiting while RVIZ displays plan3..."
    # rospy.sleep(5)

    # execute this plan on a real robot or a robot in gazebo
    ret_val = arm_group.execute(plan3)
    if ret_val == True:
        print "Success"
    else:
        print "Fail"
    """
    
    import IPython; IPython.embed()

    # 4. Adding/Removing Objects and Attaching/Detaching Objects
    # First, we will define the collision object message
    collision_object = moveit_msgs.msg.CollisionObject()

    # 5. When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"


if __name__=='__main__':
    # try:
    move_group_python_interface_tutorial()
    # except rospy.ROSInterruptException:
        # pass
