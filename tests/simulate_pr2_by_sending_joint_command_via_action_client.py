#! /usr/bin/env python
import rospy,sys

# Brings in the SimpleActionClient
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('send_traj_msg')

    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/l_arm_controller', FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(rospy.Duration(10.0))

    # verify joint trajectory action servers are available
    client_server = client.wait_for_server(rospy.Duration(10.0))
    if not client_server:
        msg = ("Action server not available."
               " Verify action server availability.")
        rospy.logerr(msg)
        rospy.signal_shutdown(msg)
        sys.exit(1)

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']

    goal.trajectory.points = []
    point0 = JointTrajectoryPoint()
    point0.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point0.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    # To be reached 1 second after starting along the trajectory
    point0.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point0)

    point1 = JointTrajectoryPoint()
    point1.positions = [-0.3,0.2,-0.1,-1.2,1.5,0.5,0.0]
    point1.positions = [0.0,0.0,0.0,0.0,0.0,0.1,0.0]
    point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0]
    point1.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(point1)

    point2 = JointTrajectoryPoint()
    point2.positions = [0.0,0.1,0.0,0.0,0.0,0.1,0.0]
    point2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point2.time_from_start = rospy.Duration(3.0)
    goal.trajectory.points.append(point2)


    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    print goal
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()  # A FibonacciResult
    # print("Result:", ', '.join([str(n) for n in result.sequence]))
    import IPython;IPython.embed()

