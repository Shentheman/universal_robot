#! /usr/bin/env python
import rospy,sys,copy

# Brings in the SimpleActionClient
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# http://gazebosim.org/tutorials?tut=drcsim_ros_cmds&cat=
# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29
# http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('send_traj_msg')

    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

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
    goal.trajectory.joint_names = ['elbow_joint',\
            'shoulder_lift_joint', 'shoulder_pan_joint',\
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    goal.trajectory.points = []
    point0 = JointTrajectoryPoint()
    point0.positions = [0.0,0.0,0.0,0.0,0.0,0.0]
    point0.velocities = [0.0,0.0,0.0,0.0,0.0,0.0]
    # To be reached 1 second after starting along the trajectory
    point0.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point0)

    for i in range(10):
        point = copy.deepcopy(point0)
        point.positions[2] = 0.2*i
        point.positions[-2] = 0.2*i
        point.time_from_start = rospy.Duration(i*0.2+2)
        goal.trajectory.points.append(point)
    
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    print goal
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    result = client.get_result()
    import IPython;IPython.embed()

