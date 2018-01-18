#!/usr/bin/env python
import rospy

import actionlib  # handles action things
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import threading


class FloorActionClient(object):
    """Example of controlling the floor robot using joint trajectory action."""
    def __init__(self):
        rospy.init_node("floor_action_client", anonymous=True)
        # We need some info on the joint_states
        rospy.Subscriber("/floor/joint_states",  # To know where we are
                         JointState,  # Message type
                         self.messageCallback)

        self.joint_positions = [0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0]

        # One problem with rospy subscribers is that they work as a separate
        # thread. I think this means we should ensure that joint_positions
        # are free to be edited when we want to. This means using a lock
        # from the threading library.
        self.lock = threading.Lock()

        # We need to name our joints
        self.joint_names = [
            "floor_joint_a1",
            "floor_joint_a2",
            "floor_joint_a3",
            "floor_joint_a4",
            "floor_joint_a5",
            "floor_joint_a6"
        ]

        # Now let's setup the action client
        ac = actionlib.SimpleActionClient("/floor/joint_trajectory_action",
                                          FollowJointTrajectoryAction)
        self.action_client = ac

        # Wait for the server (joint controllers) to initialize
        self.action_client.wait_for_server()

    def messageCallback(self, msg):
        """That which handles joint_state messages"""
        with self.lock:
            self.joint_positions = list(msg.position)

    def goToPosition(self, joint_goal, dur, goal_tolerance=1.0):
        """Packages a joint goal into a trajectory thingy"""
        # Define the look and feel of the trajectory
        tr = JointTrajectory()
        tr.joint_names = self.joint_names
        tr.points.append(JointTrajectoryPoint())
        tr.points[0].positions = joint_goal
        tr.points[0].velocities = [0.0 for i in self.joint_names]
        tr.points[0].accelerations = [0.0 for i in self.joint_names]
        tr.points[0].time_from_start = rospy.Duration(dur)

        # Set it as a goal (separate object)
        tr_goal = FollowJointTrajectoryGoal()
        tr_goal.trajectory = tr
        tr_goal.goal_time_tolerance = rospy.Duration(goal_tolerance)
        self.action_client.send_goal(tr_goal)

    def getCurrentPosition(self):
        """Thread safe way of getting joint positions"""
        with self.lock:
            return self.joint_positions


if __name__ == "__main__":
    try:
        # Start our object
        fac = FloorActionClient()

        # Wait until there are joint_states to listen to
        rospy.wait_for_message("/floor/joint_states", JointState, timeout=None)

        # Let's move joint A3 0.5 rads back and forth
        currj = fac.getCurrentPosition()
        desj = [i for i in currj]  # shallow copy
        desj[2] = desj[2] + 0.2
        rospy.loginfo("We are at \n" + str(currj))
        rospy.loginfo("We'll go to\n" + str(desj))

        rospy.loginfo("Packaging and sending goal")
        fac.goToPosition(joint_goal=desj, dur=10.0)

        rospy.loginfo("Waiting for results")
        fac.action_client.wait_for_result()
        currj = fac.getCurrentPosition()
        rospy.loginfo("Ended up at\n"+str(currj))
        rospy.sleep(1)

        rospy.loginfo("Let's return")
        currj = fac.getCurrentPosition()
        desj = [i for i in currj]
        desj[2] = desj[2] - 0.2
        fac.goToPosition(joint_goal=desj, dur=10.0)
        fac.action_client.wait_for_result()
        currj = fac.getCurrentPosition()
        rospy.loginfo("Returned to \n" + str(currj))

        rospy.sleep(5)
        rospy.loginfo("Let's repeat, but now we'll use cancel_goal to interrupt")
        currj = fac.getCurrentPosition()
        desj = [i for i in currj]
        desj[2] = desj[2] + 0.2
        rospy.loginfo("We want to go to\n" + str(desj))
        fac.goToPosition(joint_goal=desj, dur=10.0)
        
        rospy.sleep(5)
        fac.action_client.cancel_goal()
        currj = fac.getCurrentPosition()
        rospy.loginfo("Goal interrupted. And we are at\n"+str(currj))
        rospy.sleep(5)

        rospy.loginfo("Let's try again and \"interrupt\" by sending a new goal.")
        currj = fac.getCurrentPosition()
        desj = [i for i in currj]
        desj[2] = desj[2] + 0.2
        rospy.loginfo("We want to go to\n"+str(desj))
        fac.goToPosition(joint_goal=desj, dur=10.0)

        rospy.sleep(3)
        rospy.loginfo("Sending \"interrupting\" goal!")
        currj = fac.getCurrentPosition()
        desj = [i for i in currj]
        desj[2] = desj[2] - 0.2
        fac.goToPosition(joint_goal=desj, dur=20.0)
        rospy.loginfo("Goal sent at \n"+str(currj))

        rospy.sleep(30)
        currj = fac.getCurrentPosition()
        rospy.loginfo("And at the end we are at\n"+str(currj))
        # Wait until ctrl-c
        rospy.spin()
        
    except rospy.ROSInterruptException:
        # This stuff is to ensure we get the ctrl-c and treat it well
        print("Killed")
