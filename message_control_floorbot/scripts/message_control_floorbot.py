#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest


import threading
import numpy as np


class FloorController(object):
    """Example of controlling the floor robot using joint position commands.
    WARNING: Giving sane speeds and behavior is entirely up to you."""
    def __init__(self):
        rospy.init_node("floor_message_controller", anonymous=True)
        self.rate = rospy.Rate(80.0)  # Publishing rate
        # We need some info on the joint_states
        self.sub = rospy.Subscriber("/floor/joint_states",
                                    JointState,
                                    self.jointStateCallback)
        self.joint_states = [0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0]
        # As in action_control_floorbot_client, I think we have to use locks to
        # ensure safety of the joint_states
        self.lock = threading.Lock()

        # It's always good to name our joints, and this time we'll do it using
        # rosparam. In the config file:
        # robotlab_support/config/floor/controller_joint_names.yaml
        # we have set the joint names already. And they've been loaded into
        # rosparam in the load_rsi.launch. Thus it's easier to just read them.
        self.joint_names = rospy.get_param("/floor/controller_joint_names")

        # Now let's setup the topic publisher:
        self.pub = rospy.Publisher("/floor/joint_position_controller/command",
                                   Float64MultiArray,
                                   queue_size=2)

    def jointStateCallback(self, msg):
        """When a joint_state message is published, this guy gets it."""
        with self.lock:
            self.joint_states = list(msg.position)

    def getJointState(self):
        with self.lock:
            return self.joint_states

    def goToPosition(self, joint_goal, dur):
        """Interpolates from current state to joint_goal and publishes."""
        currj = np.asarray(self.getJointState())
        rospy.loginfo("Going from "+str(currj))
        rospy.loginfo("Going to "+str(joint_goal))
        goalj = np.asarray(joint_goal)
        displacementj = goalj - currj
        nsteps = int((dur*80.0))  # Controller operates at 80 Hz
        dj = displacementj/nsteps
        desiredj = currj
        for i in xrange(nsteps):
            desiredj = desiredj + dj  # Desired single step
            command = Float64MultiArray()
            command.data = list(desiredj)
            self.pub.publish(command)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        # This guy has our rospy.init_node, run him first
        fc = FloorController()
        rospy.loginfo("Message_control_floorbot.py takes a bit of space")
        rospy.loginfo("Make sure floorbot won't collide with something!!!")
        #  Block until joint_states becomes available:
        rospy.loginfo("Waiting for /floor/joint_states message")
        rospy.wait_for_message("/floor/joint_states", JointState, timeout=None)

        # First let's switch the active controller.
        # The default controller loaded by load_rsi.launch in
        # robot_streaming_interface.launch is
        # position_trajectory_controller
        rospy.loginfo("Waiting for switchcontroller service")
        rospy.wait_for_service("/floor/controller_manager/switch_controller")

        # make a callable Service Proxy object (rospy interface to service)
        s = rospy.ServiceProxy("/floor/controller_manager/switch_controller",
                               SwitchController)
        # Call the switch
        resp = s.call(SwitchControllerRequest(["joint_position_controller"],  # Start these guys
                                              ["position_trajectory_controller"],  # stop these guys
                                              SwitchControllerRequest.STRICT))
        # Strictness is either STRICT or BEST_EFFORT
        # STRICT: switching will fail and result in a no-op if anything goes
        # wrong (an invalid controller name, a controller that failed to
        # start, etc. )
        # BEST_EFFORT: means that even when something goes wrong with on
        # controller, the service will still try to start/stop the remaining
        # controllers.

        if resp.ok == 1:
            rospy.loginfo("Controller switched!")
        else:
            rospy.loginfo("Controller not switched")
            quit()

        rospy.loginfo("!!!! Get ready on the E-stop !!!!")
        rospy.sleep(10)

        currj = fc.getJointState()
        startj = [i for i in currj]  # shallow copy
        desj = [i for i in currj]  # shallow copy
        desj[2] = desj[2] + 0.2
        fc.goToPosition(joint_goal=desj, dur=10.0)

        # Where'd we go?
        currj = fc.getJointState()
        rospy.loginfo("Ended up at \n"+str(currj))
        rospy.sleep(2)

        # Return
        rospy.loginfo("Let's return!")
        currj = fc.getJointState()
        desj = [i for i in currj]
        desj[2] = currj[2] - 0.2
        fc.goToPosition(joint_goal=desj, dur=5.0)

        currj = fc.getJointState()
        desj = [i for i in currj]  # shallow copy
        desj[2] = desj[2] + 0.2
        fc.goToPosition(joint_goal=desj, dur=2.5)

        # Return
        rospy.loginfo("Let's return!")
        currj = fc.getJointState()
        desj = [i for i in currj]
        desj[2] = currj[2] - 0.2
        fc.goToPosition(joint_goal=desj, dur=1.25)  # Approx. limit
        rospy.sleep(5)

        # More movements
        rospy.loginfo("Let's move some more")
        rospy.sleep(1)
        currj = fc.getJointState()
        desj = [i for i in currj]
        desj[0] = desj[0] - 0.3
        desj[1] = desj[1] + 0.3
        desj[2] = desj[2] - 0.3
        desj[3] = desj[3] + 0.3
        fc.goToPosition(joint_goal=desj, dur=10)
        rospy.sleep(1)

        # Return to start
        rospy.loginfo("Let's return to start")
        fc.goToPosition(joint_goal=startj, dur=5)
        rospy.sleep(10)

        # http://wiki.ros.org/rospy/Overview/Services#Calling_services
    except rospy.ROSInterruptException:
        print("Killed")
