#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest

import threading
import numpy as np


class FloorController(object):
    """Example of controlling the floor robot using the callback routine.
    Basically when a joint_states topic comes, the callback triggers."""
    def __init__(self, rate=80.0):
        # We're switching things around. Now init_node will be called
        # outside the object. By doing this, multiple of these objects
        # can be created. Not that'd we'd want to. But we can!

        # Only used for dj calculation
        self.rate = rate  # Hz

        # Subscriber to the joint_states
        self.sub = rospy.Subscriber("/floor/joint_states",
                                    JointState,
                                    self.callbackControl)
        self.joint_states = [0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0]
        
        # Publisher to the joint position controller
        self.pub = rospy.Publisher("/floor/joint_position_controller/command",
                                   Float64MultiArray,
                                   queue_size=2)

        # Ensure thread safety
        self.lock = threading.Lock()

        # The callback may trigger before we have set up a goal
        self.goal_set = False

        # The maximum joint_rate that we can accept
        self.max_speed = 6.0  # rad/s
        # Don't go above. The hardware can take 7.0
        # but the simulator can't do higher than 6 ish.

        # The goal tolerance
        self.tolerance = 0.01  # rad

    def callbackControl(self, msg):
        # Thread safety
        with self.lock:
            currj = np.array(msg.position)
            self.joint_states = currj
            if self.goal_set:
                if np.linalg.norm(self.goalj-currj) > self.tolerance:
                    dj = (self.max_speed/float(self.rate))*self.deltaj
                    desiredj = currj + dj
                    command = Float64MultiArray()
                    command.data = desiredj
                    self.pub.publish(command)
                else:
                    self.goal_set = False
            else:
                pass

    def setGoal(self, goalj):
        with self.lock:
            self.goalj = np.array(goalj)
            self.deltaj = np.array(goalj) - self.joint_states
            self.goal_set = True

    def stop(self):
        with self.lock:
            self.goal_set = False

    def getJointState(self):
        with self.lock:
            return self.joint_states

    def getGoalSet(self):
        with self.lock:
            return self.goal_set

if __name__ == "__main__":
    try:
        rospy.init_node("callback_control_floorbot", anonymous=True)
        rospy.loginfo("Waiting for /floor/joint_states message")
        rospy.wait_for_message("/floor/joint_states", JointState, timeout=None)

        rospy.loginfo("Waiting for /floor/controller_manager/switch_controller")
        rospy.wait_for_service("/floor/controller_manager/switch_controller")

        # make a callable Service Proxy object (rospy interface to service)
        s = rospy.ServiceProxy("/floor/controller_manager/switch_controller",
                               SwitchController)
        # Call the switch
        resp = s.call(SwitchControllerRequest(["joint_position_controller"],  # Start these guys
                                              ["position_trajectory_controller"],  # stop these guys
                                              SwitchControllerRequest.STRICT))

        if resp.ok == 1:
            rospy.loginfo("Controller switched!")
        else:
            rospy.loginfo("Controller not switched!")
            quit()
        
        rospy.loginfo("Starting")
        
        fc = FloorController()
        rospy.sleep(5)
        currj = fc.getJointState()
        rospy.loginfo("We are at "+str(currj))
        goalj = [i for i in currj]  # shallow copy
        startj = [i for i in currj]  # shallow copy
        goalj[2] = goalj[2] + 0.3
        rospy.loginfo("Let's go to "+str(goalj))
        fc.setGoal(goalj)
        # We'd like to do more later, so we don't just rospy.spin()
        while fc.getGoalSet():
            rospy.sleep(0.1)
        
        currj = fc.getJointState()
        rospy.loginfo("Reached " + str(currj))
        rospy.sleep(2)
        
        
        currj = fc.getJointState()
        rospy.loginfo("Let's return!")
        goalj = [i for i in currj]  # shallow copy
        startj = [i for i in currj]  # shallow copy
        goalj[2] = goalj[2] - 0.3
        rospy.loginfo("Let's go to "+str(goalj))
        fc.setGoal(goalj)
        # We'd like to do more later, so we don't just rospy.spin()
        while fc.getGoalSet():
            rospy.sleep(0.1)
        
        currj = fc.getJointState()
        rospy.loginfo("Reached " + str(currj))

    except rospy.ROSInterruptException:
        print "killed"
