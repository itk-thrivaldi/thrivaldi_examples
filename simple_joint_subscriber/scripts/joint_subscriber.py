#!/usr/bin/env python
import rospy # For all things ros with python

# JointState is defined in sensor_msgs.msg
# If you know a message but not where it is
# call rosmsg info MSGNAME from the terminal
from sensor_msgs.msg import JointState

# This tutorial takes heavily from
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)

# In this example we make a simple subscriber that listens for JointState
# messages, and prints them. Uses a functional approach.

def message_callback(msg):
    """This function is called on the message every time a message arrives."""
    rospy.loginfo("Joint position received:"+str(msg.position))


def joint_listener():
    """Blocking function that sets up node, subscription and waits for
    messages."""
    # Start ros node
    rospy.init_node("joint_listener", anonymous=True)
    # Tell the central command we want to hear about /joint_states
    rospy.Subscriber("/joint_states",  # Topic we subscribe to
                     JointState,  # message type that topic has
                     message_callback)  # function to call when message arrives
    rospy.spin()

# If this script is run alone, not just imported:
if __name__ == "__main__":
    joint_listener()

# Ensure that the python script is executable by running:
# chmod +x joint_subscriber.py

# Call this script by running:
# rosrun joint_subscriber joint_subscriber.py
