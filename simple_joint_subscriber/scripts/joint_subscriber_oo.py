#!/usr/bin/env python
import rospy # For all things ros with python

# JointState is defined in sensor_msgs.msg
# If you know a message but not where it is
# call rosmsg info MSGNAME from the terminal
from sensor_msgs.msg import JointState

# This tutorial takes heavily from
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)

# In this example we make a simple subscriber that listens for JointState
# messages, and prints them. Uses an object-oriented approach.


class JointSubscriber(object):
    def __init__(self):
        rospy.init_node("joint_listener", anonymous=True)
        rospy.Subscriber("/joint_states",  # Topic of interest
                         JointState,  # message type
                         self.message_callback)  # callback function
        rospy.loginfo("JoinSubscriberObject initialized")

    def message_callback(self, msg):
        """Function called on the message when a new message arrives."""
        rospy.loginfo("Joint position received:"+str(msg.position))

    def run(self):
        """Blocking function that just calls rospy.spin"""
        rospy.spin()


#  If this script is run alone, not just imported, this executes:
if __name__ == "__main__":
    # Initialize subscriber object
    js = JointSubscriber()

    # To ensure a clean shutdown, handle the ros interrupt exception
    try:
        js.run()
    except rospy.ROSInterruptException:
        pass

# Ensure that the python script is executable by running:
# chmod +x joint_subscriber_oo.py

# Call this script by running:
# rosrun joint_subscriber joint_subscriber_oo.py
