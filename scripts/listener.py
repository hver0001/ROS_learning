#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from induction.msg import TurtleInfo # Import the turtle information

# Callback called when the data is received
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "\nTurtle (%s) %s has a quality of %s.", data.id, data.name, data.quality)
    
# Listener waits for data
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

	# Subsribes to the topic
    rospy.Subscriber("ocean", TurtleInfo, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# This function is called when the script is launched
if __name__ == '__main__':
    listener()
