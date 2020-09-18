#!/usr/bin/env python
import rospy
import random
from induction.msg import TurtleInfo # Import the turtle information

# Set a list of names
turtleNames = ["Apple", "Ben", "Cecelia", "Duncan", "Eggbert", "Fred", "Gina", "Holly", "Ishra", "Jack",
"Kyle", "Lilly", "Mike", "Nath", "Ophelia", "Penny", "Quarter", "Ross", "Squirt", "Tammy", "Ursula",
"Vig", "Wally", "X", "Yousef", "Zed"]

# Publishes a turtle
def turtle():
	
	pub = rospy.Publisher('ocean', TurtleInfo, queue_size=10) # Topic is called ocean
	rospy.init_node('turtle', anonymous=True) # Node is called turtle
	rate = rospy.Rate(0.2) # 1 every 5 seconds
	
	turtleID = 0
	
	# Loop until roscore stops
	while not rospy.is_shutdown():
		# Create the message type
		msg = TurtleInfo()
		msg.id = turtleID
		msg.quality = random.randint(1, 10)
		msg.name = turtleNames[random.randint(0, len(turtleNames) - 1)]
	
		# Only publish if the turtles are good enough
		if (msg.quality >= 7):
			# Publish the message
			rospy.loginfo(msg)
			pub.publish(msg)
		
		# Otherwise broadcast error
		else:
			rospy.loginfo("New turtle does not have a quality level high enough to show.")
			
		rate.sleep()
		
		turtleID += 1

# This is the main function that is called when the script is run
if __name__ == '__main__':
	try:
		turtle()
	except rospy.ROSInterruptException:
		pass
