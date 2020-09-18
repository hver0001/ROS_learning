// Include the message files and packages 
#include "ros/ros.h"
#include <induction/TurtleInfo.h>

// Random number packages
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

// Standard namespace for strings
using namespace std;

#include <sstream> // Used for displaying output

// Create a list of turtle names
const char *turtleNames[26] = {"Apple", "Ben", "Cecelia", "Duncan", "Eggbert", "Fred", "Gina", "Holly", "Ishra", "Jack", "Kyle", "Lilly", "Mike", "Nath", "Ophelia", "Penny", "Quarter", "Ross", "Squirt", "Tammy", "Ursula", "Vig", "Wally", "X", "Yousef", "Zed"};

// The main function called when this script is executed
int main(int argc, char **argv) {

	/*
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* This must be called before any other part of the system can be called.
	*/
	ros::init(argc, argv, "turtle"); // Takes in the node name

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages. If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	ros::Publisher turtle_pub = n.advertise<induction::TurtleInfo>("ocean", 1000);

	// Create a loop rate
	ros::Rate loop_rate(10);
	
	// Randomises the random
	srand(time(NULL));

	// A count for the ID
	int ID = 0;
	
	// Loops until roscore stops
	while (ros::ok()) {
		
		// Creates a information object
		induction::TurtleInfo msg;
		
		// Sets the variables
		msg.name = turtleNames[rand() % 26];
		msg.id = ID;
		msg.quality = rand() % 10 + 1;
		
		

		// Publishes the information as a string
		ROS_INFO("Turtle #%d, %s has been sent with quality of %d.", msg.id, msg.name.c_str(), msg.quality);

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		turtle_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++ID;
	}

	return 0;
}

