// This node handle the communication with RoboController board

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include "robocontroller/robocontroller.h"

using namespace std;

ros::Time last_vel_cmd; // Time of the last velocity comand received
double timeout_sec = 1; // Timeout for motor stop if non velocity command is received

void vel_cmd_callback( const geometry_msgs::Twist& msg )
{
	last_vel_cmd = ros::Time::now();
}

void stop_motors()
{
	// TODO send velocity 0 msg to RoboController
}

int main( int argc, char **argv) 
{
	ros::init( argc, argv, "robocontroller_node" );
	ros::NodeHandle nh;
	
	// Subscribing to cmd_vel message in geometry_msgs topic
	ros::Subscriber cmd_vel_sub = nh.subscribe( "cmd_vel", 3, &vel_cmd_callback ); // TODO verify the correct topic

	ros::Rate rate( 30 ); // RoboController publish telemetry at 30hz
	
	while( ros::ok() )
	{
		ros::spinOnce(); // Process pending callback

		// TODO request telemetry data to RoboController		
	
		// >>>>> if not received a movement message for N msec then stop motors!
		double time_since_last_cmd = (ros::Time::now() - last_vel_cmd).toSec();
		if( time_since_last_cmd > timeout_sec )
		{
			stop_motors();
			ROS_WARN("No velocity command received. Motor Stopped");
		}
		// <<<<< if not received a movement message for N msec then stop motors!
		rate.sleep();
	}
}
