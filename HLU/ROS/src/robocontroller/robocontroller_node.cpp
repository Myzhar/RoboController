// This node handle the communication with RoboController board

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include "rbctrliface/rbctrliface.h"
#include "rbctrliface/robotctrl.h"
#include "rbctrliface/modbus_registers.h"
#include "rbctrliface/rc_common.h"

using namespace std;

ros::Time last_vel_cmd_time; // Time of the last velocity comand received
double vel_cmd_timeout_sec; // Timeout for motor stop if non velocity command is received

RbCtrlIface* rbCtrlIface= NULL; // RoboController Interface
RobotCtrl* rbCtrl = NULL; // Robot control functions


// >>>>> Functions
void vel_cmd_callback( const geometry_msgs::Twist& msg );
void test_connection();
// <<<<< Functions

void vel_cmd_callback( const geometry_msgs::Twist& msg )
{
    last_vel_cmd_time = ros::Time::now();

    double v = msg.linear.x;
    double omega = msg.angular.z;

    ROS_INFO_STREAM( "Received Twist. [v: " << v << "; omega: " << omega <<"]" );

    double D = 0.4; // TODO replace with real dimension of the robot!

    double Vr = v - 0.5 * omega * D;
    double Vl = v + 0.5 * omega * D;

    ROS_INFO_STREAM( "Vr: " << Vr << " ; Vl: " << Vl << " m/sec" );

    rbCtrl->setMotorSpeeds( Vl, Vr );

}


void test_connection()
{
    ROS_INFO_STREAM("Testing RoboController connection");

    if( !rbCtrlIface->testBoardConnection() )
    {
        ROS_ERROR_STREAM("Robocontroller is not replying. Trying reconnection...");
        rbCtrlIface->connectModbus( -1 );
    }
}

int main( int argc, char **argv) 
{
    ros::init( argc, argv, "robocontroller_node" );
    ros::NodeHandle nh;

    // Subscribing to cmd_vel message in geometry_msgs topic
    // Only 3 messages in queue since movement commands must be replace by newest
    ros::Subscriber cmd_vel_sub = nh.subscribe( "/robocontroller/cmd_vel", 3, &vel_cmd_callback );

    // Subscription to use Turtle keyboard node
    ros::Subscriber cmd_vel_turtle_sub = nh.subscribe( "/turtle1/cmd_vel", 3, &vel_cmd_callback );

    // >>>>> Interface to RoboController board

    // TODO load params using ROS parameters
    vel_cmd_timeout_sec = 1;
    int boardIdx = 1;
    string serialPort = "/dev/ttyUSB0";
    int serialbaudrate = 57600;
    char parity = 'N';
    int data_bit = 8;
    int stop_bit = 1;
    bool simulMode = false;

    rbCtrlIface = new RbCtrlIface( boardIdx, serialPort, serialbaudrate, parity, data_bit, stop_bit, simulMode );

    if(!rbCtrlIface->isConnected())
    {
        ROS_FATAL_STREAM( "RoboController not found on port " << serialPort );
        return -1;
    }
    // <<<<< Interface to RoboController board

    // Robot Control functions
    rbCtrl = new RobotCtrl( &nh, rbCtrlIface );

    // RoboController publishes telemetry at 30hz
    ros::Rate rate( 30 );

    last_vel_cmd_time = ros::Time::now();

    while( ros::ok() )
    {
        ros::spinOnce(); // Process pending callback

        // TODO request telemetry data to RoboController

        // >>>>> if not received a movement message for 1 sec then stop motors!
        double time_since_last_cmd = (ros::Time::now() - last_vel_cmd_time).toSec();
        if( !rbCtrl->isMotorStopped() &&
                time_since_last_cmd > vel_cmd_timeout_sec )
        {
            rbCtrl->stopMotors();

            ROS_INFO_STREAM( "No velocity command received since " << time_since_last_cmd << "seconds. Motor Stopped!" );

            test_connection(); // Let's send a test message to board to verify that it is connected
        }
        // <<<<< if not received a movement message for N msec then stop motors!
        rate.sleep();
    }

    if(rbCtrlIface)
        delete rbCtrlIface;

    if(rbCtrl)
        delete rbCtrl;

    return 0;
}
