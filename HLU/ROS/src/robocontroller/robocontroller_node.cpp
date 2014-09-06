// This node handle the communication with RoboController board

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include "rbctrliface/rbctrliface.h"
#include "rbctrliface/modbus_registers.h"

using namespace std;

ros::Time last_vel_cmd_time; // Time of the last velocity comand received
double vel_cmd_timeout_sec = 1; // Timeout for motor stop if non velocity command is received

RbCtrlIface* rbCtrl = NULL; // RoboController Interface

bool motor_stopped = true;

// >>>>> Functions
void vel_cmd_callback( const geometry_msgs::Twist& msg );
bool stop_motors();
void test_connection();
bool setMotorSpeeds( double speed0, double speed1 );
// <<<<< Functions

void vel_cmd_callback( const geometry_msgs::Twist& msg )
{
    last_vel_cmd_time = ros::Time::now();

    ROS_INFO_STREAM( "Received Twist. [Vx: " << msg.linear.x << "; Rot_z: " << msg.angular.z );

    motor_stopped = false;
}

bool stop_motors()
{
    int count = 0;

    ros::Rate rate( 30 );
    while( 1 ) // Try to stop the motors 5 times for security!
    {
        if( setMotorSpeeds( 0.0, 0.0) )
        {
            motor_stopped = true;
            return true;
        }
        else
            count++;

        if(count==5)
            return false;

        rate.sleep();
    }
}

void test_connection()
{
    ROS_INFO_STREAM("Testing RoboController connection");

    if( !rbCtrl->testBoardConnection() )
    {
        ROS_ERROR_STREAM("Robocontroller is not replying. Trying reconnection...");
        rbCtrl->connectModbus( -1 );
    }
}

bool setMotorSpeeds( double speed0, double speed1 )
{
    // TODO Verify that RoboController is in PID mode

    // >>>>> 16 bit saturation
    if( speed0 > 32.767)
        speed0 = 32.767;

    if( speed0 < -32.768 )
        speed0 = -32.768;

    if( speed1 > 32.767)
        speed1 = 32.767;

    if( speed1 < -32.768 )
        speed1 = -32.768;
    // <<<<< 16 bit saturation

    // >>>>> New SetPoint to RoboController
    uint16_t address = WORD_PWM_CH1;

    vector<uint16_t> data;
    data.resize(2);

    uint16_t sp; // Speed is integer 2-complement!
    if(speed0 >= 0)
        sp = (uint16_t)(speed0*1000.0);
    else
        sp = (uint16_t)(speed0*1000.0+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[0] = sp;

    if(speed1 >= 0)
        sp = (uint16_t)(speed1*1000.0);
    else
        sp = (uint16_t)(speed1*1000.0+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[1] = sp;

    bool commOk = rbCtrl->writeMultiReg( address, 2, data );
    // <<<<< New SetPoint to RoboController

    if( commOk )
    {
        if( speed0!=0.0 && speed0!=0 )
            motor_stopped = false;
        else
            motor_stopped = true;
    }

    return commOk;
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
    int boardIdx = 1;
    string serialPort = "/dev/ttyUSB0";
    int serialbaudrate = 57600;
    char parity = 'N';
    int data_bit = 8;
    int stop_bit = 1;
    bool simulMode = false;

    rbCtrl = new RbCtrlIface( boardIdx, serialPort, serialbaudrate, parity, data_bit, stop_bit, simulMode );

    if(!rbCtrl->isConnected())
    {
        ROS_FATAL_STREAM( "RoboController not found on port " << serialPort );
        return -1;
    }
    // <<<<< Interface to RoboController board

    // RoboController publishes telemetry at 30hz
    ros::Rate rate( 30 );

    last_vel_cmd_time = ros::Time::now();

    while( ros::ok() )
    {
        ros::spinOnce(); // Process pending callback

        // TODO request telemetry data to RoboController

        // >>>>> if not received a movement message for 1 sec then stop motors!
        double time_since_last_cmd = (ros::Time::now() - last_vel_cmd_time).toSec();
        if( !motor_stopped && time_since_last_cmd > vel_cmd_timeout_sec )
        {
            stop_motors();

            ROS_INFO_STREAM( "No velocity command received since " << time_since_last_cmd << "seconds. Motor Stopped!" );

            test_connection(); // Let's send a test message to board to verify that it is connected
        }
        // <<<<< if not received a movement message for N msec then stop motors!
        rate.sleep();
    }

    if(rbCtrl)
        delete rbCtrl;

    return 0;
}
