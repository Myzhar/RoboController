#include "robotctrl.h"
#include "modbus_registers.h"

RobotCtrl::RobotCtrl(ros::NodeHandle* nh, RbCtrlIface *rbCtrl)
{
    mNodeH = nh;
    mMotStopped = true;

    mRbCtrl = rbCtrl;
}

bool RobotCtrl::setMotorSpeeds( double speedL, double speedR )
{
    // TODO Verify that RoboController is in PID mode

    // >>>>> 16 bit saturation
    if( speedL > 32.767)
        speedL = 32.767;

    if( speedL < -32.768 )
        speedL = -32.768;

    if( speedR > 32.767)
        speedR = 32.767;

    if( speedR < -32.768 )
        speedR = -32.768;
    // <<<<< 16 bit saturation

    // >>>>> New SetPoint to RoboController
    uint16_t address = WORD_PWM_CH1;

    vector<uint16_t> data;
    data.resize(2);

    uint16_t sp; // Speed is integer 2-complement!
    if(speedL >= 0)
        sp = (uint16_t)(speedL*1000.0);
    else
        sp = (uint16_t)(speedL*1000.0+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[0] = sp;

    if(speedR >= 0)
        sp = (uint16_t)(speedR*1000.0);
    else
        sp = (uint16_t)(speedR*1000.0+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[1] = sp;

    bool commOk = mRbCtrl->writeMultiReg( address, 2, data );
    // <<<<< New SetPoint to RoboController

    if( commOk )
    {
        if( speedL!=0.0 && speedL!=0 )
            mMotStopped = false;
        else
            mMotStopped = true;
    }

    return commOk;
}

bool RobotCtrl::stopMotors()
{
    int count = 0;

    ros::Rate rate( 30 );
    while( 1 ) // Try to stop the motors 5 times for security!
    {
        if( setMotorSpeeds( 0.0, 0.0) )
        {
            mMotStopped = true;
            return true;
        }
        else
            count++;

        if(count==5)
            return false;

        rate.sleep();
    }
}
