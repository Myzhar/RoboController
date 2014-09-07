#ifndef ROBOTCTRL_H
#define ROBOTCTRL_H

#include <ros/ros.h>
#include "rbctrliface.h"

class RobotCtrl
{
public:
    RobotCtrl(ros::NodeHandle* nh,RbCtrlIface* rbCtrl);

    bool setMotorSpeeds(double speedL, double speedR );
    bool stopMotors();

    inline bool isMotorStopped(){return mMotStopped;}

private:
    ros::NodeHandle* mNodeH;
    RbCtrlIface* mRbCtrl;

    bool mMotStopped;
};

#endif // ROBOTCTRL_H
