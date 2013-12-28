#ifndef ROBOCONTROLLERSDK_GLOBAL_H
#define ROBOCONTROLLERSDK_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ROBOCONTROLLERSDK_LIBRARY)
#  define ROBOCONTROLLERSDKSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ROBOCONTROLLERSDKSHARED_EXPORT Q_DECL_IMPORT
#endif

#include "exception.h"

/**
 * @enum MotorCtrlMode
 * @brief Motor control modes.
 */
typedef enum
{
    mcDirectPWM = 0, /**< PID is not active, speeds are in PWM mode: 0 -> Motor Stopped, +100/-100 -> Motor at max rate */
    mcPID = 1 /**< PID is active, speeds are in mm/sec */
} MotorCtrlMode;

/**
 * @enum Option
 * @brief ON/OFF for toggle options
 */
typedef enum
{
    Off = 0, /**< Option not active */
    On = 1 /**< Option active */
} Option;

/**
 * @enum PinLevel
 * @brief High/Low for pin levels
 */
typedef enum
{
    Low = 0, /**< Low Active */
    High = 1 /**< High Active */
} PinLevel;

/**
 * @enum EncoderPos
 * @brief Indicates the position of the Encoder
 */
typedef enum
{
    Motor = 0, /**< Encoder suited on the shaft of the motor */
    Wheel = 1 /**< Encoder suited on the shaft of the wheel */
} EncoderPos;

/**
 * @enum CommMode
 * @brief Communication modes.
 */
typedef enum
{
    cmNormal = 0, /**< Normal communication over unsecure UDP protocol */
    cmConfiguration = 1 /**< Secure communication mode over TCP used to send configuration parameters */
} CommMode;

/**
  * @struct _BoardStatus
  * @brief Used to mantain the state of the
  *        configuration of the board
  */
typedef struct _BoardStatus
{
    bool pidEnable; /**< Indicates if the motor PID controls are enabled */
    bool wdEnable; /**< Indicates if the Command WatchDod is enabled. If it is true and the board does not receive command for N msec (@ref getWatchDogTime and @ref setWatchDogTime)the motors stop */
    bool saveToEeprom; /**< Indicates if the Board parameters are saved to Eeprom when changed */
    bool accelRampEnable; /**< Indicates if acceleration is limited by speed ramps */
} BoardStatus;

/**
  * @struct _RobotConfiguration
  * @brief Used to mantain the state of the configuration of the robot
  */
typedef struct _RobotConfiguration
{
    // >>>>> Dimensions
    quint16 Weight; /**< Robot Weight (g) */
    quint16 Width;  /**< Robot Width (mm) */
    quint16 Height; /**< Robot Height (mm) */
    quint16 Lenght; /**< Robot Lenght (mm) */
    // <<<<< Dimensions

    // >>>>> Wheels, Motors and Reduction
    quint16 WheelBase;              /**< Distance between the center of the Wheels (mm) */
    quint16 WheelRadiusLeft;        /**< Radius of the left wheel (0.01mm) */
    quint16 WheelRadiusRight;       /**< Radius of the right wheel (0.01mm) */
    quint16 EncoderCprLeft;         /**< Count per Round of the left encoder */
    quint16 EncoderCprRight;        /**< Count per Round of the right encoder */
    quint16 MaxRpmMotorLeft;        /**< Max RPM of the left motor */
    quint16 MaxRpmMotorRight;       /**< Max RPM of the right motor */
    quint16 MaxAmpereMotorLeft;     /**< Max current assorbed by left motor (mA) */
    quint16 MaxAmpereMotorRight;    /**< Max current assorbed by right motor (mA) */
    quint16 MaxTorqueMotorLeft;     /**< Max torque of the left motor (Ncm) */
    quint16 MaxTorqueMotorRight;    /**< Max torque of the right motor (Ncm) */
    quint16 RatioShaftLeft;         /**< Reduction Ratio from the shaft of the left motor to the shaft of the left wheel*/
    quint16 RatioShaftRight;        /**< Reduction Ratio from the shaft of the right motor to the shaft of the right wheel*/
    quint16 RatioMotorLeft;         /**< Reduction Ratio on the left Motor Shaft (Put 1 if you set Max RPM considering it just reduced) */
    quint16 RatioMotorRight;        /**< Reduction Ratio on the right Motor Shaft (Put 1 if you set Max RPM considering it just reduced) */
    PinLevel MotorEnableLevel;  /**< Enable Level of the Robot Driver (Low/High)*/
    EncoderPos EncoderPosition; /**< Encoder on the shaft of the motor or of the wheel */
    // <<<<< Wheels, Motors and Reduction
} RobotConfiguration;

#endif // ROBOCONTROLLERSDK_GLOBAL_H
