#ifndef ROBOCONTROLLERSDK_GLOBAL_H
#define ROBOCONTROLLERSDK_GLOBAL_H

#include <QtCore/qglobal.h>
#include <QString>

#if defined(ROBOCONTROLLERSDK_LIBRARY)
#  define ROBOCONTROLLERSDKSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ROBOCONTROLLERSDKSHARED_EXPORT Q_DECL_IMPORT
#endif

#include "exception.h"

// >>>>> Radians/Degree conversion constants
#ifdef RAD2DEG
#undef RAD2DEG
#endif

#ifdef DEG2RAD
#undef DEG2RAD
#endif

#define RAD2DEG 57.29577951308233
#define DEG2RAD 0.0174532925199433
// <<<<< Radians/Degree conversion constants

// >>>>> Multicast Server IPs
#define MULTICAST_DATA_SERVER_IP "239.255.43.21"
#define MULTICAST_WEBCAM_SERVER_IP "239.255.43.31"
// <<<<< Multicast Server IPs

namespace roboctrl
{

/**
 * @enum MotorCtrlMode
 * @brief Motor control modes.
 */
typedef enum
{
    mcDirectPWM = 0,    /**< PID is not active, speeds are in PWM mode: 0 -> Motor Stopped, +100/-100 -> Motor at max rate */
    mcPID = 1           /**< PID is active, speeds are in mm/sec */
} MotorCtrlMode;

/**
 * @enum Option
 * @brief ON/OFF for toggle options
 */
typedef enum
{
    Off = 0,    /**< Option not active */
    On = 1      /**< Option active */
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
    Motor = 0,  /**< Encoder suited on the shaft of the motor */
    Wheel = 1   /**< Encoder suited on the shaft of the wheel */
} EncoderPos;

/**
 * @enum AnalogCalibValue
 * @brief Indicates the type of value for Analogic Ports calibration
 */
typedef enum
{
    CalLow = 0, /**< Lower Value for analogic calibration */
    CalHigh = 1 /**< Higher Value for analogic calibration */
} AnalogCalibValue;

/**
  * @struct _BoardStatus
  * @brief Used to mantain the state of the
  *        configuration of the board
  */
typedef struct _BoardStatus
{
    bool pidEnable;         /**< Indicates if the motor PID controls are enabled */
    bool wdEnable;          /**< Indicates if the Command WatchDod is enabled. If it is true and the board does not receive command for N msec (@ref getWatchDogTime and @ref setWatchDogTime)the motors stop */
    bool saveToEeprom;      /**< Indicates if the Board parameters are saved to Eeprom when changed */
    bool accelRampEnable;   /**< Indicates if acceleration is limited by speed ramps */
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
    PinLevel MotorEnableLevel;      /**< Enable Level of the Robot Driver (Low/High)*/
    EncoderPos EncoderPosition;     /**< Encoder on the shaft of the motor or of the wheel */
    // <<<<< Wheels, Motors and Reduction

    // >>>>> Battery
    quint16 MaxChargedBatteryLevel; /**< Value of the power battery fully charged (Volts * 1000) */
    quint16 MinChargedBatteryLevel; /**< Value of the power battery to be considered discharged (Volts * 1000) */
    // <<<<< Battery
} RobotConfiguration;

/**
  * @struct _RobotTelemetry
  * @brief Used to keep track of the telemetry of the robot
  */
typedef struct _RobotTelemetry
{
    QString CtrlClientIP;   /**< IP of the client that is controlling the robot */
    qint16 PwmLeft;         /**< Last value of the PWM for left wheel */
    qint16 PwmRight;        /**< Last value of the PWM for right wheel */
    qreal RpmLeft;          /**< Last RPM for left wheel */
    qreal RpmRight;         /**< Last RPM for right wheel */
    qreal LinSpeedLeft;     /**< Last Linear Speed for left Wheel */
    qreal LinSpeedRight;    /**< Last Linear Speed for right Wheel */
    qreal Battery;          /**< Last battery voltage */
} RobotTelemetry;

}

#endif // ROBOCONTROLLERSDK_GLOBAL_H
