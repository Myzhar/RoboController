#ifndef RC_COMMON_H
#define RC_COMMON_H

#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;

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
 * @enum MotorCtrlMode
 * @brief Motor control modes.
 */
typedef enum
{
    motLeft = 0,    /**< Used to indicate the left motor */
    motRight = 1    /**< Used to indicate the right motor */
} MotorPos;

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
    uint16_t Weight; /**< Robot Weight (g) */
    uint16_t Width;  /**< Robot Width (mm) */
    uint16_t Height; /**< Robot Height (mm) */
    uint16_t Lenght; /**< Robot Lenght (mm) */
    // <<<<< Dimensions

    // >>>>> Wheels, Motors and Reduction
    uint16_t WheelBase;              /**< Distance between the center of the Wheels (mm) */
    uint16_t WheelRadiusLeft;        /**< Radius of the left wheel (0.01mm) */
    uint16_t WheelRadiusRight;       /**< Radius of the right wheel (0.01mm) */
    uint16_t EncoderCprLeft;         /**< Count per Round of the left encoder */
    uint16_t EncoderCprRight;        /**< Count per Round of the right encoder */
    uint16_t MaxRpmMotorLeft;        /**< Max RPM of the left motor */
    uint16_t MaxRpmMotorRight;       /**< Max RPM of the right motor */
    uint16_t MaxAmpereMotorLeft;     /**< Max current assorbed by left motor (mA) */
    uint16_t MaxAmpereMotorRight;    /**< Max current assorbed by right motor (mA) */
    uint16_t MaxTorqueMotorLeft;     /**< Max torque of the left motor (Ncm) */
    uint16_t MaxTorqueMotorRight;    /**< Max torque of the right motor (Ncm) */
    uint16_t RatioShaftLeft;         /**< Reduction Ratio from the shaft of the left motor to the shaft of the left wheel*/
    uint16_t RatioShaftRight;        /**< Reduction Ratio from the shaft of the right motor to the shaft of the right wheel*/
    uint16_t RatioMotorLeft;         /**< Reduction Ratio on the left Motor Shaft (Put 1 if you set Max RPM considering it just reduced) */
    uint16_t RatioMotorRight;        /**< Reduction Ratio on the right Motor Shaft (Put 1 if you set Max RPM considering it just reduced) */
    PinLevel MotorEnableLevel;      /**< Enable Level of the Robot Driver (Low/High)*/
    EncoderPos EncoderPosition;     /**< Encoder on the shaft of the motor or of the wheel */
    // <<<<< Wheels, Motors and Reduction

    // >>>>> Battery
    uint16_t MaxChargedBatteryLevel; /**< Value of the power battery fully charged (Volts * 1000) */
    uint16_t MinChargedBatteryLevel; /**< Value of the power battery to be considered discharged (Volts * 1000) */
    // <<<<< Battery
} RobotConfiguration;

/**
  * @struct _RobotTelemetry
  * @brief Used to keep track of the telemetry of the robot
  */
typedef struct _RobotTelemetry
{
    string CtrlClientIP;   /**< IP of the client that is controlling the robot */
    int16_t PwmLeft;         /**< Last value of the PWM for left wheel */
    int16_t PwmRight;        /**< Last value of the PWM for right wheel */
    double RpmLeft;          /**< Last RPM for left wheel */
    double RpmRight;         /**< Last RPM for right wheel */
    double LinSpeedLeft;     /**< Last Linear Speed for left Wheel */
    double LinSpeedRight;    /**< Last Linear Speed for right Wheel */
    double Battery;          /**< Last battery voltage */
} RobotTelemetry;

#endif // RC_COMMON_H
