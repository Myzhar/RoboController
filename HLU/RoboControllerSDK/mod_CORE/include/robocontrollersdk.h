#ifndef ROBOCONTROLLERSDK_H
#define ROBOCONTROLLERSDK_H

#include <RoboControllerSDK_global.h>

#include <QThread>
#include <QTimer>
#include <QMutex>
#include <QString>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QTcpSocket>

#define ROBOT_CONFIG_INI_FILE "./robotConfig.ini"

#define SERVER_REPLY_TIMEOUT_MSEC 5000
#define UDP_PING_TIME_MSEC 1000
#define CONTROL_UDP_TIMEOUT 15000 // Control timeout after 15 sec

namespace roboctrl
{

class ROBOCONTROLLERSDKSHARED_EXPORT RoboControllerSDK : public QThread
{
    Q_OBJECT

public:
    explicit RoboControllerSDK(QString serverAddr=QString("127.0.0.1"),
                               quint16 udpStatusPortSend=14550,
                               quint16 udpStatusPortListen=14555,
                               quint16 multicastUdpPort=14565,
                               quint16 udpControlPort=14560,
                               quint16 tcpPort=14500);

    virtual ~RoboControllerSDK();

    /** @brief Searches for the server on the local network
     *
     *
     * @returns IP of the server or an empty QString
     *
     */
    static QString findServer(quint16 udpSendPort=14550, quint64 udpListenPort=14555 ); // Tested

    /** @brief Sets the speed of the motor in m/sec
     *
     * @param motorIdx Index of the motor (@ref MotorPos)
     * @param speed The speed in m/sec - Speed range: [-32.768/+32.767] m/sec
     *
     * @note This function works only when @ref mMotorCtrlMode is
     *       equal to @ref mcPid, else it does nothing
     */
    void setMotorSpeed( MotorPos motorIdx, double speed );

    /** @brief Sets the speed of the motors in m/sec
     *
     * @param speed0 The speed of motor 0 in m/sec - Speed range: [-32.768/+32.767] m/sec
     * @param speed1 The speed of motor 1 in m/sec - Speed range: [-32.768/+32.767] m/sec
     *
     * @note This function works only when @ref mMotorCtrlMode is
     *       equal to @ref mcPid, else it does nothing
     */
    void setMotorSpeeds( double speed0, double speed1 );

    /** @brief Sets the PWM of the motor
     *
     * @param motorIdx Index of the motor (@ref MotorPos)
     * @param pwm PWM range: [-2048/2047]
     *
     * @note This function works only when @ref mMotorCtrlMode is
     *       equal to @ref mcDirectPWM, else it does nothing
     */
    void setMotorPWM( MotorPos motorIdx, int pwm ); // Tested

    /** @brief Sets the PWMs of both motors
     *
     * @param pwmMotor0 PWM range: [-2048/2047]
     * @param pwmMotor1 PWM range: [-2048/2047]
     *
     * @note This function works only when @ref mMotorCtrlMode is
     *       equal to @ref mcDirectPWM, else it does nothing
     */
     void setMotorPWMs( quint16 pwmMotor0, quint16 pwmMotor1 );

    /** @brief Sets motor PID Controllers parameters.
     *
     * @param motorIdx Index of the motor (@ref MotorPos)
     * @param Kp Proportional Action gain
     * @param Ki Integral Action gain
     * @param Kd Derivative Action gain
     */
    void setMotorPidGains( MotorPos motorIdx, quint16 Kp, quint16 Ki, quint16 Kd );

    /** @brief Send a request for motor PID gains.
     *         The reply is received with @ref newMotorPIDGains
     *         signal
     *
     * @param motorIdx Index of the motor (@ref MotorPos)
     */
    void getMotorPidGains( MotorPos motorIdx );

    /** @brief Gets current board status (@ref BoardStatus)
     *         The reply is received with @ref newBoardStatus
     */
    void getBoardStatus(); // Tested

    /** @brief Sets a new @ref BoardStatus
     *
     * @param status The new status to be set
     */
    bool setBoardStatus( BoardStatus &status ); // Tested

    /** @brief Load the Robot Configuration from ini file
     *         The Robot configuration is not stored on
     *         Robot EEPROM unless saving is active.
     *
     * @param iniFile The path of the ini file
     *
     * @return true if everything is ok
     */
    bool getRobotConfigurationFromIni( QString iniFile = ROBOT_CONFIG_INI_FILE );

    /** @brief Load the Robot Configuration from Robot EEPROM
     *         The Robot configuration is retrieved from Robot
     */
    void getRobotConfigurationFromEeprom( );

    /** @brief Save the Robot Configuration to Robot EEPROM
     *         The Robot configuration is saved on the Robot
     *
     * @param iniFile The path of the ini file
     */
    void saveRobotConfigurationToIni( QString iniFile = ROBOT_CONFIG_INI_FILE );

    /** @brief Save the Robot Configuration to Robot EEPROM
     *         The Robot configuration is saved on the Robot
     */
    void saveRobotConfigurationToEeprom( );

    /** @brief Set a new Robot Configuration without saving to EEPROM
     */
    void setRobotConfiguration( RobotConfiguration& roboConfig );

    /** @brief Try to take control of the robot for driving
     */
    void getRobotControl();

    /** @brief Release the control of the robot for other clients
     */
    void releaseRobotControl();

    /** @brief Calibrates the Battery charge value
     *
     * @param valueType the type of value to be set (see @ref AnalogCalibValue)
     * @param curChargeVal Current value of charge read with a tester connected to the battery
     */
    void setBatteryCalibrationParams( AnalogCalibValue valueType, double curChargeVal);

    /** @brief Disables the Communication Watchdog
     *         WatchDog if active stops motors if communication is lost
     */
    void disableWatchdog(); // Tested

    /** @brief Enables the Communication Watchdog
     *         WatchDog if active stops motors if communication is lost
     *
     * @param timeOut_msec if the board does not receive commands by an @ref RoboControllerSDK
     *                     object stops the motor for security
     */
    void enableWatchdog( quint16 timeOut_msec);

    /** @brief Send a request for watchdog time.
     *         The reply is received with /ref newWatchdogValue
     *         signal
     */
    void getWatchdogTime( ); // Tested

    /** @brief Retrieve the last telemetry received by @ref QRobotTelemetryServer
     *  @return The timestamp of the last telemetry received
     */
    qint64 getLastTelemetry( RobotTelemetry& telemetry );

protected:
    /// Thread function
    virtual void run();

private:
    /// Operations performed during communication
    //void commThread();

    /// TCP Connection
    void connectToTcpServer();
    /// TCP Disconnection
    void disconnectTcpServer();

    /// UDP Connection
    void connectToUdpServers();
    /// UDP Disconnection
    void disconnectUdpServers();

    /// Processes a reply message
    void processReplyMsg(QDataStream *inStream );

    /// Updates Robot Configuration from data stream
    void updateRobotConfigurationFromDataStream( QDataStream* inStream );

    /// Sends a command to TCP server
    bool sendBlockTCP( quint16 msgCode, QVector<quint16> &data );

    /// Sends a command to UDP server
    void sendBlockUDP( QUdpSocket *socket, QHostAddress addr, quint16 port, quint16 msgCode, QVector<quint16> &data, bool waitReply=false );

protected slots:
    /// Processes data from TCP Socket
    void onTcpReadyRead();
    /// Handles errors on TCP socket
    void onTcpError(QAbstractSocket::SocketError err);
    /// Handles the "Host Found" status
    void onTcpHostFound();

    /// Processes data from UDP Status Socket
    void onUdpStatusReadyRead();
    /// Processes data from UDP Telemetry Socket
    void onUdpTelemetryReadyRead();

    /// Handles errors on UDP Status Socket
    void onUdpStatusError( QAbstractSocket::SocketError err );
    /// Handles errors on UDP Control Socket
    void onUdpControlError( QAbstractSocket::SocketError err );

    /// Send Test ping to UDP Servers
    void onUdpTestTimerTimeout();

    /// Ping Timer handler
    void onPingTimerTimeout();

signals:
    /// Signal emitted when TCP Socket is connected
    void tcpConnected();
    /// Signal emitted when TCP Socket is disconnected
    void tcpDisconnected();
    /// Signal emitted when UDP Socket is connected
    void udpConnected();
    /// Signal emitted when UDP Socket is disconnected
    void udpDisconnected();

    /// Signal emitted when new PID gains are received
    void newMotorPIDGains( MotorPos motorIdx, quint16 Kp, quint16 Ki, quint16 Kd );
    /// Signal emitted when a new Robot Configuration is received from robot
    void newRobotConfiguration( RobotConfiguration& robConf );
    /// Signal emitted when a new Board Status is ready
    void newBoardStatus(BoardStatus& status);
    /// Signal emitted when a new Watchdog value is available
    void newWatchdogTime( quint64 wd_value );
    /// Signal emitted when a new telemetry is available
    void newTelemetryAvailable();

    /// Signal emitted when client takes Robot Control successfully
    void robotControlTaken();
    /// Signal emitted when client try to take Robot Control, but fails
    void robotControlNotTaken();
    /// Signal emitted when client releases the Robot Control
    void robotControlReleased();

private:
    qint64 mLastServerReqTime; /**< Information about last connection time */

    int mUdpStatusPortSend;      /**< Socket UDP Status Port to send datagrams*/
    int mUdpStatusPortListen;    /**< Socket UDP Status Port to bind to receive datagrams*/
    int mUdpControlPortSend;     /**< Socket UDP Control Port */
    int mUdpTelemetryMulticastPort; /**< Multicast UDP Port for telemetry */
    int mTcpPort;          /**< Socket TCP Port */

    QString mServerAddr;   /**< Server address */

    QMutex mConnMutex;  /**< Mutex to share connection between functions safely */

    QTcpSocket* mTcpSocket;         /**< TCP Socket for secure communications */
    QUdpSocket* mUdpStatusSocket;   /**< UDP Socket for status communications */
    QUdpSocket* mUdpControlSocket;   /**< UDP Socket for control communications */
    QUdpSocket* mUdpMulticastTelemetrySocket; /**< UDP Socket to receive telemetry */

    quint16 mNextTcpBlockSize;      /**< Used to recover incomplete TCP block */

    bool mTcpConnected;     /**< Indicates if TCP Socket is connected */
    bool mUdpConnected;     /**< Indicates if UDP Socket is connected */

    MotorCtrlMode mMotorCtrlMode; /**< Current motor control mode */
    BoardStatus mBoardStatus; /**< Current board status */

    bool mBoardStatusValid; /**< Indicates if mBoardStatus is valid */

    RobotConfiguration mRobotConfig; /**< Configuration of the robot */

    int mBoardIdx;          /**< Id of the connected board */

    QString mLastTcpErrorMsg; /**< Last TCP Socket Error */
    QString mLastUdpErrorMsg; /**< Last UDP Socket Error */

    bool mStopped; /**< Thread stopped */
    quint64 mWatchDogTimeMsec; /**< Board WatchDog Time in millisecond*/
    QTimer mPingTimer; /** Id of the ping timer. Ping is called with a time smaller of 10% than watchdog time to mantain the board active */

    bool            mWatchDogEnable;
    bool            mNewStatusBit1Received;
    quint16         mStatusBits1;
    bool            mNewStatusBit2Received;
    quint16         mStatusBits2;

    quint16         mMsgCounter; /**< Counts the number of message sent */

    bool mReceivedStatus2; /**< Indicates if a new WORD_STATUSBIT2 has been received.
                                If both @ref mReceivedStatus2 and @ref mReceivedRobConfig
                                are true a @ref newRobotConfiguration signal can be emitted.*/
    bool mReceivedRobConfig; /**< Indicates if a new RobotConfig has been received from EEPROM.
                                  If both @ref mReceivedStatus2 and @ref mReceivedRobConfig
                                  are true a @ref newRobotConfiguration signal can be emitted. */

    QTimer mUdpPingTimer; /**< Timer of Udp Servers testing */

    RobotTelemetry mTelemetry; /**< Telemetry of the robot, updated every @ref TELEMETRY_UPDATE_MSEC msec */
    qint64 mLastTelemetryTimestamp; /**< The timestamp of the last received telemetry */
};

}

#endif // ROBOCONTROLLERSDK_H
