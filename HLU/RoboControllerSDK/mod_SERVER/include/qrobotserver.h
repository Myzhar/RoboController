#ifndef QROBOCONTROLLERSERVER_H
#define QROBOCONTROLLERSERVER_H

#include "RoboControllerSDK_global.h"

#include <QThread>
#include <QSettings>
#include <QDebug>
#include <QVector>
#include <network_msg.h>
#include <modbus.h>
#include <QTimer>
#include <QMutex>
#include <QAbstractSocket>

#define WORD_TEST_BOARD 0
#define TEST_TIMER_INTERVAL_MSEC 1000
#define SRV_CONTROL_UDP_TIMEOUT_MSEC 30000
#define TELEMETRY_UPDATE_MSEC 30

#define INITIAL_REPLY_BUFFER_SIZE 20

class QTcpServer;
class QNetworkSession;
class QTcpSocket;
class QUdpSocket;

namespace roboctrl
{

class ROBOCONTROLLERSDKSHARED_EXPORT QRobotServer : public QThread
{
    Q_OBJECT

public:
    explicit QRobotServer(quint16 serverUdpControl=14560, quint16 statusMulticastPort=14565,
                          quint16 serverUdpStatusListener=14550, quint16 serverUdpStatusSender=14555,
                          quint16 serverTcpPort=14500, bool testMode=false, QObject *parent=0); ///< Default constructor
    virtual ~QRobotServer(); ///< Destructor

signals:
    
public slots:

private slots:    
    void onNewTcpConnection(); ///< Called for each incoming TCP connection
    void onTcpClientDisconnected(); ///< Called when a client disconnects from TCP Server

    void onTcpReadyRead(); ///< Called when a new data from TCP socket is available
    void onUdpStatusReadyRead(); ///< Called when a new data from UDP Status socket is available
    void onUdpControlReadyRead(); ///< Called when a new data from UDP Control socket is available

    bool updateTelemetry(); ///< Called to update telemetry from RoboController

private:
    void openTcpSession(); ///< Opens TCP socket
    void openUdpStatusSession(); ///< Opens UDP Status socket
    void openUdpControlSession(); ///< Opens UDP Control socket

    void sendBlockTCP( quint16 msgCode, QVector<quint16>& data ); ///< Send data block to TCP socket
    void sendStatusBlockUDP(QHostAddress addr, quint16 msgCode, QVector<quint16>& data );///< Send data block to UDP socket
    void multicastSendTelemetry(); ///< Send telemetry in multicast

    modbus_t* initializeSerialModbus( const char *device,
                                      int baud, char parity, int data_bit,
                                      int stop_bit ); ///< Initializes Modbus Serial Connection to RoboController

    bool connectModbus( int retryCount=-1); ///< retryCount=-1 puts the server in an infinite loop trying reconnection
    bool testBoardConnection(); ///< Tests if the board has not been disconnected

    bool readMultiReg( quint16 startAddr, quint16 nReg ); ///< Called to read registers from RoboController

    // QVector used instead of QList to provide direct data access using "Data()" function
    bool writeMultiReg( quint16 startAddr, quint16 nReg, QVector<quint16> vals ); ///< Called to write registers to RoboController

//    void readSpeedsAndSend(QHostAddress addr); ///< Called to send to client the speed of the robot after receiveing a command of movement

    void releaseControl(); ///< Release the control of the robot

protected:
    virtual void run() Q_DECL_OVERRIDE;
    virtual void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

private:
    QMutex  	    mBoardMutex; ///< Mutex on Robocontroller board

    QTcpServer*     mTcpServer; ///< TCP Server Object
    QTcpSocket*     mTcpSocket; ///< TCP Socket
    
    QUdpSocket*     mUdpStatusSocket; ///< UDP Status Socket Listener
    QUdpSocket*     mUdpControlSocket; ///< UDP Control Socket Listener
    QUdpSocket*     mMulticastUdpTelemetryServer; ///< Multicast Telemetry Socket

    QSettings*      mSettings; ///< Settings in file INI

    unsigned int    mServerTcpPort; ///< Port of the TCP Server
    unsigned int    mServerUdpStatusPortListen; ///< Port of the UDP Status Listen Server
    unsigned int    mServerUdpStatusPortSend; ///< Port of the UDP Status Listen Server
    unsigned int    mServerUdpControlPortListen; ///< Port of the UDP Control Server @note The control server receives without replying, the client can control if a motion command is successfull using the Status UDP Socket.

    unsigned int    mMulticastUdpTelemetryServerPort; ///< Port of the Multicast UDP server, the server will send telemetry in multicast each @ref TELEMETRY_UPDATE_MSEC msec

    modbus_t*       mModbus;  ///< ModBus protocol implementation
    quint16         mBoardIdx;      /// Id of the connected board

    unsigned int    mReplyBufSize; ///< current size of the reply buffer
    quint16*        mReplyBuffer;  ///< dinamic reply buffer (resized only if necessary)

    bool            mBoardConnected; ///< Indicates if the RoboController board is connected

    int             mBoardTestTimerId; ///< Id of the test timer.
    int             mTcpClientCount; ///< Number of Clients connected on TCP
    int             mControlTimeoutTimerId; ///< If a client does not send control command for @ref SVR_CONTROL_UDP_TIMEOUT
    int             mTelemetryUpdateTimerId; ///< Telemetry update timer ID

    QString         mControllerClientIp; ///< Ip address of the client that took control for driving the robot using @ref getRobotControl function

    quint16         mMsgCounter; /// Counts the message sent

    quint16         mNextTcpBlockSize;      ///< Used to recover incomplete TCP block
    quint16         mNextUdpCmdBlockSize;      ///< Used to recover incomplete UDP Control block
    quint16         mNextUdpStatBlockSize;      ///< Used to recover incomplete UDP Status block

    bool            mTestMode; ///< If true the server does not connect to RoboController, but allows connection to sockets to test communications

    RobotTelemetry  mTelemetry; ///< Telemetry of the robot, updated every @ref TELEMETRY_UPDATE_MSEC msec
};

}

#endif // QROBOCONTROLLERSERVER_H
