#ifndef QROBOCONTROLLERSERVER_H
#define QROBOCONTROLLERSERVER_H

#include "RoboControllerSDK_global.h"

#include <QThread>
#include <QSettings>
#include <QDebug>
#include <QVector>
#include <network_msg.h>
#include <QTimer>
#include <QMutex>
#include <QAbstractSocket>
#include "qrobotctrlserver.h"
#include "qrobottelemetryserver.h"
#include "qrobocontrollerinterface.h"

#define TEST_TIMER_INTERVAL_MSEC 1000

class QTcpServer;
class QNetworkSession;
class QTcpSocket;
class QUdpSocket;

namespace roboctrl
{

class ROBOCONTROLLERSDKSHARED_EXPORT QRobotServer : public QThread
{
    Q_OBJECT

    friend class QRobotCtrlServer; ///< @ref QRobotCtrlServer must be able to access to RoboController using @ref writeMultiReg function
    friend class QRobotTelemetryServer; ///< @ref QRobotTelemetryServer must be able to access to RoboController using @ref readMultiReg function

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
    void onUdpInfoServerReadyRead(); ///< Called when a new data from UDP Status socket is available    

    void onCtrlGained( QString clientIP ); ///< Called when the client gains the control of the robot
    void onCtrlRefused( QString clientIP ); ///< Called when the client tries to get control of the robot, but it is just assigned to another client
    void onCtrlLost( QString clientIP ); ///< Called when the client loses control of the robot for a release or a timeout

private:
    void openTcpSession(); ///< Opens TCP socket
    void openUdpStatusSession(); ///< Opens UDP Status socket


    void sendBlockTCP( quint16 msgCode, QVector<quint16>& data ); ///< Send data block to TCP socket

    void sendInfoBlockUDP(QHostAddress addr, quint16 msgCode, QVector<quint16>& data );///< Send data block to UDP socket

protected:
    virtual void run() Q_DECL_OVERRIDE;
    virtual void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

private:
    QRoboControllerInterface* mRoboController; ///< Pointer to RoboController interface

    // >>>>> Servers
    QTcpServer*     mTcpServer; ///< TCP Server Object
    QTcpSocket*     mTcpSocket; ///< TCP Socket
    
    QUdpSocket*     mUdpInfoServer; ///< UDP Info Socket Listener   TODO: Move to its own server


    QRobotCtrlServer* mRobotCtrlServer; ///< Control Server Thread
    QRobotTelemetryServer* mRobotTelemetryServer; ///< Telemetry Server
    // <<<<< Servers

    QSettings*      mSettings; ///< Settings in file INI

    unsigned int    mServerTcpPort; ///< Port of the TCP Server
    unsigned int    mServerUdpInfoPortListen; ///< Port of the UDP Info Listen Server
    unsigned int    mServerUdpInfoPortSend; ///< Port of the UDP Info Listen Server
    unsigned int    mServerUdpControlPortListen; ///< Port of the UDP Control Server @note The control server receives without replying, the client can control if a motion command is successfull using the Status UDP Socket.

    unsigned int    mMulticastUdpTelemetryServerPort; ///< Port of the Multicast UDP server, the server will send telemetry in multicast each @ref TELEMETRY_UPDATE_MSEC msec


    quint16         mBoardIdx;

    int             mBoardTestTimerId; ///< Id of the test timer.
    int             mTcpClientCount; ///< Number of Clients connected on TCP


    quint16         mMsgCounter; /// Counts the message sent

    quint16         mNextTcpBlockSize;      ///< Used to recover incomplete TCP block
    //quint16         mNextUdpCmdBlockSize;      ///< Used to recover incomplete UDP Control block
    //quint16         mNextUdpStatBlockSize;      ///< Used to recover incomplete UDP Status block

    bool            mTestMode; ///< If true the server does not connect to RoboController, but allows connection to sockets to test communications
};

}

#endif // QROBOCONTROLLERSERVER_H
