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
#define TEST_TIMER_INTERVAL 1000

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
    explicit QRobotServer(int serverUdpControl=4550,int serverUdpStatus=4560,int serverTcpPort=4500, QObject *parent=0); ///< Default constructor
    virtual ~QRobotServer(); ///< Destructor

signals:
    
public slots:

private slots:
    void openTcpSession(); ///< Called when a new Tcp session is opened
    void openUdpStatusSession(); ///< Called when a new Udp Status session is opened
    void openUdpControlSession(); ///< Called when a new Udp Control session is opened
    void onNewTcpConnection(); ///< Called for each incoming TCP connection
    void onNewUdpStatusConnection(); ///< Called for each incoming UDP Status connection
    void onNewUdpControlConnection(); ///< Called for each incoming UDP Control  connection
    void onTcpReadyRead(); ///< Called when a new data from TCP socket is available
    void onUdpStatusReadyRead(); ///< Called when a new data from UDP Status socket is available
    void onUdpControlReadyRead(); ///< Called when a new data from UDP Control socket is available
    void onClientDisconnected(); ///< Called when a client disconnects

private:
    void sendBlock( QAbstractSocket* socket, quint16 msgCode, QVector<quint16>& data ); ///< Send data block to socket

    modbus_t* initializeSerialModbus( const char *device,
                                      int baud, char parity, int data_bit,
                                      int stop_bit ); ///< Initializes Modbus Serial Connection to RoboController

    bool connectModbus( int retryCount=-1); /*< retryCount=-1 puts the server in an infinite loop trying reconnection */
    bool testBoardConnection(); ///< Tests if the board has not been disconnected

    bool readMultiReg( quint16 startAddr, quint16 nReg ); ///< Called to read registers from RoboController

    // QVector used instead of QList to provide direct data access using "Data()" function
    bool writeMultiReg( quint16 startAddr, quint16 nReg, QVector<quint16> vals ); ///< Called to write registers to RoboController

protected:
    virtual void run() Q_DECL_OVERRIDE;
    virtual void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

private:
    QMutex  	    mBoardMutex; ///< Mutex on Robocontroller board

    QTcpServer*     mTcpServer; ///< TCP Server Object
    QTcpSocket*     mTcpSocket; ///< TCP Socket
    
    QUdpSocket*     mUdpStatusSocket; ///< UDP Status Socket
    QUdpSocket*     mUdpControlSocket; ///< UDP Control Socket

    QSettings*      mSettings; ///< Settings in file INI

    unsigned int    mServerTcpPort; ///< Port of the TCP Server
    unsigned int    mServerUdpStatusPort; ///< Port of the UDP Status Server
    unsigned int    mServerUdpControlPort; ///< Port of the UDP Control Server

    modbus_t*       mModbus;  ///< ModBus protocol implementation
    quint16         mBoardIdx;      /// Id of the connected board

    unsigned int    mReplyBufSize; ///< current size of the reply buffer
    quint16*        mReplyBuffer;  ///< dinamic reply buffer (resized only if necessary)

    bool            mBoardConnected; ///< Indicates if the RoboController board is connected

    int             mTestTimerId; ///< Id of the test timer.
    int             mClientCount; /// Number of connected Client

    quint16         mMsgCounter; /// Counts the message sent

    quint16         mNextTcpBlockSize;      ///< Used to recover incomplete TCP block
    quint16         mNextUdpCmdBlockSize;      ///< Used to recover incomplete UDP Control block
    quint16         mNextUdpStatBlockSize;      ///< Used to recover incomplete UDP Status block
};

}

#endif // QROBOCONTROLLERSERVER_H
