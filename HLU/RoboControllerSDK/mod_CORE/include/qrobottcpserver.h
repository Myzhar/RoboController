#ifndef QROBOCONTROLLERSERVER_H
#define QROBOCONTROLLERSERVER_H

#include "RoboControllerSDK_global.h"

#include <QThread>
#include <QSettings>
#include <QDebug>
#include <QtTest/QTest>
#include <QVector>
#include <network_msg.h>
#include <modbus.h>
#include <QTimer>

#define WORD_TEST_BOARD 0
#define TEST_TIMER_INTERVAL 1000

#define INITIAL_REPLY_BUFFER_SIZE 20

class QTcpServer;
class QNetworkSession;
class QTcpSocket;

namespace roboctrl
{

class ROBOCONTROLLERSDKSHARED_EXPORT QRobotTcpServer : public QThread
{
    Q_OBJECT

public:
    explicit QRobotTcpServer(int serverPort=4500, QObject *parent=0); ///< Default constructor
    virtual ~QRobotTcpServer(); ///< Destructor
    
signals:
    
public slots:

private slots:
    void onSessionOpened(); ///< Called when a new session is opened
    void onNewConnection(); ///< Called for each incoming connection
    void onReadyRead(); ///< Called when a new data from network is available
    void onClientDisconnected(); ///< Called when a client disconnects

private:
    void sendBlock( quint16 msgCode, QVector<quint16>& data ); ///< Send data block to TCP

    modbus_t* initializeSerialModbus( const char *device,
                                      int baud, char parity, int data_bit,
                                      int stop_bit ); ///< Initializes Modbus Serial Connection to RoboController

    bool connectModbus( int retryCount=-1); /*< retryCount=-1 puts the server in an infinite loop trying reconnection */
    bool testBoardConnection(); ///< Tests if the board has not been disconnected

    bool readMultiReg( quint16 startAddr, quint16 nReg ); ///< Called to read a registers from RoboController

    // QVector used instead of QList to provide direct data access using "Data()" function
    bool writeMultiReg( quint16 startAddr, quint16 nReg, QVector<quint16> vals );

protected:
    virtual void run() Q_DECL_OVERRIDE;
    virtual void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

private:
    QTcpServer*     mTcpServer; ///< TCP Server Object
    QTcpSocket*     mClientSocket; ///< TCP Socket

    QSettings*      mSettings; ///< Settings in file INI

    unsigned int    mServerPort;

    modbus_t*       mModbus;  ///< ModBus protocol implementation
    quint16         mBoardIdx;      /// Id of the connected board

    unsigned int    mReplyBufSize; ///< current size of the reply buffer
    quint16*        mReplyBuffer;  ///< dinamic reply buffer (resized only if necessary)

    bool            mBoardConnected; ///< Indicates if the RoboController board is connected

    int             mTestTimerId; ///< Id of the test timer.
    int             mClientCount; /// Number of connected Client

    bool            mTestServerMode; /// Server test without board connection

    quint16         mMsgCounter; /// Counts the message sent

    quint16         mNextTcpBlockSize;      ///< Used to recover incomplete TCP block
};

}

#endif // QROBOCONTROLLERSERVER_H
