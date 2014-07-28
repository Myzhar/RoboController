#include "qrobottelemetryserver.h"
#include "qrobotserver.h"
#include <QDateTime>
#include <QTimerEvent>
#include "modbus_registers.h"

#include <loghandler.h>

namespace roboctrl
{

#define TELEMETRY_UPDATE_MSEC 30

QRobotTelemetryServer::QRobotTelemetryServer(QRoboControllerInterface* robocontroller, quint16 sendPort, QObject *parent) :
    QThread(parent),
    mUdpMulticastTelemetryServer(NULL),
    mUpdateTimer(NULL),
    mRoboController(robocontroller)
{
    mMulticastUdpTelemetryServerPort = sendPort;

   openUdpServerSession();
}

QRobotTelemetryServer::~QRobotTelemetryServer()
{
    closeUdpServerSession();
}

void QRobotTelemetryServer::openUdpServerSession()
{
    if(mUpdateTimer)
        delete mUpdateTimer;

    mUpdateTimer = new QTimer();
    mUpdateTimer->setInterval( TELEMETRY_UPDATE_MSEC );
    mUpdateTimer->setTimerType( Qt::PreciseTimer );

    if(mUdpMulticastTelemetryServer)
        delete mUdpMulticastTelemetryServer;
    mUdpMulticastTelemetryServer = NULL;

    mUdpMulticastTelemetryServer = new QUdpSocket();
    // Multicast on the subnet
    mUdpMulticastTelemetryServer->setSocketOption( QAbstractSocket::MulticastTtlOption, 1 );

    // Start telemetry update timer
    mUpdateTimer->start();

    connect( mUpdateTimer, SIGNAL(timeout()),
             this, SLOT(onUpdateTimerTimeout()) );

    start();
}

void QRobotTelemetryServer::closeUdpServerSession()
{
    disconnect( mUpdateTimer, SIGNAL(timeout()),
             this, SLOT(onUpdateTimerTimeout()) );

    if(mUpdateTimer)
        delete mUpdateTimer;
    mUpdateTimer = NULL;

    terminate();
    wait( 5000 );

    if(mUdpMulticastTelemetryServer)
        delete mUdpMulticastTelemetryServer;
    mUdpMulticastTelemetryServer = NULL;
}

void QRobotTelemetryServer::setCtrlIP( QString clientIP)
{
    mTelemetry.CtrlClientIP = clientIP;
}

void QRobotTelemetryServer::multicastSendTelemetry()
{
    //qDebug() << PREFIX;

    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_2);
    out << (quint16)UDP_TEL_START_VAL; // Start Word
    out << (quint16)0;      // Block size

    qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
    out << timestamp; // Timestamp

    // >>>>> Data
    out << mTelemetry.CtrlClientIP;
    out << mTelemetry.Battery;
    out << mTelemetry.LinSpeedLeft;
    out << mTelemetry.LinSpeedRight;
    out << mTelemetry.PwmLeft;
    out << mTelemetry.PwmRight;
    out << mTelemetry.RpmLeft;
    out << mTelemetry.RpmRight;
    // <<<<< Data

    out.device()->seek(0);          // Back to the beginning to set block size
    quint16 blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)UDP_TEL_START_VAL; // Start Word again
    out << (quint16)blockSize;

    int res = mUdpMulticastTelemetryServer->writeDatagram( block,
                                                           QHostAddress(MULTICAST_DATA_SERVER_IP),
                                                           mMulticastUdpTelemetryServerPort );

    if( -1==res )
    {
        qDebug() << PREFIX << tr("[%1] Missed telemetry sending")
                    .arg(QDateTime::fromMSecsSinceEpoch(timestamp).toString(Qt::ISODate));
    }
//    else
//    {
//        qDebug() << PREFIX << tr("Sent telemetry to port %1:%2").arg(MULTICAST_DATA_SERVER_IP).arg(mMulticastUdpTelemetryServerPort);
//    }
}

void QRobotTelemetryServer::run()
{
    qDebug() << tr("Telemetry Server Thread started");

    //openUdpServerSession();

    exec();

    //closeUdpServerSession();

    qDebug() << tr("Telemetry Server Thread finished.");


}

void QRobotTelemetryServer::onUpdateTimerTimeout()
{
    //qDebug() << PREFIX;

    if( updateTelemetry() )
        multicastSendTelemetry();
}

bool QRobotTelemetryServer::updateTelemetry()
{
     //qDebug() << PREFIX;

    //quint16 replyBuffer[4];

    // >>>>> Telemetry update
    // WORD_TENSIONE_ALIM 8
    // WORD_ENC1_SPEED 20
    // WORD_ENC2_SPEED 21
    // WORD_RD_PWM_CH1 22
    // WORD_RD_PWM_CH2 23

    quint16 startAddr = WORD_ENC1_SPEED;
    quint16 nReg = 4;

    QVector<quint16> reply = mRoboController->readMultiReg( startAddr, nReg );

    if( reply.isEmpty() )
        return false;

    double speed0;
    if(reply[0] < 32767)  // Speed is integer 2-complement!
        speed0 = ((double)reply[0])/1000.0;
    else
        speed0 = ((double)(reply[0]-65536))/1000.0;
    mTelemetry.LinSpeedLeft = speed0;

    double speed1;
    if(reply[1] < 32768)  // Speed is integer 2-complement!
        speed1 = ((double)reply[1])/1000.0;
    else
        speed1 = ((double)(reply[1]-65536))/1000.0;
    mTelemetry.LinSpeedRight = speed1;

    mTelemetry.PwmLeft = reply[2];
    mTelemetry.PwmRight = reply[3];

    // TODO mTelemetry.RpmLeft = // CALCULATE!!!
    // TODO mTelemetry.RpmRight = // CALCULATE!!!

    startAddr = WORD_TENSIONE_ALIM;
    nReg = 1;
    reply = mRoboController->readMultiReg( startAddr, nReg );

    mTelemetry.Battery = ((double)reply[0])/1000.0;

    return true;

}

}
