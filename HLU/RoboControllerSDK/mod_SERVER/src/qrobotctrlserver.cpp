#include "qrobotctrlserver.h"
#include "network_msg.h"
#include <QCoreApplication>
#include <loghandler.h>

namespace roboctrl
{

#define SRV_CONTROL_UDP_TIMEOUT_MSEC 10000

QRobotCtrlServer::QRobotCtrlServer( QRoboControllerInterface *robocontroller, quint16 listenPort, QObject *parent ):
    QThread(parent),
    mUdpCtrlReceiver(NULL),
    mControllerClientIp(""),
    mRoboController(robocontroller)

{
    mControlTimeoutTimerId = 0;
    mUdpServerCtrlListenPort = listenPort;

    openUdpControlSession();
}

QRobotCtrlServer::~QRobotCtrlServer()
{
    closeUdpControlSession();

}

void QRobotCtrlServer::run()
{
    qDebug() << tr("Control Server Thread started");

    //openUdpControlSession();

    exec();

    //closeUdpControlSession();

    qDebug() << tr("Control Server Thread finished.");

}

void QRobotCtrlServer::openUdpControlSession()
{
    if( mUdpCtrlReceiver )
        delete mUdpCtrlReceiver;

    mUdpCtrlReceiver = new QUdpSocket();

    if( !mUdpCtrlReceiver->bind( mUdpServerCtrlListenPort, /*QAbstractSocket::ReuseAddressHint|*/QAbstractSocket::ShareAddress  ) )
    {
        qCritical() << tr("Unable to bind the UDP Control server on %1:%2. Error: %3")
                       .arg(mUdpCtrlReceiver->localAddress().toString())
                       .arg(mUdpCtrlReceiver->localPort() )
                       .arg(mUdpCtrlReceiver->errorString()) ;

        return;
    }

    connect( mUdpCtrlReceiver, SIGNAL(readyRead()),
             this, SLOT(onUdpCtrlReadyRead()) );

    qDebug() << tr("UDP Control Server listening on port %1" )
                .arg(mUdpCtrlReceiver->localPort() );

    start();
}

void QRobotCtrlServer::closeUdpControlSession()
{
    terminate();
    wait(5000);

    disconnect( mUdpCtrlReceiver, SIGNAL(readyRead()),
             this, SLOT(onUdpCtrlReadyRead()) );

    if( mUdpCtrlReceiver )
        delete mUdpCtrlReceiver;
    mUdpCtrlReceiver = NULL;

    qDebug() << tr("UDP Control Server closed" );
}

void QRobotCtrlServer::onUdpCtrlReadyRead()
{
    int headerSize = 3; // [blockSize][msgIdx][msgCode] {[start_word] is ignored in block size}

    quint16 msgCode;

    while( mUdpCtrlReceiver->hasPendingDatagrams() ) // Receiving data while there is data available
    {
        QByteArray buffer( mUdpCtrlReceiver->pendingDatagramSize(), 0 );
        qint64 datagramSize = mUdpCtrlReceiver->pendingDatagramSize();

        if( buffer.size()< datagramSize )
            buffer.resize( datagramSize );

        QHostAddress addr;
        quint16 port;

        quint16 cmdUdpBlockSize=0;

        // Extracting the full datagram from socket
        mUdpCtrlReceiver->readDatagram( buffer.data(), buffer.size(), &addr, &port );

        QDataStream in( buffer );
        in.setVersion(QDataStream::Qt_5_2);

        // >>>>> Searching for start character
        int count = 0;
        quint16 val16;
        in >> val16;

        while( val16 != UDP_START_VAL )
        {
            count++;
            if(count == datagramSize)
            {
                QDataStream::Status st = in.status();

                qCritical() << PREFIX << tr("Read %1 bytes not founding UDP_START_VAL. Stream status: %2")
                               .arg(datagramSize).arg(st);
                return;
            }

            in >> val16;
        }
        // <<<<< Searching for start character

        // Datagram dimension
        in >> cmdUdpBlockSize;

        if( datagramSize < cmdUdpBlockSize )
        {
            qDebug() << PREFIX << tr("Received incomplete UDP Control Block..."); // This should never happens!
            break;
        }

        // Datagram IDX
        quint16 msgIdx;
        in >> msgIdx;

        //QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        //qDebug() << tr("%1 - UDP Status Received msg #%2").arg(timeStr).arg(msgIdx);

        // Datagram Code
        in >> msgCode;

        switch(msgCode)
        {
        case CMD_WR_MULTI_REG:
        {

            //qDebug() << tr("UDP Control Received msg #%1: CMD_WR_MULTI_REG").arg(msgIdx);

            if( !mRoboController->isConnected() )
            {
                // Removing unused message from buffer
                //mUdpCtrlReceiver->read( cmdUdpBlockSize-2 );

                qCritical() << PREFIX << "CMD_WR_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr;  // First word to be read

            // We can extract data size (nReg!) from message without asking it to client in the protocol
            int nReg = (cmdUdpBlockSize/sizeof(quint16)) - headerSize;

            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            QVector<quint16> vals;
            vals.reserve(nReg);

            for( int i=0; i<nReg; i++ )
            {
                quint16 data;
                in >> data;

                vals << data;
            }

            // >>>>> Control taken test
            // Before sending the control command to Robocontroller we must test
            // if the client has the control of the robot. If not we send the information
            // to the client using the UDP Status Socket
            if(addr.toString()!= mControllerClientIp) // The client has no control of the robot
            {
                /*QVector<quint16> vec;
                sendInfoBlockUDP( addr, MSG_ROBOT_CTRL_KO, vec ); // Robot not controlled by client */

                if(mControllerClientIp.isEmpty())
                    qDebug() << PREFIX << tr("The client %1 cannot send commands before taking control of the robot").arg(addr.toString());
                else
                    qDebug() << PREFIX << tr("The client %1 is controlling the robot. %2 cannot send commands").arg(mControllerClientIp).arg(addr.toString());

                emit clientControlRefused( addr.toString() );
                break;
            }

            // >>>>> Reset control timer time
            if( mControlTimeoutTimerId!=0 )
                killTimer( mControlTimeoutTimerId );
            mControlTimeoutTimerId = startTimer( SRV_CONTROL_UDP_TIMEOUT_MSEC );
            //qDebug() << PREFIX << tr("Control Timer restarted");
            // <<<<< Reset control timer time

            bool commOk = mRoboController->writeMultiReg( startAddr, nReg, vals );

            if( !commOk )
            {
                qDebug() << tr("Error writing %1 registers, starting from %2").arg(nReg).arg(startAddr);
            }

            break;
        }

        case CMD_GET_ROBOT_CTRL: // The client is trying to get control of the movements of the robot
        {
            //qDebug() << tr("UDP Status Received msg #%1: CMD_GET_ROBOT_CTRL (%2)").arg(msgIdx).arg(msgCode);
            if( mControlTimeoutTimerId!=0 )
                killTimer( mControlTimeoutTimerId );
            mControlTimeoutTimerId = startTimer( SRV_CONTROL_UDP_TIMEOUT_MSEC );

            if( mControllerClientIp.isEmpty() || mControllerClientIp==addr.toString() )
            {
                //qDebug() << tr("mControllerClientIp: %1").arg(mControllerClientIp);

                mControllerClientIp = addr.toString();

                emit clientGainedControl( mControllerClientIp );

                qDebug() << tr("The client %1 has taken the control of the robot").arg(addr.toString());
            }
            else
            {
                emit clientControlRefused( addr.toString());
            }
            break;
        }

        case CMD_REL_ROBOT_CTRL: // The client released the control of the movements of the robot
        {
            releaseControl();

            break;
        }

        default:
        {
            qDebug() << tr("UDP Control Received wrong message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

            /* // Not needed, the datagram is fully read at the beginning
            qint64 bytes = mUdpControlReceiver->pendingDatagramSize();
            if(bytes>0)
            {
                qDebug() << tr("Removing %1 bytes from UDP Control socket buffer").arg(bytes);
                char* buf = new char[bytes];
                in.readRawData( buf, bytes );
                delete [] buf;
            }*/

            break;
        }
        }

        cmdUdpBlockSize = 0;

        //QCoreApplication::processEvents( QEventLoop::AllEvents, 1 ); // TODO: Does this introduces latency?
    }
}

void QRobotCtrlServer::timerEvent(QTimerEvent *event)
{
    if( event->timerId() == mControlTimeoutTimerId )
    {
        qDebug() << tr("Control timeout. Elapsed %1 msec since last control command").arg(SRV_CONTROL_UDP_TIMEOUT_MSEC);

        releaseControl();
    }
}

void QRobotCtrlServer::releaseControl( )
{
    if(!mControllerClientIp.isEmpty())
    {
        if( mControlTimeoutTimerId!=0 )
            killTimer( mControlTimeoutTimerId );
        mControlTimeoutTimerId = 0;

        emit clientLostControl( mControllerClientIp );

        qDebug() << tr("The client %1 has released the control of the robot").arg(mControllerClientIp);

        mControllerClientIp = "";
    }    
}

}

