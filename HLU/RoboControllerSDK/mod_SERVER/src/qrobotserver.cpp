#include <qrobotserver.h>

#include <QtNetwork/QtNetwork>

#include "modbus.h"
#include "modbus-private.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include "loghandler.h"
#include <errno.h>
#include <QCoreApplication>
#include <QMutex>
#include "modbus_registers.h"
#include <QDateTime>

namespace roboctrl
{

QRobotServer::QRobotServer(quint16 serverUdpControl/*=14560*/, quint16 statusMulticastPort/*=14565*/,
                           quint16 serverUdpStatusListener/*=14550*/, quint16 serverUdpStatusSender/*=14555*/,
                           quint16 serverTcpPort/*=14500*/, bool testMode, QObject *parent/*=0*/) :
    QThread(parent),
    mRoboController(NULL),
    mTcpServer(NULL),
    mTcpSocket(NULL),
    mUdpInfoServer(NULL),
    mRobotCtrlServer(NULL),
    mSettings(NULL),
    mServerTcpPort(serverTcpPort),
    mServerUdpInfoPortListen(serverUdpStatusListener),
    mServerUdpInfoPortSend(serverUdpStatusSender),
    mServerUdpControlPortListen(serverUdpControl),
    mMulticastUdpTelemetryServerPort(statusMulticastPort),
    mMsgCounter(0),
    mTestMode(testMode)
{
    // >>>>> Server Settings ini file
    QString iniPath = QCoreApplication::applicationDirPath();
    iniPath += tr("/%1.ini").arg(QCoreApplication::applicationName());
    mSettings = new QSettings( iniPath, QSettings::IniFormat, this );
    // <<<<< Server Settings ini file

    qDebug() << " ";
    qDebug() << " RoboController Server ";
    qDebug() << "=======================";
    qDebug() << " ";

    if(testMode)
        qDebug() << "*** TESTMODE ACTIVE ***";

    mBoardIdx = mSettings->value( "boardidx", "0" ).toInt();
    if( mBoardIdx==0 )
    {
        mBoardIdx = 1;
        mSettings->setValue( "boardidx", QString("%1").arg(mBoardIdx) );
        mSettings->sync();
    }

    //if(!mTestMode)
    {
        // >>>>> MOD_BUS serial communication settings
        /* Default Values:
        [SERIAL_CONNECTION]
        serialinterface=Serial port 0
        serialbaudrate=57600
        serialparity=none
        serialdatabits=8
        serialstopbits=1 */


        mSettings->beginGroup( "SERIAL_CONNECTION" );
        QString port;
        if( mTestMode )
        {
            port = QString("Simulated");
        }
        else
        {
            QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
            if( ports.isEmpty() )
            {
                QString err = tr("No serial ports available. Cannot connect to RoboController");
                qCritical() << " ";
                qCritical() << err;
                qDebug() << "Server not started";
                qDebug() << " ";

                roboctrl::RcException exc(excRoboControllerNotFound, err.toStdString().c_str() );

                throw exc;
            }

            int i = 0;
            bool found = false;
            int port_idx = 0;
            foreach( QSerialPortInfo port, ports )
            {
                if( port.portName().compare( mSettings->value( "serialinterface" ).toString() )==0 )
                {
                    port_idx = i;
                    found = true;
                    break;
                }
                ++i;
            }

            if( !found )
            {
                port_idx = 0;
                mSettings->setValue( tr("serialinterface"), ports[0].portName() );
                mSettings->sync();
            }

#ifdef Q_OS_WIN32
            //    const QString port = embracedString( ports[port_idx].portName ) +
            //            ":";
            port = tr( "%1%2").arg(ports[port_idx].portName()).arg(":");
#else
            //const QString port = ports[port_idx].physName;
            port = ports[port_idx].systemLocation();
#endif
        }

        int serialbaudrate = mSettings->value( "serialbaudrate", "0" ).toInt();
        if( serialbaudrate==0 )
        {
            serialbaudrate = 57600;
            mSettings->setValue( "serialbaudrate", QString("%1").arg(serialbaudrate) );
            mSettings->sync();
        }

        QString serialparity = mSettings->value( "serialparity", " " ).toString();
        if( serialparity==" " )
        {
            serialparity = "none";
            mSettings->setValue( "serialparity", QString("%1").arg(serialparity) );
            mSettings->sync();
        }

        char parity;
        if( QString::compare( serialparity, "odd", Qt::CaseInsensitive)==0 )
            parity = 'O';
        else if( QString::compare( serialparity, "even", Qt::CaseInsensitive)==0 )
            parity = 'E';
        else
            parity = 'N';

        int data_bit = mSettings->value( "data_bit", "0" ).toInt();
        if( data_bit==0 )
        {
            data_bit = 8;
            mSettings->setValue( "data_bit", QString("%1").arg(data_bit) );
            mSettings->sync();
        }

        int stop_bit = mSettings->value( "stop_bit", "0" ).toInt();
        if( stop_bit==0 )
        {
            stop_bit = 1;
            mSettings->setValue( "stop_bit", QString("%1").arg(stop_bit) );
            mSettings->sync();
        }

        mSettings->endGroup();


        try
        {
            mRoboController = new QRoboControllerInterface( mBoardIdx, port, serialbaudrate, parity, data_bit, stop_bit, mTestMode );
        }
        catch( RcException &e)
        {
            throw e;
        }
    }

    // >>>>> TCP configuration
    mServerTcpPort = mSettings->value( "TCP_server_port", "0" ).toUInt();
    if( mServerTcpPort==0 )
    {
        mServerTcpPort = 14500;
        mSettings->setValue( "TCP_server_port", QString("%1").arg(mServerTcpPort) );
        mSettings->sync();
    }

    openTcpSession();

    connect(mTcpServer, SIGNAL(newConnection()), this, SLOT(onNewTcpConnection()));
    // <<<<< TCP configuration
    
    // >>>>> UDP Status configuration
    mServerUdpInfoPortListen = mSettings->value( "UDP_status_server_port_listener", "0" ).toUInt();
    if( mServerUdpInfoPortListen==0 )
    {
        mServerUdpInfoPortListen = 14550;
        mSettings->setValue( "UDP_status_server_port_listener", QString("%1").arg(mServerUdpInfoPortListen) );
        mSettings->sync();
    }

    mServerUdpInfoPortSend = mSettings->value( "UDP_status_server_port_senderlistener", "0" ).toUInt();
    if( mServerUdpInfoPortSend==0 )
    {
        mServerUdpInfoPortSend = 14555;
        mSettings->setValue( "UDP_status_server_port_senderlistener", QString("%1").arg(mServerUdpInfoPortSend) );
        mSettings->sync();
    }

    openUdpStatusSession();
    // <<<<< UDP Status configuration
    
    // >>>>> UDP Control configuration
    mServerUdpControlPortListen = mSettings->value( "UDP_Control_server_port_listener", "0" ).toUInt();
    if( mServerUdpControlPortListen==0 )
    {
        mServerUdpControlPortListen = 14560;
        mSettings->setValue( "UDP_Control_server_port_listener", QString("%1").arg(mServerUdpControlPortListen) );
        mSettings->sync();
    }

    mRobotCtrlServer = new QRobotCtrlServer( mRoboController, mServerUdpControlPortListen );

    connect( mRobotCtrlServer, SIGNAL(clientGainedControl(QString)),
             this, SLOT(onCtrlGained( QString ) ) );
    connect(mRobotCtrlServer, SIGNAL(clientControlRefused(QString)),
            this, SLOT(onCtrlRefused( QString ) ) );
    connect( mRobotCtrlServer, SIGNAL(clientLostControl(QString)),
             this, SLOT(onCtrlLost( QString )) );
    // <<<<< UDP Control configuration

    // >>>>> UDP Telemetry Multicast
    mMulticastUdpTelemetryServerPort = mSettings->value( "UDP_Multicast_telemetry_port", "0" ).toUInt();
    if( mMulticastUdpTelemetryServerPort==0 )
    {
        mMulticastUdpTelemetryServerPort = 14565;
        mSettings->setValue( "UDP_Multicast_telemetry_port", QString("%1").arg(mMulticastUdpTelemetryServerPort) );
        mSettings->sync();
    }

    mRobotTelemetryServer = new QRobotTelemetryServer( mRoboController, mMulticastUdpTelemetryServerPort );
    // <<<<< UDP Telemetry Multicast

    mSettings->sync();

    mTcpClientCount = 0;

    // Start Server Thread
    this->start();

    //if(!mTestMode)
    {
        // Start Ping Timer
        mBoardTestTimerId = startTimer( TEST_TIMER_INTERVAL_MSEC, Qt::PreciseTimer );
    }
}

QRobotServer::~QRobotServer()
{
    if( mRobotCtrlServer && mRobotCtrlServer->isRunning() )
    {
        mRobotCtrlServer->terminate();
        //while(mRobotCtrlServer->isRunning());
        mRobotCtrlServer->wait();
    }
    delete mRobotCtrlServer;


    if(this->isRunning())
    {
        terminate();
        while(this->isRunning());
    }



    if(mTcpServer)
        delete mTcpServer;

    if(mUdpInfoServer)
        delete mUdpInfoServer;

    if( mRoboController )
        delete mRoboController;



}

void QRobotServer::openTcpSession()
{
    if(mTcpServer)
        delete mTcpServer;

    mTcpServer = new QTcpServer();
    mTcpServer->setMaxPendingConnections( 2 );

    if (!mTcpServer->listen( QHostAddress::Any, mServerTcpPort ))
    {
        qCritical() << tr("Unable to start the server: %1.")
                       .arg(mTcpServer->errorString());
        qCritical() << tr("Please verify that there is not another server running.");
        return;
    }

    QString ipAddress;
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();

    // use the first non-localhost IPv4 address
    for (int i = 0; i < ipAddressesList.size(); ++i)
    {
        if (ipAddressesList.at(i) == QHostAddress::LocalHost ||
                ipAddressesList.at(i).toIPv4Address())
        {
            ipAddress = ipAddressesList.at(i).toString();
            qDebug() << tr("TCP Server running on IP: %1 Port: %2" )
                        .arg(ipAddress).arg(mTcpServer->serverPort());
        }
    }
}

void QRobotServer::openUdpStatusSession()
{
    if( mUdpInfoServer )
        delete mUdpInfoServer;

    mUdpInfoServer = new QUdpSocket();

    if( !mUdpInfoServer->bind( mServerUdpInfoPortListen, /*QAbstractSocket::ReuseAddressHint|*/QAbstractSocket::ShareAddress ) )
    {
        qCritical() << tr("Unable to bind the UDP Status server on %1:%2. Error: %3")
                       .arg(mUdpInfoServer->localAddress().toString())
                       .arg(mUdpInfoServer->localPort() )
                       .arg(mUdpInfoServer->errorString());
        return;
    }

    connect( mUdpInfoServer, SIGNAL(readyRead()),
             this, SLOT(onUdpInfoServerReadyRead()) );

    qDebug() << tr("UDP Status Server listening on port %1" )
                .arg(mUdpInfoServer->localPort() );

}

void QRobotServer::onNewTcpConnection()
{
    if(mTcpSocket && mTcpSocket->state()==QTcpSocket::ConnectedState )
    {
        qDebug() << tr( "Connection from %1 refused. Only one opened connection is available.")
                    .arg( mTcpServer->nextPendingConnection()->localAddress().toString() );
        return;
    }

    mTcpSocket = mTcpServer->nextPendingConnection();

    // Disable Nable Algorithm to have low latency
    mTcpSocket->setSocketOption( QAbstractSocket::LowDelayOption, 1 );
    mTcpSocket->setSocketOption( QAbstractSocket::KeepAliveOption, 1 );

    qDebug() << tr("TCP Client connected: %1").arg( mTcpSocket->localAddress().toString() );

    connect( mTcpSocket, SIGNAL(disconnected()),
             this, SLOT(onTcpClientDisconnected()) );
    connect( mTcpSocket, SIGNAL(readyRead()),
             this, SLOT(onTcpReadyRead()) );

    QVector<quint16> data;
    data << (quint16)mBoardIdx;
    sendBlockTCP( MSG_CONNECTED, data );

    mTcpClientCount++;
}

void QRobotServer::sendBlockTCP(quint16 msgCode, QVector<quint16>& data )
{
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_3);
    out << (quint16)TCP_START_VAL; // Start word
    out << (quint16)0;      // Block size
    out << mMsgCounter;     // Message counter
    out << msgCode;         // Message Code

    ++mMsgCounter;

    // >>>>> Data
    QVector<quint16>::iterator it;
    for(it = data.begin(); it != data.end(); ++it)
        out << (quint16)(*it);
    // <<<<< Data

    out.device()->seek(0);          // Back to the beginning to set block size
    int blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)TCP_START_VAL; // Start work again
    out << (quint16)blockSize;

    mTcpSocket->write( block );
    mTcpSocket->flush();

    //QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
    //qDebug() << tr("%1 - Sent TCP msg #%2 - Code: %3").arg(timeStr).arg(mMsgCounter).arg(msgCode);
}

void QRobotServer::sendInfoBlockUDP( QHostAddress addr, quint16 msgCode, QVector<quint16>& data )
{
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_3);
    out << (quint16)UDP_START_VAL; // Start Word
    out << (quint16)0;      // Block size
    out << mMsgCounter;     // Message counter
    out << msgCode;         // Message Code

    ++mMsgCounter;

    // >>>>> Data
    QVector<quint16>::iterator it;
    for(it = data.begin(); it != data.end(); ++it)
        out << (quint16)(*it);
    // <<<<< Data

    out.device()->seek(0);          // Back to the beginning to set block size
    int blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)UDP_START_VAL; // Start Word again
    out << (quint16)blockSize;

    mUdpInfoServer->writeDatagram( block, addr, mServerUdpInfoPortSend );
    mUdpInfoServer->flush();

    //QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
    //qDebug() << tr("%1 - Sent msg #%2 -> %3 to %4:%5").arg(timeStr).arg(mMsgCounter).arg(msgCode).arg(addr.toString()).arg(mServerUdpStatusPortSend);
}

void QRobotServer::onTcpReadyRead()
{
    QDataStream in(mTcpSocket);
    in.setVersion(QDataStream::Qt_5_3);

    int headerSize = 3; // [blockSize][msgIdx][msgCode] {[start_word] is ignored in block size}

    mNextTcpBlockSize=0;
    quint16 msgCode;

    //quint32 count = 0;

    forever // Receiving data while there is data available
    {
        //qDebug() << tr("onTcpReadyRead - cycle: %1").arg(++count);

        qint64 bytesAvailable = mTcpSocket->bytesAvailable();

        if( mNextTcpBlockSize==0) // No incomplete blocks received before
        {
            if (bytesAvailable < (qint64)sizeof(quint16))
            {
                //qDebug() << PREFIX << tr("No more TCP Data available: %1 bytes").arg(bytesAvailable);
                break;
            }

            // >>>>> Searching for the start word
            int count = 0;
            quint16 val16;
            //in >> val16;

            do
            {
                in >> val16;

                if(count == bytesAvailable)
                {
                    qCritical() << PREFIX << tr("Read %1 bytes not founding TCP_START_VAL.")
                                   .arg(bytesAvailable);
                    return;
                }
                count++;

            }
            while( val16 != TCP_START_VAL );
            // <<<<< Searching for the start word

            // Datagram dimension
            in >> mNextTcpBlockSize; // Updated only if we are parsing a new block
        }


        if ( bytesAvailable < mNextTcpBlockSize)
        {
            qDebug() << PREFIX << tr("Received incomplete TCP Block... waiting for the missing data (aspected %1 bytes - received %2 bytes)").arg(mNextTcpBlockSize).arg(bytesAvailable);
            break;
        }

        // Datagram IDX
        quint16 msgIdx;
        in >> msgIdx;

        //QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        //qDebug() << tr("%1 - TCP Received msg #%2").arg(timeStr).arg(msgIdx);

        // Datagram Code
        in >> msgCode;

        switch(msgCode)
        {
        case CMD_SERVER_PING_REQ: // Sent by client to verify that Server is running
        {
            //qDebug() << tr("TCP Received msg #%1: MSG_SERVER_PING_REQ (%2)").arg(msgIdx).arg(msgCode);

            QVector<quint16> vec;
            sendBlockTCP( MSG_SERVER_PING_OK, vec );

            break;
        }

        case CMD_RD_MULTI_REG:
        {
            //qDebug() << tr("TCP Received msg #%1: CMD_RD_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

            if( !mRoboController->isConnected() )
            {
                // Buffer clearing
                mTcpSocket->read( mNextTcpBlockSize-2 );

                QVector<quint16> vec;
                sendBlockTCP( MSG_RC_NOT_FOUND, vec);

                qCritical() << PREFIX << "CMD_RD_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr; // First word to be read
            quint16 nReg;
            in >> nReg;
            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            QVector<quint16> readRegReply = mRoboController->readMultiReg( startAddr, nReg );

            if( readRegReply.isEmpty() )
            {
                QVector<quint16> vec;
                vec << CMD_RD_MULTI_REG;
                vec << startAddr;
                sendBlockTCP( MSG_FAILED, vec );
            }
            else
                sendBlockTCP( MSG_READ_REPLY, readRegReply );

            break;
        }

        case CMD_WR_MULTI_REG:
        {
            //qDebug() << tr("TCP Received msg #%1: CMD_WR_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

            if( !mRoboController->isConnected() )
            {
                // Buffer clearing
                mTcpSocket->read( mNextTcpBlockSize-2 );

                QVector<quint16> vec;
                sendBlockTCP( MSG_RC_NOT_FOUND, vec );

                qCritical() << PREFIX << "CMD_WR_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr;  // First word to be read

            // We can extract data size (nReg!) from message without asking it to client in the protocol
            int nReg = (mNextTcpBlockSize/sizeof(quint16)) - headerSize;
            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            QVector<quint16> vals;
            vals.reserve(nReg);

            for( int i=0; i<nReg; i++ )
            {
                quint16 data;
                in >> data;

                vals << data;
            }

            /*if( mTestMode )
            {
                QVector<quint16> vec;
                vec << startAddr;
                vec << nReg;
                sendBlockTCP( MSG_WRITE_OK, vec );
                break;
            }*/

            bool commOk = mRoboController->writeMultiReg( startAddr, nReg, vals );

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_WR_MULTI_REG;
                vec << startAddr;
                sendBlockTCP( MSG_FAILED, vec );

                qDebug() << PREFIX << tr("writeMultiReg failed!");
            }
            else
            {
                QVector<quint16> vec;
                vec << startAddr;
                vec << nReg;
                sendBlockTCP( MSG_WRITE_OK, vec );
            }
            break;
        }

        default:
        {
            qDebug() << tr("Received wrong message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

            qint64 bytes = mTcpSocket->bytesAvailable();
            if(bytes>0)
            {
                qDebug() << tr("Removing %1 bytes from TCP socket buffer").arg(bytes);
                char* buf = new char[bytes];
                //in.readRawData( buf, mNextTcpBlockSize );
                in.readRawData( buf, bytes );
                delete [] buf;
            }

            QVector<quint16> vec;
            sendBlockTCP(  MSG_FAILED, vec );

            break;
        }
        }

        mNextTcpBlockSize = 0;

        //QCoreApplication::processEvents( QEventLoop::AllEvents, 5 );
    }
}

void QRobotServer::onUdpInfoServerReadyRead()
{
    int headerSize = 3; // [blockSize][msgIdx][msgCode] {[start_word] is ignored in block size}

    quint16 msgCode;

    while( mUdpInfoServer->hasPendingDatagrams() ) // Receiving data while there is data available
    {
        QByteArray buffer( mUdpInfoServer->pendingDatagramSize(), 0 );
        qint64 datagramSize = mUdpInfoServer->pendingDatagramSize();

        if( buffer.size()< datagramSize )
            buffer.resize( datagramSize );

        QHostAddress addr;
        quint16 port;

        quint16 infoUdpBlockSize;

        // Extracting the full datagram from socket
        mUdpInfoServer->readDatagram( buffer.data(), buffer.size(), &addr, &port );

        // TOD0: Overwriting client address with multicast address... server now sends information in multicast
        // addr = QHostAddress( )

        QDataStream in( buffer );
        in.setVersion(QDataStream::Qt_5_3);

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
        in >> infoUdpBlockSize; // Updated only if we are parsing a new block

        if( datagramSize < infoUdpBlockSize )
        {
            qDebug() << PREFIX << tr("Received incomplete UDP Status Block... "); // This should never happens!
            break;
        }

        // Datagram IDX
        quint16 msgIdx;
        in >> msgIdx;

        //QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        //qDebug() << tr("%1 - UDP Status Received msg #%2 by %3:%4").arg(timeStr).arg(msgIdx).arg(addr.toString()).arg(port);

        // Datagram Code
        in >> msgCode;

        switch(msgCode)
        {
        case CMD_SERVER_PING_REQ: // Sent by client to verify that Info Server is running
        {
            //qDebug() << tr("UDP Status Received msg #%1: CMD_SERVER_PING_REQ (%2)").arg(msgIdx).arg(msgCode);

            QVector<quint16> vec;
            sendInfoBlockUDP( addr, MSG_SERVER_PING_OK, vec );

            break;
        }

        case CMD_RD_MULTI_REG:
        {
            //qDebug() << tr("UDP Status Received msg #%1: CMD_RD_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

            if( !mRoboController->isConnected() )
            {
                // Removing unused message from buffer
                //mUdpInfoServer->read( infoUdpBlockSize-2 ); // Not needed, the datagram is fully read at the beginning

                QVector<quint16> vec;
                sendInfoBlockUDP( addr, MSG_RC_NOT_FOUND, vec );

                qCritical() << PREFIX << "CMD_RD_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr; // First word to be read
            quint16 nReg;
            in >> nReg;
            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            QVector<quint16> readRegReply = mRoboController->readMultiReg( startAddr, nReg );

            if( readRegReply.isEmpty() )
            {
                QVector<quint16> vec;
                vec << CMD_RD_MULTI_REG;
                vec << startAddr;

                sendInfoBlockUDP( addr, MSG_FAILED, vec );
            }
            else
                sendInfoBlockUDP( addr, MSG_READ_REPLY, readRegReply );

            break;
        }       

        default:
        {
            qDebug() << tr("UDP Info server - Received message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

            /* // Not needed, the datagram is fully read at the beginning
            qint64 bytes = mUdpInfoServer->pendingDatagramSize();
            if(bytes>0)
            {
                qDebug() << tr("Removing %1 bytes from UDP Status socket buffer").arg(bytes);
                char* buf = new char[bytes];
                in.readRawData( buf, bytes );
                delete [] buf;
            }*/

            break;
        }
        }

        infoUdpBlockSize = 0;


        //QCoreApplication::processEvents( QEventLoop::AllEvents, 5 ); // TODO: Does this introduces latency?
    }
}

void QRobotServer::onTcpClientDisconnected()
{
    qDebug() << tr( "TCP Client disconnected" );

    mTcpClientCount--;
}

void QRobotServer::timerEvent(QTimerEvent *event)
{
    if( event->timerId() == mBoardTestTimerId )
    {
        /*if(mTestMode)
        {
            qDebug() << "TEST MODE - Ping Ok";
            return;
        }*/

        if( !mRoboController->testBoardConnection() )
        {
            //mBoardConnected = false;

            qCritical() << tr("Robocontroller %1 not replying. Trying reconnection...").arg(mBoardIdx);
            mRoboController->connectModbus( -1 );
        }
    }
}

void QRobotServer::run()
{
    qDebug() << tr("QRobotServer thread started");

    exec();

    qDebug() << tr("QRobotServer thread finished");
}

void QRobotServer::onCtrlGained( QString clientIP )
{
    mRobotTelemetryServer->setCtrlIP(clientIP);

    QVector<quint16> vec;
    sendInfoBlockUDP( QHostAddress(clientIP), MSG_ROBOT_CTRL_OK, vec ); // Robot controlled by client
}

void QRobotServer::onCtrlRefused( QString clientIP )
{
    QVector<quint16> vec;
    sendInfoBlockUDP( QHostAddress(clientIP), MSG_ROBOT_CTRL_KO, vec ); // Robot not controlled by client
}


void QRobotServer::onCtrlLost( QString clientIP )
{
    mRobotTelemetryServer->setCtrlIP( QString());

    QVector<quint16> vec;
    sendInfoBlockUDP( QHostAddress(clientIP), MSG_ROBOT_CTRL_RELEASED, vec ); // Robot not controlled by any client
}


}
