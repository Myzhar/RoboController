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
    mModbus(NULL),
    mReplyBuffer(NULL),
    mBoardConnected(false),
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
        qDebug() << " TESTMODE ACTIVE ";

    mReplyBufSize = INITIAL_REPLY_BUFFER_SIZE;
    mReplyBuffer = new quint16[mReplyBufSize];

    mBoardIdx = mSettings->value( "boardidx", "0" ).toInt();
    if( mBoardIdx==0 )
    {
        mBoardIdx = 1;
        mSettings->setValue( "boardidx", QString("%1").arg(mBoardIdx) );
        mSettings->sync();
    }

    if(!mTestMode)
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
        const QString port = tr( "%1%2").arg(ports[port_idx].portName()).arg(":");
#else
        //const QString port = ports[port_idx].physName;
        const QString port = ports[port_idx].systemLocation();
#endif

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

        int count = 0;



        qDebug() << tr("#%1 - Initializing connection to RoboController Id: %2").arg(count+1).arg(mBoardIdx);

        while( !initializeSerialModbus( port.toLatin1().data(),
                                        serialbaudrate, parity, data_bit, stop_bit ) &&
               count < 10 )
        {
            qWarning() << tr( "Failed to initialize mod_bus on port: %1").arg(port);
            qWarning() << tr("Trying again in one second...");

            count++;

            msleep( 1000 );
            qDebug() << tr("#%1 - Initializing connection to RoboController Id: %2").arg(count+1).arg(mBoardIdx);
        }

        if( count > 10 )
        {
            QString err = tr("* Robocontroller not connected in 10 seconds. Server not started!");
            qCritical() << " ";
            qCritical() << err;

            roboctrl::RcException exc(excRoboControllerNotFound, err.toStdString().c_str() );

            throw exc;
        }


        mSettings->endGroup();


        // >>>>> Board connection
        bool res = connectModbus( 10 );
        if( !res )
        {
            QString err = tr("Failed to connect to modbus on port: %1").arg(port);
            qCritical() << "Server not started";
            qCritical() << err;

            roboctrl::RcException exc(excRoboControllerNotFound, err.toStdString().c_str() );

            throw exc;
        }

        qDebug() << tr("RoboController connected");
        // <<<<< Board connection

        // <<<<< MOD_BUS serial communication settings
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

    mRobotCtrlServer = new QRobotCtrlServer( mServerUdpControlPortListen, this );

    connect( mRobotCtrlServer, SIGNAL(clientGainedControl(QString)),
             this, SLOT(onCtrlGained( QString ) ) );
    connect(mRobotCtrlServer, SIGNAL(clientControlRefused(QString)),
            this, SLOT(onCtrlRefused( QString ) ) );
    connect( mRobotCtrlServer, SIGNAL(clientLostControl(QString)),
             this, SLOT(onCtrlLost( QString )) );
    // <<<<< UDP Control configuration

    // >>>>> UDP Multicast
    mMulticastUdpTelemetryServerPort = mSettings->value( "UDP_Multicast_telemetry_port", "0" ).toUInt();
    if( mMulticastUdpTelemetryServerPort==0 )
    {
        mMulticastUdpTelemetryServerPort = 14565;
        mSettings->setValue( "UDP_Multicast_telemetry_port", QString("%1").arg(mMulticastUdpTelemetryServerPort) );
        mSettings->sync();
    }

    mUdpMulticastTelemetryServer = new QUdpSocket();
    // Multicast on the subnet
    mUdpMulticastTelemetryServer->setSocketOption( QAbstractSocket::MulticastTtlOption, 1 );
    // <<<<< UDP Multicast

    mSettings->sync();

    mTcpClientCount = 0;

    // Start telemetry update timer
    mTelemetryUpdateTimerId = startTimer( TELEMETRY_UPDATE_MSEC, Qt::PreciseTimer );

    // Start Server Thread
    this->start();

    if(!mTestMode)
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

    if(mUdpMulticastTelemetryServer)
        delete mUdpMulticastTelemetryServer;

    if( mModbus )
    {
        modbus_close( mModbus );
        modbus_free( mModbus );
    }

    if(mReplyBuffer)
        delete [] mReplyBuffer;
}

void QRobotServer::openTcpSession()
{
    if(mTcpServer)
        delete mTcpServer;

    mTcpServer = new QTcpServer(this);
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
    out.setVersion(QDataStream::Qt_5_2);
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
    out.setVersion(QDataStream::Qt_5_2);
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

void QRobotServer::multicastSendTelemetry()
{
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_2);
    out << (quint16)UDP_START_VAL; // Start Word
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
    int blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)UDP_START_VAL; // Start Word again
    out << (quint16)blockSize;

    int res = mUdpMulticastTelemetryServer->writeDatagram( block,
                                                           QHostAddress(MULTICAST_DATA_SERVER_IP),
                                                           mMulticastUdpTelemetryServerPort );

    if( -1==res )
    {
        qDebug() << tr("[%1] Missed telemetry sending")
                    .arg(QDateTime::fromMSecsSinceEpoch(timestamp).toString(Qt::ISODate));
    }
}

void QRobotServer::onTcpReadyRead()
{
    QDataStream in(mTcpSocket);
    in.setVersion(QDataStream::Qt_5_2);

    int headerSize = 3; // [blockSize][msgIdx][msgCode] {[start_word] is ignored in block size}

    mNextTcpBlockSize=0;
    quint16 msgCode;

    forever // Receiving data while there is data available
    {
        qint64 bytesAvailable = mTcpSocket->bytesAvailable();

        if( mNextTcpBlockSize==0) // No incomplete blocks received before
        {
            if (bytesAvailable < (qint64)sizeof(quint16))
            {
                //qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
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
                    qCritical() << Q_FUNC_INFO << tr("Read %1 bytes not founding TCP_START_VAL.")
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
            qDebug() << Q_FUNC_INFO << tr("Received incomplete TCP Block... waiting for the missing data (aspected %1 bytes - received %2 bytes)").arg(mNextTcpBlockSize).arg(bytesAvailable);
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

            if( !mBoardConnected )
            {
                // Dovrebbe essere così:
                mTcpSocket->read( mNextTcpBlockSize-2 ); // Tolgo i byte rimanenti dal buffer

                QVector<quint16> vec;
                sendBlockTCP( MSG_RC_NOT_FOUND, vec);

                qCritical() << Q_FUNC_INFO << "CMD_RD_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr; // First word to be read
            quint16 nReg;
            in >> nReg;
            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            bool commOk = readMultiReg( startAddr, nReg );

            QVector<quint16> readRegReply;

            readRegReply.resize( nReg+2 );

            readRegReply[0] = (quint16)startAddr;
            readRegReply[1] = (quint16)nReg;
            memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) );

            if( !commOk )
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

            if( !mBoardConnected )
            {
                // Dovrebbe essere così:
                mTcpSocket->read( mNextTcpBlockSize-2 ); // Tolgo i byte rimanenti dal buffer

                QVector<quint16> vec;
                sendBlockTCP( MSG_RC_NOT_FOUND, vec );

                qCritical() << Q_FUNC_INFO << "CMD_WR_MULTI_REG - Board not connected!";
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

            bool commOk = writeMultiReg( startAddr, nReg, vals );

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_WR_MULTI_REG;
                vec << startAddr;
                sendBlockTCP( MSG_FAILED, vec );
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

        QCoreApplication::processEvents( QEventLoop::AllEvents, 5 );
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

                qCritical() << Q_FUNC_INFO << tr("Read %1 bytes not founding UDP_START_VAL. Stream status: %2")
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
            qDebug() << Q_FUNC_INFO << tr("Received incomplete UDP Status Block... "); // This should never happens!
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

            if( !mBoardConnected )
            {
                // Removing unused message from buffer
                //mUdpInfoServer->read( infoUdpBlockSize-2 ); // Not needed, the datagram is fully read at the beginning

                QVector<quint16> vec;
                sendInfoBlockUDP( addr, MSG_RC_NOT_FOUND, vec );

                qCritical() << Q_FUNC_INFO << "CMD_RD_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr; // First word to be read
            quint16 nReg;
            in >> nReg;
            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            bool commOk = readMultiReg( startAddr, nReg );

            QVector<quint16> readRegReply;

            readRegReply.resize( nReg+2 );

            readRegReply[0] = (quint16)startAddr;
            readRegReply[1] = (quint16)nReg;
            memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) );

            if( !commOk )
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

            /*case CMD_WR_MULTI_REG: // Info Server should not write on RoboController registers!!!
        {
            //qDebug() << tr("UDP Status Received msg #%1: CMD_WR_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

            if( !mBoardConnected )
            {
                // Removing unused message from buffer
                // mUdpInfoServer->read( infoUdpBlockSize-2 ); // Not needed, the datagram is fully read at the beginning

                QVector<quint16> vec;
                sendInfoBlockUDP( addr, MSG_RC_NOT_FOUND, vec );

                qCritical() << Q_FUNC_INFO << "CMD_RD_MULTI_REG - Board not connected!";
                break;
            }

            quint16 startAddr;
            in >> startAddr;  // First word to be read

            // We can extract data size (nReg!) from message without asking it to client in the protocol
            int nReg = (infoUdpBlockSize/sizeof(quint16)) - headerSize;

            //qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

            QVector<quint16> vals;
            vals.reserve(nReg);

            for( int i=0; i<nReg; i++ )
            {
                quint16 data;
                in >> data;

                vals << data;
            }

            bool commOk = writeMultiReg( startAddr, nReg, vals );

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_WR_MULTI_REG;
                vec << startAddr;
                sendInfoBlockUDP( addr, MSG_FAILED, vec );
            }
            else
            {

                //qDebug() << tr("UDP Status Received msg #%1: CMD_GET_ROBOT_CTRL (%2)").arg(msgIdx).arg(msgCode);
                mControlTimeoutTimerId = startTimer(SRV_CONTROL_UDP_TIMEOUT_MSEC );

                if( mControllerClientIp.isEmpty() || mControllerClientIp==addr.toString() )
                {
                    mControllerClientIp = addr.toString();
                    QVector<quint16> vec;
                    sendInfoBlockUDP( addr, MSG_ROBOT_CTRL_OK, vec ); // Robot control taken

                    qDebug() << tr("The client %1 has taken the control of the robot").arg(addr.toString());
                }
                else
                {
                    QVector<quint16> vec;
                    sendInfoBlockUDP( addr, MSG_ROBOT_CTRL_KO, vec ); // Robot not free
                }
                break;
            }
            break;
        } */

        default:
        {
            qDebug() << tr("Received message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

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


        QCoreApplication::processEvents( QEventLoop::AllEvents, 5 ); // TODO: Does this introduces latency?
    }
}

void QRobotServer::onTcpClientDisconnected()
{
    qDebug() << tr( "TCP Client disconnected" );

    mTcpClientCount--;
}

modbus_t* QRobotServer::initializeSerialModbus( const char *device,
                                                int baud, char parity, int data_bit,
                                                int stop_bit )
{
    if( mModbus )
    {
        modbus_close( mModbus );
        modbus_free( mModbus );
    }

    mModbus = modbus_new_rtu( device, baud, parity,
                              data_bit, stop_bit );

    return mModbus;
}

bool QRobotServer::testBoardConnection()
{
    uint16_t val;
    int nReg = 1;
    int res = modbus_read_input_registers( mModbus, WORD_TEST_BOARD, nReg, &val );

    if(res!=1)
    {
        qCritical() << PREFIX << "Board Ping failed!!!";
        /*qCritical() << PREFIX << "modbus_read_input_registers error -> " <<  modbus_strerror( errno )
                    << "[First regAddress: " << WORD_TEST_BOARD << "- #reg: " << nReg <<  "]";*/

        return false;
    }

    return true;
}

bool QRobotServer::readMultiReg( quint16 startAddr, quint16 nReg )
{
    if(mTestMode)
        return true;

    mBoardMutex.lock();
    {
        // >>>>> Reply buffer resize if needed
        if( nReg > mReplyBufSize )
        {
            mReplyBufSize *= 2;
            delete [] mReplyBuffer;
            mReplyBuffer = new quint16[mReplyBufSize];
        }
        // <<<<< Reply buffer resize if needed

        int res = modbus_read_input_registers( mModbus, startAddr, nReg, mReplyBuffer );

        if(res!=nReg)
        {
            qCritical() << PREFIX << "modbus_read_input_registers error -> " <<  modbus_strerror( errno )
                        << "[First regAddress: " << startAddr << "- #reg: " << nReg <<  "]";

            mBoardMutex.unlock();
            return false;
        }
    }
    mBoardMutex.unlock();
    return true;
}

bool QRobotServer::writeMultiReg( quint16 startAddr, quint16 nReg,
                                  QVector<quint16> vals )
{
    if(mTestMode)
        return true;

    mBoardMutex.lock();
    {
        int res = modbus_write_registers( mModbus, startAddr, nReg, vals.data() );

        if(res!=nReg)
        {
            qCritical() << PREFIX << "modbus_write_registers error -> " <<  modbus_strerror( errno )
                        << "[First regAddress: " << startAddr << "- #reg: " << nReg <<  "]";

            mBoardMutex.unlock();
            return false;
        }
    }
    mBoardMutex.unlock();
    return true;
}

bool QRobotServer::connectModbus( int retryCount/*=-1*/)
{
    if( !mModbus )
    {
        qCritical() << PREFIX << "ModBus data structure not initialized!";
        return false;
    }

    // Closing to reset active connections

    //modbus_close( mModbus);

    if( modbus_connect( mModbus ) == -1 )
    {
        qCritical() << PREFIX << "Modbus connection failed";
        return false;
    }

    int res=-1;
    // res = modbus_flush( mModbus );

    msleep( 1000 );

    timeval new_timeout;
    new_timeout.tv_sec = 2;
    new_timeout.tv_usec = 0;
    modbus_set_response_timeout( mModbus, &new_timeout );
    modbus_set_byte_timeout( mModbus, &new_timeout );

    res = modbus_set_slave( mModbus, mBoardIdx );
    if( res != 0 )
    {
        qCritical() << PREFIX << ": modbus_set_slave error -> " <<  modbus_strerror( errno );

        modbus_flush( mModbus );

        return false;
    }
    //qDebug() << PREFIX << "Modbus connected";

    int tryCount=0;
    bool ok = false;

    forever
    {
        qWarning() << tr( "- testBoardConnection - Attempt: %1").arg(tryCount+1);
        ok = testBoardConnection();
        tryCount++;

        if( tryCount==retryCount || ok )
            break;
        else
            qWarning() << tr("Trying again...");

        //msleep(1000); // Not needed there is the timeout on testBoardConnection function
    }

    if( !ok )
    {
        qCritical() << tr("Error on modbus: %1").arg(modbus_strerror( errno ));
        return false;
    }

    mBoardConnected = true;
    return true;
}

bool QRobotServer::updateTelemetry()
{
    quint16 replyBuffer[4];

    // >>>>> Telemetry update
    // WORD_TENSIONE_ALIM 8
    // WORD_ENC1_SPEED 20
    // WORD_ENC2_SPEED 21
    // WORD_RD_PWM_CH1 22
    // WORD_RD_PWM_CH2 23

    mBoardMutex.lock();
    {
        quint16 startAddr = WORD_ENC1_SPEED;
        quint16 nReg = 4;
        int res = modbus_read_input_registers( mModbus, startAddr, nReg, replyBuffer );

        if(res!=nReg)
        {
            mBoardMutex.unlock();
            return false;
        }
    }
    mBoardMutex.unlock();

    double speed0;
    if(replyBuffer[0] < 32768)  // Speed is integer 2-complement!
        speed0 = ((double)replyBuffer[0])/1000.0;
    else
        speed0 = ((double)(replyBuffer[0]-65536))/1000.0;
    mTelemetry.LinSpeedLeft = speed0;

    double speed1;
    if(replyBuffer[1] < 32768)  // Speed is integer 2-complement!
        speed1 = ((double)replyBuffer[1])/1000.0;
    else
        speed1 = ((double)(replyBuffer[1]-65536))/1000.0;
    mTelemetry.LinSpeedRight = speed1;

    mTelemetry.PwmLeft = replyBuffer[2];
    mTelemetry.PwmRight = replyBuffer[3];

    // TODO mTelemetry.RpmLeft = // CALCULATE!!!
    // TODO mTelemetry.RpmRight = // CALCULATE!!!

    mBoardMutex.lock();
    {
        quint16 startAddr = WORD_TENSIONE_ALIM;
        quint16 nReg = 1;
        int res = modbus_read_input_registers( mModbus, startAddr, nReg, replyBuffer );

        if(res!=nReg)
        {
            mBoardMutex.unlock();
            return false;
        }
    }
    mBoardMutex.unlock();

    mTelemetry.Battery = ((double)replyBuffer[0])/1000.0;

    return true;

}

void QRobotServer::timerEvent(QTimerEvent *event)
{
    if( event->timerId() == mTelemetryUpdateTimerId )
    {
        if( updateTelemetry() )
            multicastSendTelemetry();
    }
    else if( event->timerId() == mBoardTestTimerId )
    {
        if(mTestMode)
        {
            qDebug() << "TEST MODE - Ping Ok";
            return;
        }

        if( !testBoardConnection() )
        {
            mBoardConnected = false;

            qCritical() << tr("Robocontroller %1 not replying. Trying reconnection...").arg(mBoardIdx);
            connectModbus( -1 );
        }
        else
        {
            mBoardConnected = true;
            //qDebug() << "Ping Ok";
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
    mTelemetry.CtrlClientIP = clientIP;

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
    mTelemetry.CtrlClientIP = QString();

    QVector<quint16> vec;
    sendInfoBlockUDP( QHostAddress(clientIP), MSG_ROBOT_CTRL_RELEASED, vec ); // Robot not controlled by any client
}


}
