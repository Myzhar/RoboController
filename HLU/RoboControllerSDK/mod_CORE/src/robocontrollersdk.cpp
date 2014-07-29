#include <robocontrollersdk.h>
#include <rcexception.h>
#include <QDataStream>
#include <QCoreApplication>

#include <network_msg.h>
#include <modbus_registers.h>
#include <QDateTime>
#include <QSettings>
#include <QFile>
#include <QNetworkInterface>
#include <QHostAddress>
#include <loghandler.h>

namespace roboctrl
{

RoboControllerSDK::RoboControllerSDK(QString serverAddr/*=QString("127.0.0.1")*/,
                                     quint16 udpStatusPortSend/*=14550*/,
                                     quint16 udpStatusPortListen/*=14555*/,
                                     quint16 multicastUdpPort/*=14565*/,
                                     quint16 udpControlPort/*=14560*/,
                                     quint16 tcpPort/*=14500*/) :
    mTcpSocket(NULL),
    mUdpStatusSocket(NULL),
    mUdpControlSocket(NULL),
    mUdpMulticastTelemetrySocket(NULL)
{
    mStopped = true;
    mWatchDogTimeMsec = 1000;
    mMsgCounter = 0;

    // Ping Timer
    connect( &mPingTimer, SIGNAL(timeout()), this, SLOT(onPingTimerTimeout()));

    // >>>>> Default board status
    mBoardStatusValid = false;

    mBoardStatus.pidEnable = true;
    mBoardStatus.accelRampEnable = true;
    mBoardStatus.saveToEeprom = true;
    mBoardStatus.wdEnable = true;
    // <<<<< Default board status

    // >>>>> TCP Socket
    mTcpSocket = new QTcpSocket();

    mServerAddr = serverAddr;
    mTcpPort = tcpPort;

    mNextTcpBlockSize=0;

    connect(mTcpSocket, SIGNAL(readyRead()),
            this, SLOT(onTcpReadyRead()));
    connect(mTcpSocket, SIGNAL(hostFound()),
            this, SLOT(onTcpHostFound()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(onTcpError(QAbstractSocket::SocketError)));

    try
    {
        connectToTcpServer();
    }
    catch( RcException &e )
    {
        qDebug() << e.getExcMessage();
        throw e;
    }
    // <<<<< TCP Socket

    // >>>>> UDP Sockets
    mUdpControlSocket = new QUdpSocket();
    mUdpStatusSocket = new QUdpSocket();
    mUdpMulticastTelemetrySocket = new QUdpSocket();

    mUdpControlPortSend = udpControlPort;
    mUdpStatusPortSend = udpStatusPortSend;
    mUdpStatusPortListen = udpStatusPortListen;
    mUdpTelemetryMulticastPort = multicastUdpPort;

    connect( mUdpStatusSocket, SIGNAL(readyRead()),
             this, SLOT(onUdpStatusReadyRead()) );

    connect( mUdpMulticastTelemetrySocket, SIGNAL(readyRead()),
             this, SLOT(onUdpTelemetryReadyRead()) );

    connect( mUdpControlSocket, SIGNAL(error(QAbstractSocket::SocketError)),
             this, SLOT(onUdpControlError(QAbstractSocket::SocketError)) );
    connect( mUdpStatusSocket, SIGNAL(error(QAbstractSocket::SocketError)),
             this, SLOT(onUdpStatusError(QAbstractSocket::SocketError)) );

    try
    {
        connectToUdpServers();
    }
    catch( RcException &e )
    {
        qDebug() << e.getExcMessage();
        throw e;
    }

    // <<<<< UDP Sockets
    mMotorCtrlMode = mcPID; // RoboController is in PID mode by default

    // Start thread
    mStopped = false;

    mPingTimer.setTimerType( Qt::PreciseTimer );

    start();
}

RoboControllerSDK::~RoboControllerSDK()
{
    this->terminate();

    while(this->isRunning());
    disconnectTcpServer();
    disconnectUdpServers();
}

QString RoboControllerSDK::findServer(quint16 udpSendPort/*=14550*/ , quint64 udpListenPort/*=14555*/)
{
    /*if( mUdpConnected )
        return mServerAddr;*/

    QUdpSocket* udp = new QUdpSocket();
    if( !udp->bind( udpListenPort, /*QAbstractSocket::ReuseAddressHint|*/QAbstractSocket::ShareAddress ) )
    {
        qDebug() << tr("UDP error: %1").arg(udp->errorString() );
        udp->abort();
        delete udp;
        return QString();
    }

    QVector<quint16> vec;

    // >>>>> sendCommand( udp, CMD_SERVER_PING_REQ, vec  );
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_3);
    out << (quint16)UDP_START_VAL; // Start word
    out << (quint16)0;    // Block size
    out << (quint16)1;    // Message counter
    out << (quint16)CMD_SERVER_PING_REQ;       // Message Code

    out.device()->seek(0);          // Back to the beginning to set block size
    int blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)UDP_START_VAL; // Start word again
    out << (quint16)blockSize;

    udp->writeDatagram( block, QHostAddress::Broadcast, udpSendPort);
    udp->flush();
    // <<<<< sendCommand( udp, CMD_SERVER_PING_REQ, vec  );

    msleep( 500 );

    if( udp->waitForReadyRead( 10000 ) )
    {
        QHostAddress addr;
        quint16 port;

        qint64 size = udp->pendingDatagramSize();
        char* data = new char[size];

        qint64 readCount = udp->readDatagram( data, size, &addr, &port );

        if( readCount==0 || readCount!=size )
        {
            delete [] data;
            udp->abort();
            delete udp;

            return QString();
        }

        for( int i=0; i<readCount; i++ )
        {
            if( data[i]==MSG_SERVER_PING_OK )
            {
                if( i>2 && data[i-4]==4 ) // The block size must be 6
                {
                    delete [] data;
                    udp->abort();
                    delete udp;
                    return addr.toString();
                }
            }
        }

        delete [] data;
        udp->abort();
        delete udp;
        return QString(); // Correct MSG_SERVER_PING_OK not found!
    }
    else
    {
        udp->abort();
        delete udp;
        return QString();
    }
}

void RoboControllerSDK::connectToTcpServer()
{
    mTcpConnected = false;
    mTcpSocket->connectToHost( QHostAddress(mServerAddr), mTcpPort );
    if( !mTcpSocket->waitForConnected( 5000 ) )
    {
        throw RcException( excTcpNotConnected, tr("It is not possible to connect to TCP server: %1")
                           .arg(mTcpSocket->errorString() ).toLocal8Bit() );
    }

    // Disable Nable Algorithm to have low latency
    mTcpSocket->setSocketOption( QAbstractSocket::LowDelayOption, 1 );
    mTcpSocket->setSocketOption( QAbstractSocket::KeepAliveOption, 1 );

    int count=0;
    while( count < 5 && !mTcpConnected )
    {
        sleep(1);
        QCoreApplication::processEvents();
        count ++;
    }

    if( !mTcpConnected )
        throw RcException( excTcpNotConnected, tr("It is not possible to find a valid TCP server: %1")
                           .arg(mTcpSocket->errorString() ).toLocal8Bit() );

    emit tcpConnected();
}

void RoboControllerSDK::disconnectTcpServer()
{
    if(!mTcpSocket)
        return;

    mTcpSocket->disconnectFromHost();
    if( mTcpSocket->state() == QAbstractSocket::UnconnectedState ||
            mTcpSocket->waitForDisconnected(1000) )
        qDebug() << tr("TCP Socket Disconnected");
}

void RoboControllerSDK::connectToUdpServers()
{
    mUdpConnected = false;

    // >>>>> Connect to UDP Status Server

    if( !mUdpStatusSocket->bind( /*QHostAddress(mServerAddr),*/
                                 mUdpStatusPortListen,
                                 QUdpSocket::ShareAddress|QUdpSocket::ReuseAddressHint ) )
    {
        throw RcException( excUdpNotConnected, tr("It is not possible to bind UDP Status server: %1")
                           .arg(mUdpControlSocket->errorString() ).toLocal8Bit() );
    }
    // <<<<< Connect to UDP Status Server

    // >>>>> Connect to UDP Multicast Telemetry Server
    // To connect to multicast server we need that the client socket is binded
    if( !mUdpMulticastTelemetrySocket->bind( QHostAddress::AnyIPv4,
                                             mUdpTelemetryMulticastPort,
                                             QUdpSocket::ShareAddress|QUdpSocket::ReuseAddressHint ) )
    {
        throw RcException( excUdpNotConnected, tr("It is not possible to bind UDP Telemetry server: %1")
                           .arg(mUdpMulticastTelemetrySocket->errorString() ).toLocal8Bit() );
    }


    if( !mUdpMulticastTelemetrySocket->joinMulticastGroup( QHostAddress(MULTICAST_DATA_SERVER_IP) ) )
    {
        qDebug() << tr("Connection to Multicast telemetry server failed on port %1")
                    .arg(mUdpTelemetryMulticastPort);
    }
    else
    {
        qDebug() << tr("Connected to Multicast Telemetry Server. Listening on port %1").arg(mUdpTelemetryMulticastPort);
    }

    connect( &mUdpPingTimer, SIGNAL(timeout()),
             this, SLOT(onUdpTestTimerTimeout()) );

    mUdpPingTimer.start(UDP_PING_TIME_MSEC);

    mUdpConnected = true;

    emit udpConnected();
}

void RoboControllerSDK::disconnectUdpServers()
{
    /*if(mUdpControlSocket)
    {
        mUdpControlSocket->close();
        if( mUdpControlSocket->state() == QAbstractSocket::UnconnectedState ||
                mUdpControlSocket->waitForDisconnected(1000) )
            qDebug() << tr("UDP Control Socket Disconnected");
    }*/

    if(mUdpStatusSocket)
    {
        mUdpStatusSocket->close();
        if( mUdpStatusSocket->state() == QAbstractSocket::UnconnectedState ||
                mUdpStatusSocket->waitForDisconnected(1000) )
            qDebug() << tr("UDP Status Socket Disconnected");
    }

    if(mUdpMulticastTelemetrySocket)
    {
        mUdpMulticastTelemetrySocket->leaveMulticastGroup( QHostAddress(MULTICAST_DATA_SERVER_IP) );
        mUdpStatusSocket->close();
        if( mUdpMulticastTelemetrySocket->state() == QAbstractSocket::UnconnectedState ||
                mUdpMulticastTelemetrySocket->waitForDisconnected(1000) )
            qDebug() << tr("UDP Telemetry Socket Disconnected");
    }
}

void RoboControllerSDK::onTcpReadyRead()
{
    QDataStream in(mTcpSocket);
    in.setVersion(QDataStream::Qt_5_3);

    // mNextTcpBlockSize=0; WRONG!!!!!!!!
    quint16 msgCode;

    forever // Receiving data while there is data available
    {
        //QCoreApplication::processEvents( QEventLoop::AllEvents, 5 );

        qint64 bytesAvailable = mTcpSocket->bytesAvailable();

        if( mNextTcpBlockSize==0) // No incomplete blocks received before
        {
            if ( bytesAvailable < (qint64)sizeof(quint16))
            {
                //qDebug() << PREFIX << tr("No more TCP Data available");
                break;
            }

            // >>>>> Searching for the start word
            int count = 0;
            quint16 val16;

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
            qDebug() << PREFIX << tr("Received incomplete TCP Block... waiting for the missing data");
            break;
        }

        // Datagram IDX
        quint16 msgIdx;
        in >> msgIdx;

        QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        //qDebug() << tr("%1 - Received msg #%2").arg(timeStr).arg(msgIdx);

        // Datagram Code
        in >> msgCode;

        switch(msgCode)
        {
        case MSG_CONNECTED:
        {
            if( !mTcpConnected )
            {
                mTcpConnected = true;
                //qDebug() << tr("TCP Received msg #%1: MSG_CONNECTED").arg(msgIdx);
                qDebug() << tr( "Server ready: %1").arg(mTcpSocket->localAddress().toString() );

                in >> mBoardIdx;
            }
            else
            {
                qDebug() << tr( "Protocol error - received a bad confirmation: %1")
                            .arg(msgCode);

                throw RcException( excProtocolBadConfirmation, tr( "Protocol error - received a bad confirmation: %1")
                                   .arg(msgCode).toLocal8Bit() );
            }
            break;
        }

        case MSG_FAILED:
        {
            quint16 msg;
            quint16 reg;

            in>>msg;
            in>>reg;

            //qDebug() << tr("TCP Received msg #%1: MSG_FAILED - Command: %2 - Reg: %3")
            //.arg(msgIdx).arg(msg).arg(reg);
            break;
        }

        case MSG_WRITE_OK:
        {
            quint16 reg;
            quint16 nReg;

            in>>reg;
            in>>nReg;

            //qDebug() << tr("TCP Received msg #%1: MSG_WRITE_OK - StartReg: %2 - Nreg: %3")
            //.arg(msgIdx).arg(reg).arg(nReg);
            break;
        }

        case MSG_READ_REPLY:
        {
            //qDebug() << tr("TCP Received msg #%1: MSG_READ_REPLY").arg(msgIdx);
            processReplyMsg( &in );
            break;
        }

        case MSG_RC_NOT_FOUND:
        {
            qDebug() << tr("TCP Received msg #%1: MSG_RC_NOT_FOUND").arg(msgIdx);

            // TODO: handle this error
            break;
        }

        default:
            qDebug() << tr("TCP Received unknown message (%1) with msg #%2").arg(msgCode).arg(msgIdx);

            char buf[256];
            in.readRawData( buf, mNextTcpBlockSize );
            break;
        }

        mNextTcpBlockSize = 0;
    }
}

void RoboControllerSDK::onUdpTelemetryReadyRead()
{
    //quint32 telemetryReadyReadCount = 0;

    while( mUdpMulticastTelemetrySocket->hasPendingDatagrams() )
    {
        //qDebug() << tr("telemetryReadyReadCount: %1").arg(++telemetryReadyReadCount);

        QByteArray datagram;
        qint64 datagramSize = mUdpMulticastTelemetrySocket->pendingDatagramSize();

        if( datagram.size()< datagramSize )
            datagram.resize( datagramSize );

        datagram.resize( datagramSize );

        QHostAddress addr;
        quint16 port;

        mUdpMulticastTelemetrySocket->readDatagram( datagram.data(), datagram.size(), &addr, &port );

        QDataStream in( datagram );
        in.setVersion(QDataStream::Qt_5_3);

        int count = 0;
        quint16 val16;
        quint16 dataSize;
        in >> val16;

        while( val16 != UDP_TEL_START_VAL )
        {
            count++;
            if(count == datagramSize)
            {
                QDataStream::Status st = in.status();

                qCritical() << PREFIX << tr("Read %1 bytes not founding UDP_TEL_START_VAL. Stream status: %2")
                               .arg(datagramSize).arg(st);
                return;
            }
            in >> val16;
        }

        // Datagram dimension
        in >> dataSize; // Updated only if we are parsing a new block

        if( dataSize < sizeof(RobotTelemetry) )
        {
            qWarning() << PREFIX << tr("Received datagram with wrong size. Expected: %1, Received: %2")
                          .arg(sizeof(RobotTelemetry)).arg(dataSize);
        }

        in >> mLastTelemetryTimestamp;

        // >>>>> Data
        in >> mTelemetry.CtrlClientIP;
        in >> mTelemetry.Battery;
        in >> mTelemetry.LinSpeedLeft;
        in >> mTelemetry.LinSpeedRight;
        in >> mTelemetry.PwmLeft;
        in >> mTelemetry.PwmRight;
        in >> mTelemetry.RpmLeft;
        in >> mTelemetry.RpmRight;
        // <<<<< Data

        emit newTelemetryAvailable();
    }
}

void RoboControllerSDK::onUdpStatusReadyRead()
{
    quint16 msgCode;

    while( mUdpStatusSocket->hasPendingDatagrams() ) // Receiving data while there is data available
    {
        //QCoreApplication::processEvents( QEventLoop::AllEvents, 5 );

        QByteArray buffer( mUdpStatusSocket->pendingDatagramSize(), 0 );
        qint64 datagramSize = mUdpStatusSocket->pendingDatagramSize();

        if( buffer.size()< datagramSize )
            buffer.resize( datagramSize );

        QHostAddress addr;
        quint16 port;

        mUdpStatusSocket->readDatagram( buffer.data(), buffer.size(), &addr, &port );

        QDataStream in( buffer );
        in.setVersion(QDataStream::Qt_5_3);

        quint16 dataSize;

        //while( !in.atEnd() )
        {
            //QCoreApplication::processEvents( QEventLoop::AllEvents, 5 );

            //if( mNextUdpStBlockSize==0) // No incomplete blocks received before
            {
                if (datagramSize < (qint64)sizeof(quint16))
                {
                    //qDebug() << PREFIX << tr("No more TCP Data available");
                    break;
                }

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
                
                // Datagram dimension
                in >> dataSize; // Updated only if we are parsing a new block
            }

            /*if( datagramSize < mNextUdpStBlockSize )
            {
                qDebug() << PREFIX << tr("Received incomplete UDP Status Block... waiting for the missing data");
                break;
            }*/

            // Datagram IDX
            quint16 msgIdx;
            in >> msgIdx;

            QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
            //qDebug() << tr("%1 - UDP Status Received msg #%2 by %3:%4").arg(timeStr).arg(msgIdx).arg(addr.toString()).arg(port);

            // Datagram Code
            in >> msgCode;

            switch(msgCode)
            {
            case MSG_FAILED:
            {
                qDebug() << tr("UDP Received msg #%1: MSG_FAILED").arg(msgIdx);
                break;
            }

            case MSG_WRITE_OK:
            {
                quint16 reg;
                quint16 nReg;

                in>>reg;
                in>>nReg;

                //qDebug() << tr("UDP Received msg #%1: MSG_WRITE_OK - StartReg: %2 - Nreg: %3")
                //.arg(msgIdx).arg(reg).arg(nReg);
                break;
            }

            case MSG_READ_REPLY:
            {
                //qDebug() << tr("UDP Received msg #%1: MSG_READ_REPLY").arg(msgIdx);
                processReplyMsg( &in );
                break;
            }

            case MSG_ROBOT_CTRL_OK:
            {
                //qDebug() << tr("UDP Received msg #%1: MSG_ROBOT_CTRL_OK").arg(msgIdx);

                emit robotControlTaken();
                break;
            }

            case MSG_ROBOT_CTRL_KO:
            {
                //qDebug() << tr("UDP Received msg #%1: MSG_ROBOT_CTRL_KO").arg(msgIdx);

                emit robotControlNotTaken();
                break;
            }

            case MSG_ROBOT_CTRL_RELEASED:
            {
                //qDebug() << tr("UDP Received msg #%1: MSG_ROBOT_CTRL_RELEASED").arg(msgIdx);

                emit robotControlReleased();
                break;
            }

            case MSG_RC_NOT_FOUND:
            {
                //qDebug() << tr("UDP Received msg #%1: MSG_RC_NOT_FOUND").arg(msgIdx);

                // TODO: handle this error
                break;
            }

            default:
                qDebug() << tr("UDP Received unknown message (%1) with msg #%2").arg(msgCode).arg(msgIdx);

                char* buf = new char[mUdpStatusSocket->pendingDatagramSize()];
                //in.readRawData( buf, mNextTcpBlockSize );
                in.readRawData( buf, mUdpStatusSocket->pendingDatagramSize() );
                delete [] buf;

                break;
            }

            //mNextUdpStBlockSize = 0;
        }
    }
}

void RoboControllerSDK::processReplyMsg( QDataStream *inStream )
{
    quint16 startAddr;
    quint16 nReg;

    *inStream >> startAddr;
    *inStream >> nReg;

    //qDebug() << tr("WORD: %1 - nReg: %2")
    //            .arg(startAddr).arg(nReg);

    quint16 value;

    if(nReg==1)
    {
        if( startAddr == WORD_STATUSBIT1 ||  startAddr == WORD_STATUSBIT2 )
        {
            *inStream >> value;

            if(startAddr == WORD_STATUSBIT1)
            {
                mBoardStatus.pidEnable = value & FLG_STATUSBI1_PID_EN;
                mBoardStatus.wdEnable = value & FLG_STATUSBI1_COMWATCHDOG;
                mBoardStatus.saveToEeprom = value & FLG_STATUSBI1_EEPROM_SAVE_EN;
                mBoardStatus.accelRampEnable = value & FLG_STATUSBI1_EEPROM_RAMP_EN;

                if( mBoardStatus.pidEnable )
                    mMotorCtrlMode = mcPID;
                else
                    mMotorCtrlMode = mcDirectPWM;

                if( mBoardStatus.wdEnable )
                    mPingTimer.start( mWatchDogTimeMsec ); // Enable Ping Watchdog timer

                mBoardStatusValid = true;
                emit newBoardStatus( mBoardStatus );

            }
            else
            {
                int flagVal = value & FLG_STATUSBI2_EEPROM_ENCODER_POSITION;
                if( flagVal==0 )
                    mRobotConfig.EncoderPosition = Motor;
                else
                    mRobotConfig.EncoderPosition = Wheel;

                flagVal = value & FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY;
                if(flagVal==0)
                    mRobotConfig.MotorEnableLevel = Low;
                else
                    mRobotConfig.MotorEnableLevel = High;

                mReceivedStatus2 = true;

                if( mReceivedRobConfig && mReceivedStatus2 )
                {
                    mReceivedRobConfig = false;
                    mReceivedStatus2 = false;
                    emit newRobotConfiguration( mRobotConfig );
                }
            }
        }
        else if( startAddr == WORD_COMWATCHDOG_TIME )
        {
            *inStream >> value;
            quint64 newWd = (quint64)value;
            mWatchDogTimeMsec = newWd - newWd/10;

            emit newWatchdogTime( newWd );

            //qDebug() << tr( "New Board WatchDog Time: %1 msec").arg(newWd);
        }
        else if( startAddr == WORD_ADDRESS_SLAVE )
        {
            *inStream >> value;
            quint64 boardIdx = value;

            // qDebug() << tr( "Board Idx: %1").arg(boardIdx);
        }
        else
            qDebug() << tr("Address %1 not yet handled with nReg=1").arg(startAddr);
    }
    else if(nReg==3)
    {
        if( ( startAddr == WORD_PID_P_LEFT || startAddr == WORD_PID_P_RIGHT ) )
        {
            MotorPos motorIdx;
            if(startAddr == WORD_PID_P_LEFT)
                motorIdx = motLeft;
            else
                motorIdx = motRight;

            quint16 Kp, Ki, Kd;
            *inStream >> Kp;
            *inStream >> Ki;
            *inStream >> Kd;
            emit newMotorPIDGains(motorIdx, Kp, Ki, Kd );
        }
        else
            qDebug() << tr("Address %1 not yet handled with nReg=3").arg(startAddr);
    }
    else if(nReg==21)
    {
        if(  startAddr == WORD_ROBOT_DIMENSION_WEIGHT ) // Robot Configuration is stored in 21 consequtive registers starting from WORD_ROBOT_DIMENSION_WEIGHT
        {
            updateRobotConfigurationFromDataStream( inStream );

            if( mReceivedRobConfig && mReceivedStatus2 )
            {
                mReceivedRobConfig = false;
                mReceivedStatus2 = false;
                emit newRobotConfiguration( mRobotConfig );
            }
        }
        else
            qDebug() << tr("Address %1 not yet handled with nReg=19").arg(startAddr);

    }
    else
    {
        qDebug() << tr("Now nReg can be only 1, 3 or 19 (received: %1)").arg(nReg);
    }
}

void RoboControllerSDK::updateRobotConfigurationFromDataStream( QDataStream* inStream )
{
    *inStream >> mRobotConfig.Weight;
    *inStream >> mRobotConfig.Width;
    *inStream >> mRobotConfig.Height;
    *inStream >> mRobotConfig.Lenght;

    *inStream >> mRobotConfig.WheelBase;
    *inStream >> mRobotConfig.WheelRadiusLeft;
    *inStream >> mRobotConfig.WheelRadiusRight;
    *inStream >> mRobotConfig.EncoderCprLeft;
    *inStream >> mRobotConfig.EncoderCprRight;
    *inStream >> mRobotConfig.MaxRpmMotorLeft;
    *inStream >> mRobotConfig.MaxRpmMotorRight;
    *inStream >> mRobotConfig.MaxAmpereMotorLeft;
    *inStream >> mRobotConfig.MaxAmpereMotorRight;
    *inStream >> mRobotConfig.MaxTorqueMotorLeft;
    *inStream >> mRobotConfig.MaxTorqueMotorRight;
    *inStream >> mRobotConfig.RatioShaftLeft;
    *inStream >> mRobotConfig.RatioShaftRight;
    *inStream >> mRobotConfig.RatioMotorLeft;
    *inStream >> mRobotConfig.RatioMotorRight;
    *inStream >> mRobotConfig.MaxChargedBatteryLevel;
    *inStream >> mRobotConfig.MinChargedBatteryLevel;

    mReceivedRobConfig = true;
}

void RoboControllerSDK::onTcpError(QAbstractSocket::SocketError err)
{
    if( err==QAbstractSocket::RemoteHostClosedError )
    {
        mTcpConnected = false;
        emit tcpDisconnected();
    }


    mLastTcpErrorMsg = tr("%1 - %2").arg(err).arg(mTcpSocket->errorString());

    qDebug() << tr( "TCP Communication error: %1 - %2").arg(err).arg(mTcpSocket->errorString());
}

void RoboControllerSDK::onUdpStatusError( QAbstractSocket::SocketError err )
{
    mLastUdpErrorMsg = tr("%1 - %2").arg(err).arg(mUdpStatusSocket->errorString());

    qDebug() << tr( "UDP Status Communication error: %1 - %2").arg(err).arg(mUdpStatusSocket->errorString());
}

void RoboControllerSDK::onUdpControlError( QAbstractSocket::SocketError err )
{
    mLastUdpErrorMsg = tr("%1 - %2").arg(err).arg(mUdpControlSocket->errorString());

    qDebug() << tr( "UDP Control Communication error: %1 - %2").arg(err).arg(mUdpControlSocket->errorString());
}

void RoboControllerSDK::onTcpHostFound()
{
    qDebug() << tr( "TCP Host found. Trying to communicate with server.");
}

void RoboControllerSDK::sendBlockUDP( QUdpSocket *socket, QHostAddress addr, quint16 port, quint16 msgCode, QVector<quint16> &data, bool waitReply/*=false*/ )
{
    if(!socket)
        return;

    if(mWatchDogEnable)
        mPingTimer.start( mWatchDogTimeMsec ); // Restart timer to avoid unuseful Ping

    mUdpPingTimer.start(UDP_PING_TIME_MSEC); // Restart timer to avoid unuseful Ping

    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_3);
    out << (quint16)UDP_START_VAL; // Start word
    out << (quint16)0;     // Block size
    out << mMsgCounter;    // Message counter
    out << msgCode;        // Message Code

    ++mMsgCounter;
    //mMsgCounter %= 65536;

    // ---> Data
    QVector<quint16>::iterator it;
    for(it = data.begin(); it != data.end(); ++it)
        out << *it;
    // <--- Data

    out.device()->seek(0);          // Back to the beginning to set block size
    int blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)UDP_START_VAL; // Start word again
    out << (quint16)blockSize;

    //QMutexLocker locker( &mConnMutex );
    mConnMutex.lock();
    {
        //socket->write( block );
        socket->writeDatagram( block, addr, port );
        socket->flush();

        qint64 time = QDateTime::currentDateTime().toMSecsSinceEpoch();
        mLastServerReqTime = time;

        //QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        //qDebug() << tr("%1 - Sent msg #%2 (Code: %3) over UDP").arg(timeStr).arg(mMsgCounter).arg(msgCode);
    }
    mConnMutex.unlock();

    if( waitReply)
    {
        if( !socket->waitForReadyRead( SERVER_REPLY_TIMEOUT_MSEC ) )
        {
            qDebug() << tr("The server does not reply. Communication lost");

            throw RcException( excCommunicationLost, tr("The server does not reply to requests (Timeout: %1 msec). Last error: %2")
                               .arg(SERVER_REPLY_TIMEOUT_MSEC)
                               .arg(socket->errorString() ).toLocal8Bit() );
        }
    }
}

bool RoboControllerSDK::sendBlockTCP(quint16 msgCode, QVector<quint16> &data )
{
    if(!mTcpSocket)
        return false;

    if( mWatchDogEnable )
        mPingTimer.start( mWatchDogTimeMsec ); // Restart timer to avoid unuseful Ping

    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_3);
    out << (quint16)TCP_START_VAL; // Start word
    out << (quint16)0;     // Block size
    out << mMsgCounter;    // Message counter
    out << msgCode;        // Message Code

    ++mMsgCounter;
    mMsgCounter %= 65536;

    // ---> Data
    QVector<quint16>::iterator it;
    for(it = data.begin(); it != data.end(); ++it)
        out << *it;
    // <--- Data

    out.device()->seek(0);          // Back to the beginning to set block size
    int blockSize = (block.size() - 2*sizeof(quint16));
    out << (quint16)TCP_START_VAL; // Start work again
    out << (quint16)blockSize;

    //QMutexLocker locker( &mConnMutex );
    mConnMutex.lock();
    {
        mTcpSocket->write( block );
        mTcpSocket->flush();

        qint64 time = QDateTime::currentDateTime().toMSecsSinceEpoch();
        mLastServerReqTime = time;

        QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        //qDebug() << tr("%1 - Sent msg #%2 (Code: %3) over TCP").arg(timeStr).arg(mMsgCounter).arg(msgCode);
    }
    mConnMutex.unlock();

    if( !mTcpSocket->waitForReadyRead( SERVER_REPLY_TIMEOUT_MSEC ) )
    {
        qDebug() << tr("The server does not reply. Communication lost");

        throw RcException( excCommunicationLost, tr("The server does not reply to requests (Timeout: %1 msec). Last error: %2")
                           .arg(SERVER_REPLY_TIMEOUT_MSEC)
                           .arg(mTcpSocket->errorString() ).toLocal8Bit() );

        return false;
    }

    return true;
}

/// Disables the Communication Watchdog
/*void RoboControllerSDK::disableWatchdog()
{
    // >>>>> WatchDog disable
    mNewStatusBit1Received = false;
    mWatchDogEnable = false;

    QVector<quint16> data;
    data << (quint16)WORD_STATUSBIT1;
    data << 1; // Only one register

    sendCommand( CMD_RD_MULTI_REG, data );

    int count = 0;
    while(!mNewStatusBit1Received && count < 50 )
    {
        count++;
        msleep(100);
    }

    if(mWatchDogEnable)
    {
        // WatchDog Disabling
        mStatusBits1 &= ~(0x0002);

        QVector<quint16> data;
        data << (quint16)WORD_STATUSBIT1;
        //data << (quint16)nReg; <- Not requested by sendCommand!!!

        data << mStatusBits1;

        sendCommand( CMD_WR_MULTI_REG, data );
    }
    // <<<<< WatchDog disable
}*/

/// Enables the Communication Watchdog
/*void RoboControllerSDK::enableWatchdog()
{
    // >>>>> WatchDog disable
    mNewStatusBit1Received = false;
    mWatchDogEnable = false;

    QVector<quint16> data;
    data << (quint16)WORD_STATUSBIT1;
    data << 1; // Only one register

    sendCommand( CMD_RD_MULTI_REG, data );

    int count = 0;
    while( !mNewStatusBit1Received && count < 50 )
    {
        count++;
        QThread::msleep(100);
    }

    if(mWatchDogEnable)
    {
        // WatchDog Enabling
        mStatusBits1 |= FLG_STATUSBI1_COMWATCHDOG;

        QVector<quint16> data;
        data << (quint16)WORD_STATUSBIT1;
        //data << (quint16)nReg; <- Not requested by sendCommand!!!

        data << mStatusBits1;

        sendCommand( CMD_WR_MULTI_REG, data );
    }
    // <<<<< WatchDog disable
}*/

void RoboControllerSDK::run()
{
    qDebug() << tr("RoboControllerSDK thread started");

    /*/ Update Board Status
    getBoardStatus();

    msleep( 300 );

    // Update Robot Configuration
    getRobotConfigurationFromEeprom();

    msleep( 300 ); */

    exec();

    qDebug() << tr("RoboControllerSDK thread finished");
}

void RoboControllerSDK::getBoardStatus()
{
    QVector<quint16> data;
    data << (quint16)WORD_STATUSBIT1;
    data << 1;

    mBoardStatusValid = false;

    sendBlockTCP( CMD_RD_MULTI_REG, data );
}

bool RoboControllerSDK::setBoardStatus( BoardStatus &status )
{
    quint16 statusVal = 0x0000;
    if(status.accelRampEnable)
        statusVal |= FLG_STATUSBI1_EEPROM_RAMP_EN;
    if(status.pidEnable)
    {
        mMotorCtrlMode = mcPID;
        statusVal |= FLG_STATUSBI1_PID_EN;
    }
    else
        mMotorCtrlMode = mcDirectPWM;
    if(status.saveToEeprom)
        statusVal |= FLG_STATUSBI1_EEPROM_SAVE_EN;
    if(status.wdEnable)
        statusVal |= FLG_STATUSBI1_COMWATCHDOG;

    QVector<quint16> data;
    data << (quint16)WORD_STATUSBIT1;
    data << statusVal;

    bool ok = sendBlockTCP( CMD_WR_MULTI_REG, data );

    if( ok )
    {
        mBoardStatus = status;
        mBoardStatusValid = true;
    }
    else
        mBoardStatusValid = false;

    return ok;
}

void RoboControllerSDK::getMotorPidGains(MotorPos motorIdx )
{
    QVector<quint16> data;
    if(motorIdx==motLeft)
        data << (quint16)WORD_PID_P_LEFT;
    else
        data << (quint16)WORD_PID_P_RIGHT;
    data << 3; // 3 consecutive registers!

    sendBlockTCP( CMD_RD_MULTI_REG, data );
}

void RoboControllerSDK::setMotorPWM(MotorPos motorIdx, int pwm )
{
    if( mMotorCtrlMode != mcDirectPWM )
    {
        qWarning() << PREFIX << tr("Function available only in mcDirectPWM mode");
    }

    // >>>>> Saturation
    if( pwm > 2047)
        pwm = 2047;

    if( pwm < -2048 )
        pwm = -2048;
    // <<<<< Saturation

    // >>>>> New PWM to RoboController
    quint16 address;
    if(motorIdx == motLeft)
        address = WORD_PWM_CH1;
    else
        address = WORD_PWM_CH2;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!
    data << pwm;

    //sendBlockTCP( mUdpControlSocket, CMD_WR_MULTI_REG, data );
    sendBlockUDP( mUdpControlSocket, QHostAddress(mServerAddr), mUdpControlPortSend, CMD_WR_MULTI_REG, data, false );
    // <<<<< New PWM to RoboController
}

void RoboControllerSDK::setMotorPWMs( quint16 pwmMotor0, quint16 pwmMotor1 )
{
    if( mMotorCtrlMode != mcDirectPWM )
    {
        qWarning() << PREFIX << tr("Function available only in mcDirectPWM mode");
    }

    // >>>>> Saturation
    if( pwmMotor0 > 2047)
        pwmMotor0 = 2047;

    if( pwmMotor0 < -2048 )
        pwmMotor0 = -2048;

    if( pwmMotor1 > 2047)
        pwmMotor1 = 2047;

    if( pwmMotor1 < -2048 )
        pwmMotor1 = -2048;
    // <<<<< Saturation

    // >>>>> New PWM to RoboController
    quint16 address = WORD_PWM_CH1;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!
    data << pwmMotor0;
    data << pwmMotor1;

    //sendBlockTCP( mUdpControlSocket, CMD_WR_MULTI_REG, data );
    sendBlockUDP( mUdpControlSocket, QHostAddress(mServerAddr), mUdpControlPortSend, CMD_WR_MULTI_REG, data, false );
    // <<<<< New PWM to RoboController
}

void RoboControllerSDK::setMotorSpeeds( double speed0, double speed1 )
{
    if( mMotorCtrlMode != mcPID )
    {
        qWarning() << PREFIX << tr("Function available only in mcPID mode");
        return;
    }

    // >>>>> 16 bit saturation
    if( speed0 > 32.767)
        speed0 = 32.767;

    if( speed0 < -32.768 )
        speed0 = -32.768;

    if( speed1 > 32.767)
        speed1 = 32.767;

    if( speed1 < -32.768 )
        speed1 = -32.768;
    // <<<<< 16 bit saturation

    // >>>>> New SetPoint to RoboController
    quint16 address = WORD_PWM_CH1;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!

    quint16 sp; // Speed is integer 2-complement!
    if(speed0 >= 0)
        sp = (quint16)(speed0*1000.0);
    else
        sp = (quint16)(speed0*1000.0+65536.0);

    //quint16 sp = (quint16)(speed*1000.0+32767.5);
    data << sp;

    if(speed1 >= 0)
        sp = (quint16)(speed1*1000.0);
    else
        sp = (quint16)(speed1*1000.0+65536.0);

    //quint16 sp = (quint16)(speed*1000.0+32767.5);
    data << sp;

    sendBlockUDP( mUdpControlSocket, QHostAddress(mServerAddr), mUdpControlPortSend, CMD_WR_MULTI_REG, data, false );
    // <<<<< New SetPoint to RoboController
}

void RoboControllerSDK::setMotorSpeed(MotorPos motorIdx, double speed )
{
    if( mMotorCtrlMode != mcPID )
    {
        qWarning() << PREFIX << tr("Function available only in mcPID mode");
        return;
    }

    // >>>>> 16 bit saturation
    if( speed > 32.767)
        speed = 32.767;

    if( speed < -32.768 )
        speed = -32.768;
    // <<<<< 16 bit saturation

    // >>>>> New SetPoint to RoboController
    quint16 address;
    if(motorIdx == motLeft)
        address = WORD_PWM_CH1;
    else
        address = WORD_PWM_CH2;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!

    quint16 sp; // Speed is integer 2-complement!
    if(speed >= 0)
        sp = (quint16)(speed*1000.0);
    else
        sp = (quint16)(speed*1000.0+65536.0);

    //quint16 sp = (quint16)(speed*1000.0+32767.5);
    data << sp;

    sendBlockUDP( mUdpControlSocket, QHostAddress(mServerAddr), mUdpControlPortSend, CMD_WR_MULTI_REG, data, false );
    // <<<<< New SetPoint to RoboController
}

void RoboControllerSDK::setMotorPidGains(MotorPos motorIdx, quint16 Kp, quint16 Ki, quint16 Kd )
{
    // >>>>> New SetPoint to RoboController
    quint16 address;
    if(motorIdx == motLeft)
        address = WORD_PID_P_LEFT;
    else
        address = WORD_PID_P_RIGHT;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!

    data << Kp;
    data << Ki;
    data << Kd;

    sendBlockTCP( CMD_WR_MULTI_REG, data );
    // <<<<< New SetPoint to RoboController
}

void RoboControllerSDK::onPingTimerTimeout()
{
    QVector<quint16> pingData;
    pingData << (quint16)WORD_COMWATCHDOG_TIME;
    pingData << 1; // Only one register

    quint64 elapsed = QDateTime::currentDateTime().toMSecsSinceEpoch() - mLastServerReqTime;

    //qDebug() << tr(" Ping RC WatchDog - Elapsed: %1").arg(elapsed);

    // TODO Move this request to UDP?
    sendBlockTCP( CMD_RD_MULTI_REG, pingData );
}

bool RoboControllerSDK::getRobotConfigurationFromIni( QString iniFile )
{
    QFile file( iniFile );

    if( !file.exists() )
    {
        qDebug() << tr("File ini %1 does not exist.").arg(iniFile);
        return false;
    }

    QSettings ini(iniFile, QSettings::IniFormat);

    ini.beginGroup( "Structure");
    {
        // MyzharBot 2013-05-28
        mRobotConfig.Weight = ini.value( "Weight(g)", 4000 ).toInt();
        mRobotConfig.Width = ini.value( "Width(mm)", 420 ).toInt();
        mRobotConfig.Height = ini.value( "Height(mm)", 165 ).toInt();
        mRobotConfig.Lenght = ini.value( "Lenght(mm)", 365 ).toInt();
    }
    ini.endGroup();
    ini.beginGroup( "Wheels");
    {
        // MyzharBot 2013-05-28
        mRobotConfig.WheelBase = ini.value( "WheelBase(mm)", 340 ).toInt();
        mRobotConfig.WheelRadiusLeft = ini.value( "WheelRadiusLeft(cent_mm)", 3500 ).toInt();
        mRobotConfig.WheelRadiusRight = ini.value( "WheelRadiusRight(cent_mm)", 3500 ).toInt();
    }
    ini.endGroup();
    ini.beginGroup( "Encoders");
    {
        // MyzharBot 2013-05-28
        mRobotConfig.EncoderCprLeft = ini.value( "EncoderCprLeft", 400 ).toInt();
        mRobotConfig.EncoderCprRight = ini.value( "EncoderCprRight", 400 ).toInt();
        mRobotConfig.EncoderPosition = static_cast<EncoderPos>(ini.value( "EncoderPosition", 1 ).toInt());
    }
    ini.endGroup();
    ini.beginGroup( "Motors");
    {
        // Default values for MicroMotors E192-12-18 "http://www.micromotorssrl.com/motor_e192/e192_pg.html"
        mRobotConfig.MaxRpmMotorLeft = ini.value( "MaxRpmMotorLeft", 218 ).toInt();
        mRobotConfig.MaxRpmMotorRight = ini.value( "MaxRpmMotorRight", 218 ).toInt();
        mRobotConfig.MaxAmpereMotorLeft = ini.value( "MaxAmpereMotorLeft(mA)", 1650 ).toInt();
        mRobotConfig.MaxAmpereMotorRight = ini.value( "MaxAmpereMotorRight", 1650 ).toInt();
        mRobotConfig.MaxTorqueMotorLeft = ini.value( "MaxTorqueMotorLeft(Ncm)", 60 ).toInt();
        mRobotConfig.MaxTorqueMotorRight = ini.value( "MaxTorqueMotorRight(Ncm)", 60 ).toInt();
        mRobotConfig.RatioShaftLeft = ini.value( "RatioShaftLeft", 3 ).toInt();
        mRobotConfig.RatioShaftRight = ini.value( "RatioShaftRight", 3 ).toInt();
        mRobotConfig.RatioMotorLeft = ini.value( "RatioMotorLeft", 55 ).toInt();
        mRobotConfig.RatioMotorRight = ini.value( "RatioMotorRight", 55 ).toInt();
        mRobotConfig.MotorEnableLevel = static_cast<PinLevel>(ini.value( "MotorEnableLevel", 1 ).toInt());
    }
    ini.endGroup();
    ini.beginGroup( "Battery");
    {
        mRobotConfig.MaxChargedBatteryLevel = ini.value( "MaxChargedBatteryLevel", 16800 ).toInt();
        mRobotConfig.MinChargedBatteryLevel = ini.value( "MinChargedBatteryLevel", 12000 ).toInt();
    }
    ini.endGroup();
    emit newRobotConfiguration( mRobotConfig );
    return true;
}

void RoboControllerSDK::getRobotConfigurationFromEeprom( )
{
    mReceivedStatus2 = false;
    mReceivedRobConfig = false;

    // >>>>> Robot Configuration Data
    QVector<quint16> data;
    data << (quint16)WORD_ROBOT_DIMENSION_WEIGHT;
    data << 21; // 21 Registers!!!

    sendBlockTCP( CMD_RD_MULTI_REG, data );
    // <<<<< Robot Configuration Data

    msleep( 250 );

    // >>>>> Robot Configuration Bits
    data.clear();
    data << (quint16)WORD_STATUSBIT2;
    data << 1;

    sendBlockTCP( CMD_RD_MULTI_REG, data );
    // <<<<< Robot Configuration Bits
}

void RoboControllerSDK::saveRobotConfigurationToIni( QString iniFile )
{
    QSettings ini( iniFile, QSettings::IniFormat );
    ini.sync();

    ini.beginGroup( "Structure");
    {
        ini.setValue( "Weight_g", mRobotConfig.Weight );
        ini.setValue( "Width_mm", mRobotConfig.Width );
        ini.setValue( "Height_mm", mRobotConfig.Height );
        ini.setValue( "Lenght_mm", mRobotConfig.Lenght );
        ini.setValue( "WheelBase_mm", mRobotConfig.WheelBase );
    }
    ini.endGroup();
    ini.beginGroup( "Wheels");
    {
        ini.setValue( "WheelRadiusLeft_cent_mm", mRobotConfig.WheelRadiusLeft );
        ini.setValue( "WheelRadiusRight_cent_mm", mRobotConfig.WheelRadiusRight );
    }
    ini.endGroup();
    ini.beginGroup( "Encoders");
    {
        ini.setValue( "EncoderCprLeft", mRobotConfig.EncoderCprLeft );
        ini.setValue( "EncoderCprRight", mRobotConfig.EncoderCprRight );
        ini.setValue( "EncoderPosition", static_cast<int>(mRobotConfig.EncoderPosition) );
    }
    ini.endGroup();
    ini.beginGroup( "Motors");
    {
        ini.setValue( "MaxRpmMotorLeft", mRobotConfig.MaxRpmMotorLeft );
        ini.setValue( "MaxRpmMotorRight", mRobotConfig.MaxRpmMotorRight );
        ini.setValue( "MaxAmpereMotorLeft_mA", mRobotConfig.MaxAmpereMotorLeft );
        ini.setValue( "MaxAmpereMotorRight", mRobotConfig.MaxAmpereMotorRight );
        ini.setValue( "MaxTorqueMotorLeft_Ncm", mRobotConfig.MaxTorqueMotorLeft );
        ini.setValue( "MaxTorqueMotorRight_Ncm", mRobotConfig.MaxTorqueMotorRight );
        ini.setValue( "RatioShaftLeft", mRobotConfig.RatioShaftLeft );
        ini.setValue( "RatioShaftRight", mRobotConfig.RatioShaftRight );
        ini.setValue( "RatioMotorLeft", mRobotConfig.RatioMotorLeft );
        ini.setValue( "RatioMotorRight", mRobotConfig.RatioMotorRight );
        ini.setValue( "MotorEnableLevel", static_cast<int>(mRobotConfig.MotorEnableLevel) );
    }
    ini.endGroup();
    ini.beginGroup( "Battery");
    {
        ini.setValue( "MaxChargedBatteryLevel", mRobotConfig.MaxChargedBatteryLevel );
        ini.setValue( "MinChargedBatteryLevel", mRobotConfig.MinChargedBatteryLevel );
    }
    ini.endGroup();
    ini.sync();
}

void RoboControllerSDK::setRobotConfiguration( RobotConfiguration& roboConfig )
{
    memcpy( &mRobotConfig, &roboConfig, sizeof(RobotConfiguration) );
}

void RoboControllerSDK::getRobotControl()
{
    if(!mUdpStatusSocket)
        return;

    QVector<quint16> vec;
    sendBlockUDP( mUdpControlSocket, QHostAddress(mServerAddr), mUdpControlPortSend, CMD_GET_ROBOT_CTRL, vec, false );
}

void RoboControllerSDK::releaseRobotControl()
{
    if(!mUdpStatusSocket)
        return;

    if( mMotorCtrlMode == mcPID )
        setMotorSpeeds( 0.0, 0.0 );
    else
        setMotorPWMs( 0, 0 );


    QVector<quint16> vec;
    sendBlockUDP( mUdpControlSocket, QHostAddress(mServerAddr), mUdpControlPortSend, CMD_REL_ROBOT_CTRL, vec, false );
}

void RoboControllerSDK::setBatteryCalibrationParams( AnalogCalibValue valueType, double curChargeVal)
{
    quint16 charVal = (quint16)(curChargeVal*1000.0);

    QVector<quint16> data;
    data << (quint16)WORD_VAL_TAR_FS;
    data << charVal;
    sendBlockTCP( CMD_WR_MULTI_REG, data );

    msleep(500);

    data.clear();
    quint16 flag = (valueType==CalLow)?0x00001:0x0020;
    data << (quint16)WORD_FLAG_TARATURA;
    data << flag;
    sendBlockTCP( CMD_WR_MULTI_REG, data );

    msleep(500);
}

void RoboControllerSDK::enableWatchdog( quint16 timeOut_msec )
{
    mPingTimer.stop();

    mWatchDogTimeMsec = timeOut_msec;

    if( !mBoardStatusValid )
    {
        mBoardStatus.pidEnable = true;
        mBoardStatus.accelRampEnable = true;
        mBoardStatus.saveToEeprom = true;
    }

    mBoardStatus.wdEnable = true;

    setBoardStatus( mBoardStatus );

    msleep( 100 );

    QVector<quint16> data;
    data << (quint16)WORD_COMWATCHDOG_TIME;
    data << timeOut_msec;
    if( sendBlockTCP( CMD_WR_MULTI_REG, data ) )
    {
        msleep( 100 );

        // Update board status from EEPROM
        getBoardStatus();
    }
}

void RoboControllerSDK::getWatchdogTime( )
{
    QVector<quint16> data;
    data << (quint16)WORD_COMWATCHDOG_TIME;
    data << 1;

    if( sendBlockTCP( CMD_RD_MULTI_REG, data ) )
        qDebug() << tr("Motors wathdog has been enabled");
}

qint64 RoboControllerSDK::getLastTelemetry( RobotTelemetry& telemetry )
{
    memcpy( &telemetry, &mTelemetry, sizeof(RobotTelemetry) );

    return mLastTelemetryTimestamp;
}

void RoboControllerSDK::disableWatchdog( )
{
    mPingTimer.stop();

    if( !mBoardStatusValid )
    {
        mBoardStatus.pidEnable = true;
        mBoardStatus.accelRampEnable = true;
        mBoardStatus.saveToEeprom = true;
    }
    mBoardStatus.wdEnable = false;

    if( setBoardStatus( mBoardStatus ) )
    {
        mWatchDogEnable = false;
        qDebug() << tr("Motors wathdog has been disabled");
    }
    else
        qDebug() << tr("Warning: Motors wathdog has not been disabled");
}

void RoboControllerSDK::saveRobotConfigurationToEeprom( )
{
    // >>>>> Robot Configuration Data (19 consequtive registers)
    QVector<quint16> data;
    data << (quint16)WORD_ROBOT_DIMENSION_WEIGHT;
    // data << 19; <- Not requested by sendCommand!!!

    data << mRobotConfig.Weight;
    data << mRobotConfig.Width;
    data << mRobotConfig.Height;
    data << mRobotConfig.Lenght;

    data << mRobotConfig.WheelBase;
    data << mRobotConfig.WheelRadiusLeft;
    data << mRobotConfig.WheelRadiusRight;
    data << mRobotConfig.EncoderCprLeft;
    data << mRobotConfig.EncoderCprRight;
    data << mRobotConfig.MaxRpmMotorLeft;
    data << mRobotConfig.MaxRpmMotorRight;
    data << mRobotConfig.MaxAmpereMotorLeft;
    data << mRobotConfig.MaxAmpereMotorRight;
    data << mRobotConfig.MaxTorqueMotorLeft;
    data << mRobotConfig.MaxTorqueMotorRight;
    data << mRobotConfig.RatioShaftLeft;
    data << mRobotConfig.RatioShaftRight;
    data << mRobotConfig.RatioMotorLeft;
    data << mRobotConfig.RatioMotorRight;

    sendBlockTCP( CMD_WR_MULTI_REG, data );
    // <<<<< Robot Configuration Data (19 consequtive registers)

    // >>>>> Status Register 2
    data.clear();
    data << (quint16)WORD_STATUSBIT2;

    quint16 statusVal = 0;
    if(mRobotConfig.EncoderPosition)
        statusVal |= FLG_STATUSBI2_EEPROM_ENCODER_POSITION;
    if(mRobotConfig.MotorEnableLevel)
        statusVal |= FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY;

    data << statusVal;

    sendBlockTCP( CMD_WR_MULTI_REG, data );
    // <<<<< Status Register 2
}

void RoboControllerSDK::onUdpTestTimerTimeout()
{
    QVector<quint16> pingData;
    pingData << (quint16)WORD_ADDRESS_SLAVE;
    pingData << 1; // Only one register

    // qDebug() << tr("Ping UDP");

    sendBlockUDP( mUdpStatusSocket, QHostAddress(mServerAddr), mUdpStatusPortSend, CMD_RD_MULTI_REG, pingData, true );
}

}
