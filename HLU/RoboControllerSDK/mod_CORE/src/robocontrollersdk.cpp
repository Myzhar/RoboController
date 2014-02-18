#include <robocontrollersdk.h>
#include <exception.h>
#include <QDataStream>
#include <QCoreApplication>

#include "network_msg.h"
#include "modbus_registers.h"
#include <QDateTime>
#include <QSettings>
#include <QFile>

namespace roboctrl
{

RoboControllerSDK::RoboControllerSDK(int udpStatusPort,
                                     int udpControlPort,
                                     QString serverAddr,
                                     int tcpPort ) :
    mTcpSocket(NULL),
    mUdpStatusSocket(NULL),
    mUdpControlSocket(NULL)
{
    mStopped = true;
    mWatchDogTimeMsec = 1000;
    mMsgCounter = 0;

    // Ping Timer
    connect( &mPingTimer, SIGNAL(timeout()), this, SLOT(onPingTimerTimeout()));

    // >>>>> TCP Socket
    mTcpSocket = new QTcpSocket(this);

    mServerAddr = serverAddr;
    mTcpPort = tcpPort;

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(onTcpReadyRead()));
    connect(mTcpSocket, SIGNAL(hostFound()), this, SLOT(onTcpHostFound()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(onTcpError(QAbstractSocket::SocketError)));

    connectToTcpServer();
    // <<<<< TCP Socket

    // >>>>> UDP Sockets
    // TODO UDP Socket Connection
    mUdpControlSocket = new QUdpSocket();
    mUdpStatusSocket = new QUdpSocket();

    // <<<<< UDP Sockets
    mMotorCtrlMode = mcPID; // RoboController is in PID mode by default

    // Start thread
    mStopped = false;
    start();
}

RoboControllerSDK::~RoboControllerSDK()
{
    this->terminate();

    while(this->isRunning());
    disconnectTcpServer();
    disconnectUdpServers();
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
    mUdpControlSocket->connectToHost( QHostAddress(mServerAddr), mUdpControlPort );
    if( !mUdpControlSocket->waitForConnected( 5000 ) )
    {
        throw RcException( excUdpNotConnected, tr("It is not possible to connect to UDP server: %1")
                           .arg(mUdpControlSocket->errorString() ).toLocal8Bit() );
    }

    int count=0;
    while( count < 5 && !mUdpConnected )
    {
        sleep(1);
        QCoreApplication::processEvents();
        count ++;
    }

    if( !mUdpConnected )
        throw RcException( excUdpNotConnected, tr("It is not possible to find a valid TCP server: %1")
                           .arg(mUdpControlSocket->errorString() ).toLocal8Bit() );

    mUdpConnected = false;
    mUdpStatusSocket->connectToHost( QHostAddress(mServerAddr), mUdpStatusPort );
    if( !mUdpStatusSocket->waitForConnected( 5000 ) )
    {
        throw RcException( excUdpNotConnected, tr("It is not possible to connect to UDP server: %1")
                           .arg(mUdpStatusSocket->errorString() ).toLocal8Bit() );
    }

    count=0;
    while( count < 5 && !mUdpConnected )
    {
        sleep(1);
        QCoreApplication::processEvents();
        count ++;
    }

    if( !mUdpConnected )
        throw RcException( excUdpNotConnected, tr("It is not possible to find a valid TCP server: %1")
                           .arg(mUdpControlSocket->errorString() ).toLocal8Bit() );
}

void RoboControllerSDK::disconnectUdpServers()
{
    if(mUdpControlSocket)
    {
        mUdpControlSocket->close();
        if( mUdpControlSocket->state() == QAbstractSocket::UnconnectedState ||
                mUdpControlSocket->waitForDisconnected(1000) )
            qDebug() << tr("UDP Control Socket Disconnected");
    }

    if(mUdpStatusSocket)
    {
        mUdpStatusSocket->close();
        if( mUdpStatusSocket->state() == QAbstractSocket::UnconnectedState ||
                mUdpStatusSocket->waitForDisconnected(1000) )
            qDebug() << tr("UDP Status Socket Disconnected");
    }
}

void RoboControllerSDK::onTcpReadyRead()
{
    QDataStream in(mTcpSocket);
    in.setVersion(QDataStream::Qt_4_0);

    mNextTcpBlockSize=0;
    quint16 msgCode;

    forever // Receiving data while there is data available
    {
        if( mNextTcpBlockSize==0) // No incomplete blocks received before
        {
            if (mTcpSocket->bytesAvailable() < (qint64)sizeof(quint16))
            {
                qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
                break;
            }

            // Datagram dimension
            in >> mNextTcpBlockSize; // Updated only if we are parsing a new block
        }

        if (mTcpSocket->bytesAvailable() < mNextTcpBlockSize)
        {
            qDebug() << Q_FUNC_INFO << tr("Received incomplete TCP Block... waiting for the missing data");
            break;
        }

        // Datagram IDX
        quint16 msgIdx;
        in >> msgIdx;

        QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        qDebug() << tr("%1 - Received msg #%2").arg(timeStr).arg(msgIdx);

        // Datagram Code
        in >> msgCode;

        switch(msgCode)
        {
        case MSG_CONNECTED:
        {
            if( !mTcpConnected )
            {
                mTcpConnected = true;
                qDebug() << tr("Received msg #%1: MSG_CONNECTED").arg(msgIdx);
                qDebug() << tr( "Server ready: %1").arg(mTcpSocket->localAddress().toString() );

                in >> mBoardIdx;

                emit tcpConnected();
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

        case MSG_OK:
        {
            qDebug() << tr("Received msg #%1: MSG_OK").arg(msgIdx);
            break;
        }

        case MSG_FAILED:
        {
            qDebug() << tr("Received msg #%1: MSG_FAILED").arg(msgIdx);
            break;
        }

        case MSG_WRITE_OK:
        {
            qDebug() << tr("Received msg #%1: MSG_WRITE_OK").arg(msgIdx);
            break;
        }

        case MSG_READ_REPLY:
        {
            qDebug() << tr("Received msg #%1: MSG_READ_REPLY").arg(msgIdx);
            processReplyMsg( &in );
            break;
        }

        default:
            qDebug() << tr("Received unknown message (%1) with msg #%2").arg(msgCode).arg(msgIdx);

            in.readRawData( NULL, mNextTcpBlockSize );
            break;
        }

        mNextTcpBlockSize = 0;
    }
}

void RoboControllerSDK::processReplyMsg( QDataStream *inStream )
{
    quint16 startAddr;
    quint16 nReg;

    *inStream >> startAddr;
    *inStream >> nReg;

    qDebug() << tr("WORD: %1 - nReg: %2")
                .arg(startAddr).arg(nReg);

    quint16 value;

    if(nReg==1)
    {
        if( ( startAddr == WORD_PWM_CH1 || startAddr == WORD_PWM_CH2 ) )
        {
            quint16 motorIdx;
            if(startAddr == WORD_PWM_CH1)
                motorIdx = 0;
            else
                motorIdx = 1;

            *inStream >> value;
            emit newMotorPwmValue( motorIdx, value );
        }
        else if( ( startAddr == WORD_RD_PWM_CH1 || startAddr == WORD_RD_PWM_CH2 ) )
        {
            quint16 motorIdx;
            if(startAddr == WORD_RD_PWM_CH1)
                motorIdx = 0;
            else
                motorIdx = 1;

            *inStream >> value;

            emit newMotorPwmValue( motorIdx, value );
        }
        else if( ( startAddr == WORD_ENC1_SPEED || startAddr == WORD_ENC2_SPEED ) )
        {
            quint16 motorIdx;
            if(startAddr == WORD_ENC1_SPEED)
                motorIdx = 0;
            else
                motorIdx = 1;

            *inStream >> value;

            /*double speed = ((double)value-32768)/1000.0;*/
            double speed;
            if(value < 32768)  // Speed is integer 2-complement!
                speed = ((double)value)/1000.0;
            else
                speed = ((double)(value-65536))/1000.0;

            emit newMotorSpeedValue( motorIdx, speed );
        }
        else if( startAddr == WORD_STATUSBIT1 ||  startAddr == WORD_STATUSBIT2 )
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

            qDebug() << tr( "New Board WatchDog Time: %1 msec").arg(mWatchDogTimeMsec);
        }
        else
            qDebug() << tr("Address %1 not yet handled with nReg=1").arg(startAddr);
    }
    else if(nReg==3)
    {
        if( ( startAddr == WORD_PID_P_LEFT || startAddr == WORD_PID_P_RIGHT ) )
        {
            quint16 motorIdx;
            if(startAddr == WORD_PID_P_LEFT)
                motorIdx = 0;
            else
                motorIdx = 1;

            quint16 Kp, Ki, Kd;
            *inStream >> Kp;
            *inStream >> Ki;
            *inStream >> Kd;
            emit newMotorPIDGains(motorIdx, Kp, Ki, Kd );
        }
        else
            qDebug() << tr("Address %1 not yet handled with nReg=3").arg(startAddr);
    }
    else if(nReg==19)
    {
        if(  startAddr == WORD_ROBOT_DIMENSION_WEIGHT ) // Robot Configuration is stored in 19 consegutive register starting from WORD_ROBOT_DIMENSION_WEIGHT
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

    mReceivedRobConfig = true;
}

void RoboControllerSDK::onTcpError(QAbstractSocket::SocketError err)
{
    if( err==QAbstractSocket::RemoteHostClosedError )
    {
        mTcpConnected = false;
        emit tcpDisconnected();
    }

    /*if( err==QAbstractSocket::ConnectionRefusedError )
                    {
                        mTcpConnected = false;
                        emit tcpDisconnected();

                        throw RcException( extTcpConnectionRefused, tr("TCP Server error: %1")
                                           .arg(mTcpSocket->errorString() ).toLocal8Bit() );

                    }*/

    mLastTcpErrorMsg = tr("%1 - %2").arg(err).arg(mTcpSocket->errorString());

    qDebug() << tr( "TCP Communication error: %1 - %2").arg(err).arg(mTcpSocket->errorString());
}

void RoboControllerSDK::onTcpHostFound()
{
    qDebug() << tr( "TCP Host found. Trying to communicate with server.");
}

void RoboControllerSDK::sendCommand( QAbstractSocket* socket, quint16 msgCode, QVector<quint16> &data )
{
    if(!socket)
        return;

    mPingTimer.start( mWatchDogTimeMsec ); // Restart timer to avoid unuseful Ping

    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_4_0);
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
    int blockSize = (block.size() - sizeof(quint16));
    out << (quint16)blockSize;

    //QMutexLocker locker( &mConnMutex );
    mConnMutex.lock();
    {
        socket->write( block );
        socket->flush();

        qint64 time = QDateTime::currentDateTime().toMSecsSinceEpoch();
        mLastServerReqTime = time;

        QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        qDebug() << tr("%1 - Sent msg #%2").arg(timeStr).arg(mMsgCounter);
    }
    mConnMutex.unlock();

    if( !socket->waitForReadyRead( SERVER_REPLY_TIMEOUT_MSEC ) )
    {
        qDebug() << tr("The server does not reply. Communication lost");

        throw RcException( excCommunicationLost, tr("The server does not reply to requests (Timeout: %1 msec). Last error: %2")
                           .arg(SERVER_REPLY_TIMEOUT_MSEC)
                           .arg(socket->errorString() ).toLocal8Bit() );
    }
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

    mPingTimer.setTimerType( Qt::PreciseTimer );
    mPingTimer.start( mWatchDogTimeMsec );

    exec();

    qDebug() << tr("RoboControllerSDK thread finished");
}

//void RoboControllerSDK::commThread()
//{
//    QVector<quint16> pingData;
//    pingData << (quint16)WORD_COMWATCHDOG_TIME;
//    pingData << 1; // Only one register

//    forever
//    {
//        QMutexLocker mlock( &mConnMutex ) ;
//        if(mStopped)
//            return;
//        mlock.unlock();

//        QApplication::processEvents();

//        quint64 elapsed = QDateTime::currentDateTime().toMSecsSinceEpoch() - mLastServerReqTime;
//        if(elapsed > mWatchDogTimeMsec)
//        {
//            qDebug() << tr(" Ping WatchDog - Elapsed: %1").arg(elapsed);
//            sendCommand( CMD_RD_MULTI_REG, pingData );
//        }
//        else
//            msleep( mWatchDogTimeMsec-elapsed );
//    }
//}

void RoboControllerSDK::getMotorPWM( quint16 motorIdx )
{
    QVector<quint16> data;
    if(motorIdx==0)
        data << (quint16)WORD_RD_PWM_CH1;
    else
        data << (quint16)WORD_RD_PWM_CH2;
    data << 1; // Only one register

    sendCommand( mUdpStatusSocket, CMD_RD_MULTI_REG, data );
}

void RoboControllerSDK::getMotorSpeed( quint16 motorIdx )
{
    QVector<quint16> data;
    if(motorIdx==0)
        data << (quint16)WORD_ENC1_SPEED;
    else
        data << (quint16)WORD_ENC2_SPEED;
    data << 1; // Only one register

    sendCommand( mUdpStatusSocket, CMD_RD_MULTI_REG, data );
}

void RoboControllerSDK::getBoardStatus()
{
    QVector<quint16> data;
    data << (quint16)WORD_STATUSBIT1;
    data << 1;

    sendCommand( mTcpSocket, CMD_RD_MULTI_REG, data );
}

void RoboControllerSDK::setBoardStatus( BoardStatus &status )
{
    quint16 statusVal = 0;
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

    sendCommand( mTcpSocket, CMD_WR_MULTI_REG, data );
}

void RoboControllerSDK::getMotorPidGains( quint16 motorIdx )
{
    QVector<quint16> data;
    if(motorIdx==0)
        data << (quint16)WORD_PID_P_LEFT;
    else
        data << (quint16)WORD_PID_P_RIGHT;
    data << 3; // 3 consecutive registers!

    sendCommand( mTcpSocket, CMD_RD_MULTI_REG, data );
}

void RoboControllerSDK::setMotorPWM(quint16 motorIdx, int pwm )
{
    if( mMotorCtrlMode != mcDirectPWM )
    {
        qWarning() << Q_FUNC_INFO << tr("Function available only in mcDirectPWM mode");
    }

    // >>>>> Saturation
    if( pwm > 2047)
        pwm = 2047;

    if( pwm < -2048 )
        pwm = -2048;
    // <<<<< Saturation

    // >>>>> New PWM to RoboController
    quint16 address;
    if(motorIdx == 0)
        address = WORD_PWM_CH1;
    else
        address = WORD_PWM_CH2;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!

    data << pwm;

    sendCommand( mUdpControlSocket, CMD_WR_MULTI_REG, data );
    // <<<<< New PWM to RoboController
}

void RoboControllerSDK::setMotorSpeed( quint16 motorIdx, double speed )
{
    if( mMotorCtrlMode != mcPID )
    {
        qWarning() << Q_FUNC_INFO << tr("Function available only in mcPID mode");
    }

    // >>>>> 16 bit saturation
    if( speed > 32.767)
        speed = 32.767;

    if( speed < -32.768 )
        speed = -32.768;
    // <<<<< 16 bit saturation

    // >>>>> New SetPoint to RoboController
    quint16 address;
    if(motorIdx == 0)
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

    sendCommand( mUdpControlSocket, CMD_WR_MULTI_REG, data );
    // <<<<< New SetPoint to RoboController
}

void RoboControllerSDK::setMotorPidGains( quint16 motorIdx, quint16 Kp, quint16 Ki, quint16 Kd )
{
    // >>>>> New SetPoint to RoboController
    quint16 address;
    if(motorIdx == 0)
        address = WORD_PID_P_LEFT;
    else
        address = WORD_PID_P_RIGHT;

    QVector<quint16> data;
    data << address;
    //data << (quint16)nReg; <- Not requested by sendCommand!!!

    data << Kp;
    data << Ki;
    data << Kd;

    sendCommand( mTcpSocket, CMD_WR_MULTI_REG, data );
    // <<<<< New SetPoint to RoboController
}

void RoboControllerSDK::onPingTimerTimeout()
{
    QVector<quint16> pingData;
    pingData << (quint16)WORD_COMWATCHDOG_TIME;
    pingData << 1; // Only one register

    quint64 elapsed = QDateTime::currentDateTime().toMSecsSinceEpoch() - mLastServerReqTime;

    qDebug() << tr(" Ping WatchDog - Elapsed: %1").arg(elapsed);

    sendCommand( mTcpSocket, CMD_RD_MULTI_REG, pingData );
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
    data << 19; // Only one register

    sendCommand( mTcpSocket, CMD_RD_MULTI_REG, data );
    // <<<<< Robot Configuration Data

    // >>>>> Robot Configuration Bits
    data.clear();
    data << (quint16)WORD_STATUSBIT2;
    data << 1;

    sendCommand( mTcpSocket, CMD_RD_MULTI_REG, data );
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
    ini.sync();
}

void RoboControllerSDK::setRobotConfiguration( RobotConfiguration& roboConfig )
{
    memcpy( &mRobotConfig, &roboConfig, sizeof(RobotConfiguration) );
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

    sendCommand( mTcpSocket, CMD_WR_MULTI_REG, data );
    // <<<<< Robot Configuration Data (19 consequtive registers)

    // >>>>> Status Register 2
    data.clear();
    data << (quint16)WORD_STATUSBIT2;

    quint16 statusVal = 0;
    if(mRobotConfig.EncoderPosition)
        statusVal |= FLG_STATUSBI2_EEPROM_ENCODER_POSITION;
    if(mRobotConfig.MotorEnableLevel)
        statusVal |= FLG_STATUSBI2_EEPROM_ENCODER_POSITION;

    data << statusVal;

    sendCommand( mTcpSocket, CMD_WR_MULTI_REG, data );
    // <<<<< Status Register 2
}

}
