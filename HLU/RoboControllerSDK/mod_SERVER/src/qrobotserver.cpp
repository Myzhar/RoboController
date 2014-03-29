#include <qrobotserver.h>

#include <QtNetwork/QtNetwork>

#include <modbus.h>
#include <modbus-private.h>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <loghandler.h>
#include <errno.h>
#include <QCoreApplication>
#include <QMutex>
#include "modbus_registers.h"

namespace roboctrl
{

QRobotServer::QRobotServer(quint16 serverUdpControl/*=14560*/,
                           quint16 serverUdpStatusListener/*=14550*/, quint16 serverUdpStatusSender/*=14555*/,
                           quint16 serverTcpPort/*=14500*/, bool testMode, QObject *parent/*=0*/) :
    QThread(parent),
    mTcpServer(NULL),
    mTcpSocket(NULL),
    mUdpStatusSocket(NULL),
    mUdpControlSocket(NULL),
    mSettings(NULL),
    mServerTcpPort(serverTcpPort),
    mServerUdpStatusPortListen(serverUdpStatusListener),
    mServerUdpStatusPortSend(serverUdpStatusSender),
    mServerUdpControlPortListen(serverUdpControl),
    mModbus(NULL),
    mReplyBuffer(NULL),
    mBoardConnected(false),
    mControllerClientIp(""),
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
    mServerUdpStatusPortListen = mSettings->value( "UDP_status_server_port_listener", "0" ).toUInt();
    if( mServerUdpStatusPortListen==0 )
    {
        mServerUdpStatusPortListen = 14550;
        mSettings->setValue( "UDP_status_server_port_listener", QString("%1").arg(mServerUdpStatusPortListen) );
        mSettings->sync();
    }

    mServerUdpStatusPortSend = mSettings->value( "UDP_status_server_port_senderlistener", "0" ).toUInt();
    if( mServerUdpStatusPortSend==0 )
    {
        mServerUdpStatusPortSend = 14555;
        mSettings->setValue( "UDP_status_server_port_senderlistener", QString("%1").arg(mServerUdpStatusPortSend) );
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

    openUdpControlSession();
    // <<<<< UDP Control configuration

    mSettings->sync();

    mTcpClientCount = 0;

    // Start Server Thread
    this->start();

    if(!mTestMode)
    {
        // Start Ping Timer
        mBoardTestTimerId = startTimer( TEST_TIMER_INTERVAL, Qt::PreciseTimer );
    }
}

QRobotServer::~QRobotServer()
{
    if(this->isRunning())
    {
        terminate();
        while(this->isRunning());
    }

    if(mTcpServer)
        delete mTcpServer;

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
    if( mUdpStatusSocket )
        delete mUdpStatusSocket;

    mUdpStatusSocket = new QUdpSocket(this);

    if( !mUdpStatusSocket->bind( mServerUdpStatusPortListen, /*QAbstractSocket::ReuseAddressHint|*/QAbstractSocket::ShareAddress ) )
    {
        qCritical() << tr("Unable to bind the UDP Status server on %1:%2. Error: %3")
                       .arg(mUdpStatusSocket->localAddress().toString())
                       .arg(mUdpStatusSocket->localPort() )
                       .arg(mUdpStatusSocket->errorString());
        return;
    }

    connect( mUdpStatusSocket, SIGNAL(readyRead()),
             this, SLOT(onUdpStatusReadyRead()) );

    qDebug() << tr("UDP Status Server listening on port %1" )
                .arg(mUdpStatusSocket->localPort() );

}

void QRobotServer::openUdpControlSession()
{
    if( mUdpControlSocket )
        delete mUdpStatusSocket;

    mUdpControlSocket = new QUdpSocket(this);

    if( !mUdpControlSocket->bind( mServerUdpControlPortListen, /*QAbstractSocket::ReuseAddressHint|*/QAbstractSocket::ShareAddress  ) )
    {
        qCritical() << tr("Unable to bind the UDP Control server on %1:%2. Error: %3")
                       .arg(mUdpControlSocket->localAddress().toString())
                       .arg(mUdpControlSocket->localPort() )
                       .arg(mUdpStatusSocket->errorString()) ;

        return;
    }

    connect( mUdpControlSocket, SIGNAL(readyRead()),
             this, SLOT(onUdpControlReadyRead()) );

    qDebug() << tr("UDP Control Server listening on port %1" )
                .arg(mUdpControlSocket->localPort() );
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

    QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
    qDebug() << tr("%1 - Sent TCP msg #%2 - Code: %3").arg(timeStr).arg(mMsgCounter).arg(msgCode);
}

void QRobotServer::sendStatusBlockUDP( QHostAddress addr, quint16 msgCode, QVector<quint16>& data )
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

    mUdpStatusSocket->writeDatagram( block, addr, mServerUdpStatusPortSend );
    mUdpStatusSocket->flush();

    QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
    qDebug() << tr("%1 - Sent msg #%2 -> %3 to %4:%5").arg(timeStr).arg(mMsgCounter).arg(msgCode).arg(addr.toString()).arg(mServerUdpStatusPortSend);
}

void QRobotServer::onTcpReadyRead()
{
    QDataStream in(mTcpSocket);
    in.setVersion(QDataStream::Qt_5_2);

    int headerSize = 4; // [start_word][blockSize][msgIdx][msgCode]

    mNextTcpBlockSize=0;
    quint16 msgCode;

    forever // Receiving data while there is data available
    {
        qint64 bytesAvailable = mTcpSocket->bytesAvailable();

        QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );
        if( mNextTcpBlockSize==0) // No incomplete blocks received before
        {
            if (bytesAvailable < (qint64)sizeof(quint16))
            {
                qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
                break;
            }

            /*quint16 val16 = 0x0000;
            while( val16 != TCP_START_VAL ) // TODO: VERIFICARE!!!
            {
                in >> val16;
            }*/

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
            while( val16 != TCP_START_VAL ); // TODO verify if this works!
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

        QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
        qDebug() << tr("%1 - TCP Received msg #%2").arg(timeStr).arg(msgIdx);

        // Datagram Code
        in >> msgCode;

        switch(msgCode)
        {
        case CMD_SERVER_PING_REQ: // Sent by client to verify that Server is running
        {
            qDebug() << tr("TCP Received msg #%1: MSG_SERVER_PING_REQ (%2)").arg(msgIdx).arg(msgCode);


            QVector<quint16> vec;
            sendBlockTCP( MSG_SERVER_PING_OK, vec );

            break;
        }

        case CMD_RD_MULTI_REG:
        {
            qDebug() << tr("TCP Received msg #%1: CMD_RD_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

            if( !mBoardConnected )
            {
                // TODO: LEGGERE I BYTE RIMANENTI
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
            qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

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
            qDebug() << tr("TCP Received msg #%1: CMD_WR_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

            if( !mBoardConnected )
            {
                // TODO: LEGGERE I BYTE RIMANENTI
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

            qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

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
    }
}

void QRobotServer::onUdpStatusReadyRead()
{
    int headerSize = 4; // [start_word][blockSize][msgIdx][msgCode]

    mNextUdpStatBlockSize=0;
    quint16 msgCode;

    while( mUdpStatusSocket->hasPendingDatagrams() ) // Receiving data while there is data available
    {
        QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );

        QByteArray buffer( mUdpStatusSocket->pendingDatagramSize(), 0 );
        qint64 datagramSize = mUdpStatusSocket->pendingDatagramSize();

        if( buffer.size()< datagramSize )
            buffer.resize( datagramSize );

        QHostAddress addr;
        quint16 port;

        mUdpStatusSocket->readDatagram( buffer.data(), buffer.size(), &addr, &port );

        QDataStream in( buffer );
        in.setVersion(QDataStream::Qt_5_2);

        //while( !in.atEnd() )
        {
            //QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );

            if( mNextUdpStatBlockSize==0) // No incomplete blocks received before
            {
                if (datagramSize < (qint64)sizeof(quint16))
                {
                    //qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
                    break;
                }

                /*int count = 0;
                while( mNextUdpStatBlockSize == 0 )
                {
                    // Datagram dimension
                    in >> mNextUdpStatBlockSize; // Updated only if we are parsing a new block

                    count++;
                    if(count == datagramSize)
                    {
                        QDataStream::Status st = in.status();

                        qCritical() << Q_FUNC_INFO << tr("Read %1 bytes equal to ZERO. Stream status: %2")
                                       .arg(datagramSize).arg(st);
                        return;
                    }
                }*/
                int count = 0;
                quint16 val16;
                in >> val16;

                while( val16 != UDP_START_VAL ) // TODO verify if this works!
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

                // Datagram dimension
                in >> mNextUdpStatBlockSize; // Updated only if we are parsing a new block
            }

            if( datagramSize < mNextUdpStatBlockSize )
            {
                qDebug() << Q_FUNC_INFO << tr("Received incomplete UDP Status Block... waiting for the missing data");
                break;
            }

            // Datagram IDX
            quint16 msgIdx;
            in >> msgIdx;

            QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
            qDebug() << tr("%1 - UDP Status Received msg #%2 by %3:%4").arg(timeStr).arg(msgIdx).arg(addr.toString()).arg(port);

            // Datagram Code
            in >> msgCode;

            switch(msgCode)
            {
            case CMD_SERVER_PING_REQ: // Sent by client to verify that Server is running
            {
                qDebug() << tr("UDP Status Received msg #%1: CMD_SERVER_PING_REQ (%2)").arg(msgIdx).arg(msgCode);

                QVector<quint16> vec;
                sendStatusBlockUDP( addr, MSG_SERVER_PING_OK, vec );

                break;
            }

            case CMD_RD_MULTI_REG:
            {
                qDebug() << tr("UDP Status Received msg #%1: CMD_RD_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

                if( !mBoardConnected )
                {
                    // TODO: LEGGERE I BYTE RIMANENTI
                    mUdpStatusSocket->read( mNextUdpStatBlockSize-2 ); // Tolgo i byte rimanenti dal buffer

                    QVector<quint16> vec;
                    sendStatusBlockUDP( addr, MSG_RC_NOT_FOUND, vec );

                    qCritical() << Q_FUNC_INFO << "CMD_RD_MULTI_REG - Board not connected!";
                    break;
                }

                quint16 startAddr;
                in >> startAddr; // First word to be read
                quint16 nReg;
                in >> nReg;
                qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

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
                    //sendBlockTCP( mUdpStatusSocket, MSG_FAILED, vec );
                    sendStatusBlockUDP( addr, MSG_FAILED, vec );
                }
                else
                    //sendBlockTCP( mUdpStatusSocket, MSG_READ_REPLY, readRegReply );
                    sendStatusBlockUDP( addr, MSG_READ_REPLY, readRegReply );

                break;
            }

            case CMD_WR_MULTI_REG:
            {
                qDebug() << tr("UDP Status Received msg #%1: CMD_WR_MULTI_REG (%2)").arg(msgIdx).arg(msgCode);

                if( !mBoardConnected )
                {
                    // TODO: LEGGERE I BYTE RIMANENTI
                    mUdpStatusSocket->read( mNextUdpStatBlockSize-2 ); // Tolgo i byte rimanenti dal buffer

                    QVector<quint16> vec;
                    sendStatusBlockUDP( addr, MSG_RC_NOT_FOUND, vec );
                    break;
                }

                quint16 startAddr;
                in >> startAddr;  // First word to be read

                // We can extract data size (nReg!) from message without asking it to client in the protocol
                int nReg = (mNextUdpStatBlockSize/sizeof(quint16)) - headerSize;

                qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

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
                    sendStatusBlockUDP( addr, MSG_FAILED, vec );
                }
                else
                {
                    QVector<quint16> vec;
                    vec << startAddr;
                    vec << nReg;
                    sendStatusBlockUDP( addr, MSG_WRITE_OK, vec );
                }
                break;
            }

            case CMD_GET_ROBOT_CTRL:
            {
                qDebug() << tr("UDP Status Received msg #%1: CMD_GET_ROBOT_CTRL (%2)").arg(msgIdx).arg(msgCode);

                if( mControllerClientIp.isEmpty() || mControllerClientIp==addr.toString() )
                {
                    mControllerClientIp = addr.toString();
                    QVector<quint16> vec;
                    sendStatusBlockUDP( addr, MSG_ROBOT_CTRL_OK, vec ); // Robot control taken
                }
                else
                {
                    QVector<quint16> vec;
                    sendStatusBlockUDP( addr, MSG_ROBOT_CTRL_KO, vec ); // Robot not free
                }
                break;
            }

            case CMD_REL_ROBOT_CTRL:
            {
                qDebug() << tr("UDP Status Received msg #%1: CMD_LEAVE_ROBOT_CTRL (%2)").arg(msgIdx).arg(msgCode);

                mControllerClientIp = "";

                QVector<quint16> vec;
                sendStatusBlockUDP( addr, MSG_ROBOT_CTRL_RELEASED, vec ); // Robot control released

                break;
            }

            default:
            {
                qDebug() << tr("Received unknown message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);


                qint64 bytes = mUdpStatusSocket->pendingDatagramSize();
                if(bytes>0)
                {
                    qDebug() << tr("Removing %1 bytes from UDP Status socket buffer").arg(bytes);
                    char* buf = new char[bytes];
                    //in.readRawData( buf, mNextTcpBlockSize );
                    in.readRawData( buf, bytes );
                    delete [] buf;
                }

                break;
            }
            }

            mNextUdpStatBlockSize = 0;
        }
    }
}

void QRobotServer::onUdpControlReadyRead()
{
    int headerSize = 4; // [start_word][blockSize][msgIdx][msgCode]

    mNextUdpCmdBlockSize=0;
    quint16 msgCode;

    while( mUdpControlSocket->hasPendingDatagrams() ) // Receiving data while there is data available
    {
        QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );

        QByteArray buffer( mUdpControlSocket->pendingDatagramSize(), 0 );
        qint64 datagramSize = mUdpControlSocket->pendingDatagramSize();

        if( buffer.size()< datagramSize )
            buffer.resize( datagramSize );

        QHostAddress addr;
        quint16 port;

        mUdpControlSocket->readDatagram( buffer.data(), buffer.size(), &addr, &port );

        QDataStream in( buffer );
        in.setVersion(QDataStream::Qt_5_2);

        //while( !in.atEnd() )
        {
            //QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );

            if( mNextUdpCmdBlockSize==0) // No incomplete blocks received before
            {
                if (datagramSize < (qint64)sizeof(quint16))
                {
                    //qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
                    break;
                }

                /*int count = 0;
                while( mNextUdpCmdBlockSize == 0 )
                {
                    // Datagram dimension
                    in >> mNextUdpCmdBlockSize; // Updated only if we are parsing a new block

                    count++;
                    if(count == datagramSize)
                    {
                        QDataStream::Status st = in.status();

                        qCritical() << Q_FUNC_INFO << tr("Read %1 bytes equal to ZERO. Stream status: %2")
                                       .arg(datagramSize).arg(st);
                        return;
                    }
                }*/

                int count = 0;
                quint16 val16;
                in >> val16;

                while( val16 != UDP_START_VAL ) // TODO verify if this works!
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

                // Datagram dimension
                in >> mNextUdpCmdBlockSize; // Updated only if we are parsing a new block
            }

            if( datagramSize < mNextUdpCmdBlockSize )
            {
                qDebug() << Q_FUNC_INFO << tr("Received incomplete UDP Control Block... waiting for the missing data");
                break;
            }

            // Datagram IDX
            quint16 msgIdx;
            in >> msgIdx;

            QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
            qDebug() << tr("%1 - UDP Status Received msg #%2").arg(timeStr).arg(msgIdx);

            // Datagram Code
            in >> msgCode;

            switch(msgCode)
            {
            case CMD_WR_MULTI_REG:
            {
                qDebug() << tr("UDP Control Received msg #%1: CMD_WR_MULTI_REG").arg(msgIdx);

                if( !mBoardConnected )
                {
                    // TODO: LEGGERE I BYTE RIMANENTI
                    mUdpControlSocket->read( mNextUdpCmdBlockSize-2 ); // Tolgo i byte rimanenti dal buffer

                    qCritical() << Q_FUNC_INFO << "CMD_WR_MULTI_REG - Board not connected!";
                    break;
                }

                quint16 startAddr;
                in >> startAddr;  // First word to be read

                // We can extract data size (nReg!) from message without asking it to client in the protocol
                int nReg = (mNextUdpCmdBlockSize/sizeof(quint16)) - headerSize;

                qDebug() << tr("Starting address: %1 - #reg: %2").arg(startAddr).arg(nReg);

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
                // if the client has the control of the robot. If we not send the information
                // to the client using the UDP Status Socket
                if(addr.toString()!= mControllerClientIp) // The client has no control of the robot
                {
                    QVector<quint16> vec;
                    sendStatusBlockUDP( addr, MSG_ROBOT_CTRL_KO, vec ); // Robot not controlled by client

                    if(mControllerClientIp.isEmpty())
                        qDebug() << tr("The client %1 cannot send commands before taking control of the robot").arg(addr.toString());
                    else
                        qDebug() << tr("The client %1 is controlling the robot. %2 cannot send commands").arg(mControllerClientIp).arg(addr.toString());

                    break;
                }

                bool commOk = writeMultiReg( startAddr, nReg, vals );

                if( !commOk )
                {
                    qDebug() << tr("Error writing %1 registers, starting from %2").arg(nReg).arg(startAddr);
                }

                readSpeedsAndSend( addr );

                break;
            }

            default:
            {
                qDebug() << tr("UDP Control Received wrong message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

                qint64 bytes = mUdpControlSocket->pendingDatagramSize();
                if(bytes>0)
                {
                    qDebug() << tr("Removing %1 bytes from UDP Control socket buffer").arg(bytes);
                    char* buf = new char[bytes];
                    //in.readRawData( buf, mNextTcpBlockSize );
                    in.readRawData( buf, bytes );
                    delete [] buf;
                }

                break;
            }
            }

            mNextUdpCmdBlockSize = 0;
        }
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

void QRobotServer::readSpeedsAndSend( QHostAddress addr )
{
    quint16 startAddr = WORD_ENC1_SPEED;
    quint16 nReg = 2;
    bool commOk = readMultiReg( startAddr, nReg );

    QVector<quint16> readRegReply;
    readRegReply.resize( nReg+2 );

    readRegReply[0] = (quint16)startAddr;
    readRegReply[1] = (quint16)nReg;
    memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) );

    if( commOk )
        sendStatusBlockUDP( addr, MSG_READ_REPLY, readRegReply );
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

void QRobotServer::timerEvent(QTimerEvent *event)
{
    if( event->timerId() == mBoardTestTimerId )
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
            qDebug() << "Ping Ok";
        }
    }
}

void QRobotServer::run()
{
    qDebug() << tr("QRobotServer thread started");

    exec();

    qDebug() << tr("QRobotServer thread finished");
}

}
