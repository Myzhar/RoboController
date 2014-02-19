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

namespace roboctrl
{

QRobotServer::QRobotServer(int serverUdpCommand/*=4550*/,int serverUdpStatus/*=4560*/,int serverTcpPort/*=4500*/, QObject *parent/*=0*/) :
    QThread(parent),
    mTcpServer(NULL),
    mTcpSocket(NULL),
    mUdpStatusSocket(NULL),
    mUdpCommandSocket(NULL),
    mSettings(NULL),
    mServerTcpPort(serverTcpPort),
    mServerUdpCommandPort(serverUdpCommand),
    mServerUdpStatusPort(serverUdpStatus),
    mModbus(NULL),
    mReplyBuffer(NULL),
    mBoardConnected(false),
    mMsgCounter(0)
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

    mReplyBufSize = INITIAL_REPLY_BUFFER_SIZE;
    mReplyBuffer = new quint16[mReplyBufSize];

    mBoardIdx = mSettings->value( "boardidx", "0" ).toInt();
    if( mBoardIdx==0 )
    {
        mBoardIdx = 1;
        mSettings->setValue( "boardidx", QString("%1").arg(mBoardIdx) );
        mSettings->sync();
    }

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

    // >>>>> TCP configuration
    mServerTcpPort = mSettings->value( "TCP_server_port", "0" ).toUInt();
    if( mServerTcpPort==0 )
    {
        mServerTcpPort = 4500;
        mSettings->setValue( "TCP_server_port", QString("%1").arg(mServerTcpPort) );
        mSettings->sync();
    }

    openTcpSession();

    connect(mTcpServer, SIGNAL(newConnection()), this, SLOT(onNewTcpConnection()));
    // <<<<< TCP configuration
    
    // >>>>> UDP Status configuration
    mServerUdpStatusPort = mSettings->value( "UDP_status_server_port", "0" ).toUInt();
    if( mServerUdpStatusPort==0 )
    {
        mServerUdpStatusPort = 4560;
        mSettings->setValue( "UDP_status_server_port", QString("%1").arg(mServerUdpStatusPort) );
        mSettings->sync();
    }

    openUdpStatusSession();
    // <<<<< UDP Status configuration
    
    // >>>>> UDP Command configuration
    mServerUdpCommandPort = mSettings->value( "UDP_command_server_port", "0" ).toUInt();
    if( mServerUdpCommandPort==0 )
    {
        mServerUdpCommandPort = 4550;
        mSettings->setValue( "UDP_command_server_port", QString("%1").arg(mServerUdpCommandPort) );
        mSettings->sync();
    }

    openUdpCommandSession();
    // <<<<< UDP Command configuration

    mSettings->sync();

    mClientCount = 0;

    // Start Server Thread
    this->start();

    // Start Ping Timer
    startTimer( TEST_TIMER_INTERVAL, Qt::PreciseTimer );
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
            qDebug() << tr("Server running on IP: %1 Port: %2" )
                        .arg(ipAddress).arg(mTcpServer->serverPort());
        }
    }
}

void QRobotServer::openUdpStatusSession()
{
    if( mUdpStatusSocket )
        delete mUdpStatusSocket;

    mUdpStatusSocket = new QUdpSocket();

    if( !mUdpStatusSocket->bind( mServerUdpStatusPort, QAbstractSocket::ShareAddress ) )
    {
        qCritical() << tr("Unable to bind the UDP Status server on %1 port. Error: %2")
                       .arg(mServerUdpStatusPort).arg(mUdpStatusSocket->errorString());
        return;
    }

    connect( mUdpStatusSocket, SIGNAL(readyRead()),
             this, SLOT(onUdpStatusReadyRead()) );
}

void QRobotServer::openUdpCommandSession()
{
    if( mUdpCommandSocket )
        delete mUdpStatusSocket;

    mUdpCommandSocket = new QUdpSocket();

    if( !mUdpCommandSocket->bind( mServerUdpCommandPort, QAbstractSocket::ShareAddress ) )
    {
        qCritical() << tr("Unable to bind the UDP Command server on %1 port. Error: %2")
                       .arg(mServerUdpCommandPort).arg(mUdpCommandSocket->errorString());

        return;
    }

    connect( mUdpCommandSocket, SIGNAL(readyRead()),
             this, SLOT(onUdpCommandReadyRead()) );
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
             this, SLOT(onClientDisconnected()) );
    connect( mTcpSocket, SIGNAL(readyRead()),
             this, SLOT(onTcpReadyRead()) );

    QVector<quint16> data;
    data << (quint16)mBoardIdx;
    sendBlock( mTcpSocket, MSG_CONNECTED, data );

    mClientCount++;
}

void QRobotServer::onNewUdpStatusConnection()
{
    // TODO SEND MESSAGE MSG_CONNECTED
}

void QRobotServer::onNewUdpCommandConnection()
{
    // TODO SEND MESSAGE MSG_CONNECTED
}

void QRobotServer::sendBlock( QAbstractSocket* socket, quint16 msgCode, QVector<quint16>& data )
{
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_4_0);
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
    int blockSize = (block.size() - sizeof(quint16));
    out << (quint16)blockSize;

    socket->write( block );
    socket->flush();

    QString timeStr = QDateTime::currentDateTime().toString( "hh:mm:ss.zzz" );
    qDebug() << tr("%1 - Sent msg #%2").arg(timeStr).arg(mMsgCounter);

}

void QRobotServer::onTcpReadyRead()
{
    QDataStream in(mTcpSocket);
    in.setVersion(QDataStream::Qt_4_0);

    int headerSize = 3; // [blockSize][msgIdx][msgCode]

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
        case CMD_SERVER_PING_REQ: // Sent by client to verify that Server is running
        {
            qDebug() << tr("Received msg #%1: MSG_SERVER_PING_REQ").arg(msgIdx);

            QVector<quint16> vec;
            sendBlock( mTcpSocket, MSG_SERVER_PING_OK, vec );

            break;
        }

        case CMD_RD_MULTI_REG:
        {
            qDebug() << tr("Received msg #%1: CMD_RD_MULTI_REG").arg(msgIdx);

            if( !mBoardConnected )
            {
                QVector<quint16> vec;
                sendBlock( mTcpSocket, MSG_RC_NOT_FOUND, vec);

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
            memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) ); // TODO Verificare!!!

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_RD_MULTI_REG;
                vec << startAddr;
                sendBlock( mTcpSocket, MSG_FAILED, vec );
            }
            else
                sendBlock(mTcpSocket, MSG_READ_REPLY, readRegReply );

            break;
        }

        case CMD_WR_MULTI_REG:
        {
            qDebug() << tr("Received msg #%1: CMD_WR_MULTI_REG").arg(msgIdx);

            if( !mBoardConnected )
            {
                QVector<quint16> vec;
                sendBlock( mTcpSocket, MSG_RC_NOT_FOUND, vec );
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
                sendBlock( mTcpSocket, MSG_FAILED, vec );
            }
            else
            {
                QVector<quint16> vec;
                vec << startAddr;
                vec << nReg;
                sendBlock( mTcpSocket, MSG_WRITE_OK, vec );
            }
            break;
        }

        default:
        {
            qDebug() << tr("Received wrong message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

            QVector<quint16> vec;
            sendBlock( mTcpSocket, MSG_FAILED, vec );

            break;
        }
        }

        mNextTcpBlockSize = 0;
    }
}

void QRobotServer::onUdpStatusReadyRead()
{
    QDataStream in(mUdpStatusSocket);
    in.setVersion(QDataStream::Qt_4_0);

    int headerSize = 3; // [blockSize][msgIdx][msgCode]

    mNextUdpStatBlockSize=0;
    quint16 msgCode;

    forever // Receiving data while there is data available
    {
        if( mNextUdpStatBlockSize==0) // No incomplete blocks received before
        {
            if (mUdpStatusSocket->bytesAvailable() < (qint64)sizeof(quint16))
            {
                qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
                break;
            }

            // Datagram dimension
            in >> mNextUdpStatBlockSize; // Updated only if we are parsing a new block
        }

        if (mUdpStatusSocket->bytesAvailable() < mNextUdpStatBlockSize)
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
        case CMD_SERVER_PING_REQ: // Sent by client to verify that Server is running
        {
            qDebug() << tr("Received msg #%1: MSG_SERVER_PING_REQ").arg(msgIdx);

            QVector<quint16> vec;
            sendBlock( mTcpSocket, MSG_SERVER_PING_OK, vec );

            break;
        }

        case CMD_RD_MULTI_REG:
        {
            qDebug() << tr("Received msg #%1: CMD_RD_MULTI_REG").arg(msgIdx);

            if( !mBoardConnected )
            {
                QVector<quint16> vec;
                sendBlock( mUdpStatusSocket, MSG_RC_NOT_FOUND, vec);

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
            memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) ); // TODO Verificare!!!

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_RD_MULTI_REG;
                vec << startAddr;
                sendBlock( mUdpStatusSocket, MSG_FAILED, vec );
            }
            else
                sendBlock( mUdpStatusSocket, MSG_READ_REPLY, readRegReply );

            break;
        }

        case CMD_WR_MULTI_REG:
        {
            qDebug() << tr("Received msg #%1: CMD_WR_MULTI_REG").arg(msgIdx);

            if( !mBoardConnected )
            {
                QVector<quint16> vec;
                sendBlock( mUdpStatusSocket, MSG_RC_NOT_FOUND, vec );
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
                sendBlock( mUdpStatusSocket, MSG_FAILED, vec );
            }
            else
            {
                QVector<quint16> vec;
                vec << startAddr;
                vec << nReg;
                sendBlock( mUdpStatusSocket, MSG_WRITE_OK, vec );
            }
            break;
        }

        default:
        {
            qDebug() << tr("Received wrong message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

            QVector<quint16> vec;
            sendBlock( mUdpStatusSocket, MSG_FAILED, vec );

            break;
        }
        }

        mNextUdpStatBlockSize = 0;
    }
}

void QRobotServer::onUdpCommandReadyRead()
{
    QDataStream in(mUdpCommandSocket);
    in.setVersion(QDataStream::Qt_4_0);

    int headerSize = 3; // [blockSize][msgIdx][msgCode]

    mNextUdpCmdBlockSize=0;
    quint16 msgCode;

    forever // Receiving data while there is data available
    {
        if( mNextUdpCmdBlockSize==0) // No incomplete blocks received before
        {
            if (mUdpCommandSocket->bytesAvailable() < (qint64)sizeof(quint16))
            {
                qDebug() << Q_FUNC_INFO << tr("No more TCP Data available");
                break;
            }

            // Datagram dimension
            in >> mNextUdpCmdBlockSize; // Updated only if we are parsing a new block
        }

        if (mUdpCommandSocket->bytesAvailable() < mNextUdpCmdBlockSize)
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
        case CMD_SERVER_PING_REQ: // Sent by client to verify that Server is running
        {
            qDebug() << tr("Received msg #%1: MSG_SERVER_PING_REQ").arg(msgIdx);

            QVector<quint16> vec;
            sendBlock( mTcpSocket, MSG_SERVER_PING_OK, vec );

            break;
        }

        case CMD_RD_MULTI_REG:
        {
            qDebug() << tr("Received msg #%1: CMD_RD_MULTI_REG").arg(msgIdx);

            if( !mBoardConnected )
            {
                QVector<quint16> vec;
                sendBlock( mUdpCommandSocket, MSG_RC_NOT_FOUND, vec);

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
            memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) ); // TODO Verificare!!!

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_RD_MULTI_REG;
                vec << startAddr;
                sendBlock( mUdpCommandSocket, MSG_FAILED, vec );
            }
            else
                sendBlock( mUdpCommandSocket, MSG_READ_REPLY, readRegReply );

            break;
        }

        case CMD_WR_MULTI_REG:
        {
            qDebug() << tr("Received msg #%1: CMD_WR_MULTI_REG").arg(msgIdx);

            if( !mBoardConnected )
            {
                QVector<quint16> vec;
                sendBlock( mUdpCommandSocket, MSG_RC_NOT_FOUND, vec );
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

            bool commOk = writeMultiReg( startAddr, nReg, vals );

            if( !commOk )
            {
                QVector<quint16> vec;
                vec << CMD_WR_MULTI_REG;
                vec << startAddr;
                sendBlock( mUdpCommandSocket, MSG_FAILED, vec );
            }
            else
            {
                QVector<quint16> vec;
                vec << startAddr;
                vec << nReg;
                sendBlock( mUdpCommandSocket, MSG_WRITE_OK, vec );
            }
            break;
        }

        default:
        {
            qDebug() << tr("Received wrong message code(%1) with msg #%2").arg(msgCode).arg(msgIdx);

            QVector<quint16> vec;
            sendBlock( mUdpCommandSocket, MSG_FAILED, vec );

            break;
        }
        }

        mNextUdpCmdBlockSize = 0;
    }
}

void QRobotServer::onClientDisconnected()
{
    qDebug() << tr( "Client disconnected" );
    //delete mClientConnection;
    //mClientConnection = NULL;

    mClientCount--;

    // TODO Disconnect also over UDP

    // TODO Stop the server if all the clients disconnected?
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
    if( event->timerId() == mTestTimerId )
    {
        if(mClientCount<1)
            return;

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
