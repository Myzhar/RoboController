#include "qrobocontrollerinterface.h"
#include <QDebug>
#include <QThread>
#include <rcexception.h>

namespace roboctrl
{

#define WORD_TEST_BOARD 0
#define INITIAL_REPLY_BUFFER_SIZE 20

QRoboControllerInterface::QRoboControllerInterface(int boardIdx, QString serialPort,
                                                   int serialbaudrate, char parity,
                                                   int data_bit, int stop_bit,
                                                   QObject *parent) :
    QObject(parent),
    mModbus(NULL),
    mReplyBuffer(NULL)
{
    mBoardConnected = false;

    mReplyBufSize = INITIAL_REPLY_BUFFER_SIZE;
    mReplyBuffer = new quint16[mReplyBufSize];

    mBoardIdx = boardIdx;
    int count = 0;

    qDebug() << tr("#%1 - Initializing connection to RoboController Id: %2").arg(count+1).arg(mBoardIdx);

    while( !initializeSerialModbus( serialPort.toLatin1().data(),
                                    serialbaudrate, parity, data_bit, stop_bit ) &&
           count < 10 )
    {
        qWarning() << tr( "Failed to initialize mod_bus on port: %1").arg(serialPort);
        qWarning() << tr("Trying again in one second...");

        count++;

        QThread::msleep( 1000 );
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

    // >>>>> Board connection
    bool res = connectModbus( 10 );
    if( !res )
    {
        QString err = tr("Failed to connect to modbus on port: %1").arg(serialPort);
        qCritical() << "Server not started";
        qCritical() << err;

        roboctrl::RcException exc(excRoboControllerNotFound, err.toStdString().c_str() );

        throw exc;
    }

    qDebug() << tr("RoboController connected");
    // <<<<< Board connection

    // <<<<< MOD_BUS serial communication settings
}

QRoboControllerInterface::~QRoboControllerInterface()
{
    if( mModbus )
    {
        modbus_close( mModbus );
        modbus_free( mModbus );
    }

    if(mReplyBuffer)
        delete [] mReplyBuffer;
}

modbus_t* QRoboControllerInterface::initializeSerialModbus( const char *device,
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

bool QRoboControllerInterface::connectModbus( int retryCount/*=-1*/)
{
    if( !mModbus )
    {
        qCritical() /*<< PREFIX*/ << "ModBus data structure not initialized!";
        return false;
    }

    // Closing to reset active connections

    //modbus_close( mModbus);

    if( modbus_connect( mModbus ) == -1 )
    {
        qCritical() /*<< PREFIX*/ << "Modbus connection failed";
        mBoardConnected = false;
        return false;
    }

    int res=-1;
    // res = modbus_flush( mModbus );

    QThread::msleep( 1000 );

    timeval new_timeout;
    new_timeout.tv_sec = 2;
    new_timeout.tv_usec = 0;
    modbus_set_response_timeout( mModbus, &new_timeout );
    modbus_set_byte_timeout( mModbus, &new_timeout );

    res = modbus_set_slave( mModbus, mBoardIdx );
    if( res != 0 )
    {
        qCritical() /*<< PREFIX*/ << ": modbus_set_slave error -> " <<  modbus_strerror( errno );

        modbus_flush( mModbus );

        mBoardConnected = false;
        return false;
    }
    //qDebug() /*<< PREFIX*/ << "Modbus connected";

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
        mBoardConnected = false;
        return false;
    }

    mBoardConnected = true;
    return true;
}

bool QRoboControllerInterface::testBoardConnection()
{
    quint16 startAddr = WORD_TEST_BOARD;
    quint16 nReg = 1;

    QVector<quint16> reply = readMultiReg( startAddr, nReg );

    if(reply.isEmpty())
    {
        qCritical() /*<< PREFIX*/ << "Board Ping failed!!!";
        /*qCritical() << PREFIX << "modbus_read_input_registers error -> " <<  modbus_strerror( errno )
                    << "[First regAddress: " << WORD_TEST_BOARD << "- #reg: " << nReg <<  "]";*/

        mBoardConnected = false;
        return false;
    }

    mBoardConnected = true;
    return true;
}

QVector<quint16> QRoboControllerInterface::readMultiReg(quint16 startAddr, quint16 nReg)
{
    QVector<quint16> readRegReply;

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
            qCritical() /*<< PREFIX*/ << "modbus_read_input_registers error -> " <<  modbus_strerror( errno )
                        << "[First regAddress: " << startAddr << "- #reg: " << nReg <<  "]";

            mBoardMutex.unlock();
            return readRegReply;
        }

        readRegReply.resize( nReg+2 );

        readRegReply[0] = (quint16)startAddr;
        readRegReply[1] = (quint16)nReg;
        memcpy( (quint16*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(quint16) );
    }
    mBoardMutex.unlock();

    return readRegReply;
}

bool QRoboControllerInterface::writeMultiReg( quint16 startAddr, quint16 nReg,
                                  QVector<quint16> vals )
{
    mBoardMutex.lock();
    {
        int res = modbus_write_registers( mModbus, startAddr, nReg, vals.data() );

        if(res!=nReg)
        {
            qCritical() /*<< PREFIX*/ << "modbus_write_registers error -> " <<  modbus_strerror( errno )
                        << "[First regAddress: " << startAddr << "- #reg: " << nReg <<  "]";

            mBoardMutex.unlock();
            return false;
        }
    }
    mBoardMutex.unlock();
    return true;
}

}
