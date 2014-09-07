#include "rbctrliface.h"

#include <ros/ros.h>
#include <errno.h>

#include <vector>

#define WORD_TEST_BOARD 0
#define INITIAL_REPLY_BUFFER_SIZE 20


RbCtrlIface::RbCtrlIface(int boardIdx, string serialPort,
                         int serialbaudrate, char parity,
                         int data_bit, int stop_bit, bool boardSimulation ) :
    mModbus(NULL),
    mReplyBuffer(NULL),
    mSimulActive(boardSimulation)
{
    mBoardConnected = false;

    mReplyBufSize = INITIAL_REPLY_BUFFER_SIZE;
    mReplyBuffer = new uint16_t[mReplyBufSize];

    mBoardIdx = boardIdx;
    int count = 0;

    if( !mSimulActive )
    {
        // >>>>> MOD_BUS serial communication settings

        ROS_INFO_STREAM( (count+1) << "Initializing connection to RbCtrlIface Id: " << mBoardIdx );

        while( !initializeSerialModbus( serialPort.c_str(),
                                        serialbaudrate, parity, data_bit, stop_bit ) &&
               count < 10 )
        {
            ROS_WARN_STREAM( "Failed to initialize mod_bus on port: " << serialPort );
            ROS_WARN_STREAM( "Trying again in one second...");

            count++;

            ROS_INFO_STREAM( (count+1) << "Initializing connection to RbCtrlIface Id: " << mBoardIdx );
            ros::Duration(1).sleep(); // sleep for a second
        }

        if( count > 10 )
        {
            ROS_FATAL_STREAM("* RbCtrlIface not connected in 10 seconds. Server not started!");

            return;
        }
        // <<<<< MOD_BUS serial communication settings

        // >>>>> Board connection
        bool res = connectModbus( 10 );
        if( !res )
        {
            ROS_FATAL_STREAM( "Failed to connect to modbus on port:" << serialPort );
            ROS_FATAL_STREAM( "Server not started" );

            return;
        }

        ROS_INFO_STREAM( "RbCtrlIface connected" );
        // <<<<< Board connection
    }
    else
    {
        ROS_INFO_STREAM( "### RbCtrlIface is working in simulated mode ###" );

        ros::Duration(0.5).sleep(); // sleep for half a second

    }


}

RbCtrlIface::~RbCtrlIface()
{
    if( mModbus )
    {
        modbus_close( mModbus );
        modbus_free( mModbus );
    }

    if(mReplyBuffer)
        delete [] mReplyBuffer;
}

modbus_t* RbCtrlIface::initializeSerialModbus( const char *device,
                                               int baud, char parity, int data_bit,
                                               int stop_bit )
{
    // >>>>> Simulation?
    if(mSimulActive)
        return NULL;
    // <<<<< Simulation?

    if( mModbus )
    {
        modbus_close( mModbus );
        modbus_free( mModbus );
    }

    mModbus = modbus_new_rtu( device, baud, parity,
                              data_bit, stop_bit );

    return mModbus;
}

bool RbCtrlIface::connectModbus( int retryCount/*=-1*/)
{
    // >>>>> Simulation?
    if(mSimulActive)
    {
        ROS_WARN_STREAM( "ModBus replies are simulated!" );

        ros::Duration(1).sleep(); // sleep for a second
        return true;
    }
    // <<<<< Simulation?

    if( !mModbus )
    {
        ROS_FATAL_STREAM( "ModBus data structure not initialized!" );
        return false;
    }

    // Closing to reset active connections

    //modbus_close( mModbus);

    if( modbus_connect( mModbus ) == -1 )
    {
        ROS_FATAL_STREAM( "Modbus connection failed" );
        mBoardConnected = false;
        return false;
    }

    int res=-1;
    // res = modbus_flush( mModbus );

    ros::Duration(1).sleep(); // sleep for a second

    timeval new_timeout;
    new_timeout.tv_sec = 2;
    new_timeout.tv_usec = 0;
    modbus_set_response_timeout( mModbus, &new_timeout );
    modbus_set_byte_timeout( mModbus, &new_timeout );

    res = modbus_set_slave( mModbus, mBoardIdx );
    if( res != 0 )
    {
        ROS_FATAL_STREAM(  "modbus_set_slave error -> " <<  modbus_strerror( errno ) );

        modbus_flush( mModbus );

        mBoardConnected = false;
        return false;
    }

    int tryCount=0;
    bool ok = false;
    mBoardConnected = true;

    while(1)
    {
        ROS_WARN_STREAM( "- testBoardConnection - Attempt: " << (tryCount+1) );
        ok = testBoardConnection();
        tryCount++;

        if( tryCount==retryCount || ok )
            break;
        else
            ROS_WARN_STREAM( "Trying again...");
    }

    if( !ok )
    {
        ROS_FATAL_STREAM( "Error on modbus: " << modbus_strerror( errno ) );
        mBoardConnected = false;
        return false;
    }

    return true;
}

bool RbCtrlIface::testBoardConnection()
{
    // >>>>> Simulation?
    if(mSimulActive)
    {
        ros::Duration(0.001).sleep(); // sleep for a msec

        mBoardConnected = true;
        return true;
    }
    // <<<<< Simulation?

    uint16_t startAddr = WORD_TEST_BOARD;
    uint16_t nReg = 1;

    vector<uint16_t> reply = readMultiReg( startAddr, nReg );

    if(reply.empty())
    {
        ROS_ERROR_STREAM( "Board Ping failed!!!" );
        ROS_ERROR_STREAM( "modbus_read_input_registers error -> " <<  modbus_strerror( errno )
                          << "[First regAddress: " << WORD_TEST_BOARD << "- #reg: " << nReg <<  "]" );

        mBoardConnected = false;
        return false;
    }

    mBoardConnected = true;
    return true;
}

vector<uint16_t> RbCtrlIface::readMultiReg(uint16_t startAddr, uint16_t nReg)
{
    // >>>>> Simulation?
    if(mSimulActive)
    {
        vector<uint16_t> readRegReply;

        //mBoardMutex.lock();
        {
            // >>>>> Reply buffer resize if needed
            if( nReg > mReplyBufSize )
            {
                mReplyBufSize *= 2;
                delete [] mReplyBuffer;
                mReplyBuffer = new uint16_t[mReplyBufSize];
            }
            // <<<<< Reply buffer resize if needed

            readRegReply.resize( nReg+2 );

            readRegReply[0] = (uint16_t)startAddr;
            readRegReply[1] = (uint16_t)nReg;
            for( int i=0; i<nReg; i++ )
            {
                readRegReply[2+i] = (i+1)*1000;
            }

            //qWarning() << PREFIX << "ModBus replies are simulated!";
            ros::Duration(0.001).sleep(); // sleep for a msec
        }
        //mBoardMutex.unlock();

        return readRegReply;
    }
    // <<<<< Simulation?
    
    vector<uint16_t> readRegReply;

    if( !mBoardConnected )
    {
        ROS_ERROR_STREAM( "RoboController is not connected!" );
        return readRegReply;
    }

    //mBoardMutex.lock();
    {
        // >>>>> Reply buffer resize if needed
        if( nReg > mReplyBufSize )
        {
            mReplyBufSize *= 2;
            delete [] mReplyBuffer;
            mReplyBuffer = new uint16_t[mReplyBufSize];
        }
        // <<<<< Reply buffer resize if needed

        int res = modbus_read_input_registers( mModbus, startAddr, nReg, mReplyBuffer );

        if(res!=nReg)
        {
            ROS_ERROR_STREAM( "modbus_read_input_registers error -> " <<  modbus_strerror( errno )
                              << "[First regAddress: " << startAddr << "- #reg: " << nReg <<  "]" );

            //mBoardMutex.unlock();
            return readRegReply;
        }

        readRegReply.resize( nReg+2 );

        readRegReply[0] = (uint16_t)startAddr;
        readRegReply[1] = (uint16_t)nReg;
        memcpy( (uint16_t*)(readRegReply.data())+2, mReplyBuffer, nReg*sizeof(uint16_t) );
    }
    //mBoardMutex.unlock();

    return readRegReply;
}

bool RbCtrlIface::writeMultiReg( uint16_t startAddr, uint16_t nReg,
                                 vector<uint16_t> vals )
{
    if(mSimulActive)
    {
        //mBoardMutex.lock();
        {
            ros::Duration(0.001).sleep(); // sleep for a msec
        }
        //mBoardMutex.unlock();

        return true;
    }
    
    if( !mBoardConnected )
    {
        ROS_ERROR_STREAM( "RoboController is not connected!" );
        return false;
    }

    //mBoardMutex.lock();
    {
        int res = modbus_write_registers( mModbus, startAddr, nReg, vals.data() );

        if(res!=nReg)
        {
            ROS_ERROR_STREAM( "modbus_write_registers error -> " <<  modbus_strerror( errno )
                              << "[First regAddress: " << startAddr << "- #reg: " << nReg <<  "]" );

            //mBoardMutex.unlock();
            return false;
        }
    }
    //mBoardMutex.unlock();
    return true;
}
