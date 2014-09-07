#ifndef _RbCtrlIface_NODE_H_
#define _RbCtrlIface_NODE_H_

#include <modbus/modbus.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;

class RbCtrlIface 
{

public:
    RbCtrlIface( int boardIdx, string serialPort,
                              int serialbaudrate, char parity,
                              int data_bit, int stop_bit, bool simulMode = false );
    virtual ~RbCtrlIface();

public:
    modbus_t* initializeSerialModbus( const char *device,
                                      int baud, char parity, int data_bit,
                                      int stop_bit ); ///< Initializes Modbus Serial Connection to RbCtrlIface

    bool connectModbus( int retryCount=-1); ///< retryCount=-1 puts the server in an infinite loop trying reconnection
    bool testBoardConnection(); ///< Tests if the board has not been disconnected

    vector<uint16_t> readMultiReg(uint16_t startAddr, uint16_t nReg ); ///< Called to read registers from RbCtrlIface

    // Qvector used instead of QList to provide direct data access using "Data()" function
    bool writeMultiReg( uint16_t startAddr, uint16_t nReg, vector<uint16_t> vals ); ///< Called to write registers to RbCtrlIface

    inline bool isConnected(){return mBoardConnected;} ///< Returns true if the board is connected

private:
    modbus_t*       mModbus;  ///< ModBus protocol implementation
    uint16_t          mBoardIdx;      /// Id of the connected board

    uint16_t          mReplyBufSize; ///< current size of the reply buffer
    uint16_t*         mReplyBuffer;  ///< dinamic reply buffer (resized only if necessary)

    bool            mBoardConnected; ///< Indicates if the RbCtrlIface board is connected

    bool            mSimulActive; ///< Indicates if RbCtrlIface is simulated
};

#endif
