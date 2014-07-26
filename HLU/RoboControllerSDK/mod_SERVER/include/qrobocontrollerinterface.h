#ifndef QROBOCONTROLLERINTERFACE_H
#define QROBOCONTROLLERINTERFACE_H

#include <QObject>
#include <modbus.h>
#include <QVector>

namespace roboctrl
{

class QRoboControllerInterface : public QObject
{
    Q_OBJECT

public:
    explicit QRoboControllerInterface(int boardIdx, QString serialPort,
                                      int serialbaudrate, char parity,
                                      int data_bit, int stop_bit, bool simulMode = false,
                                      QObject *parent = 0);
    virtual ~QRoboControllerInterface();

public:
    modbus_t* initializeSerialModbus( const char *device,
                                      int baud, char parity, int data_bit,
                                      int stop_bit ); ///< Initializes Modbus Serial Connection to RoboController

    bool connectModbus( int retryCount=-1); ///< retryCount=-1 puts the server in an infinite loop trying reconnection
    bool testBoardConnection(); ///< Tests if the board has not been disconnected


    QVector<quint16> readMultiReg(quint16 startAddr, quint16 nReg ); ///< Called to read registers from RoboController

    // QVector used instead of QList to provide direct data access using "Data()" function
    bool writeMultiReg( quint16 startAddr, quint16 nReg, QVector<quint16> vals ); ///< Called to write registers to RoboController

    inline bool isConnected(){return mBoardConnected;} ///< Returns true if the board is connected

signals:

public slots:

private:
    QMutex  	    mBoardMutex; ///< Mutex on Robocontroller board

    modbus_t*       mModbus;  ///< ModBus protocol implementation
    quint16         mBoardIdx;      /// Id of the connected board

    unsigned int    mReplyBufSize; ///< current size of the reply buffer
    quint16*        mReplyBuffer;  ///< dinamic reply buffer (resized only if necessary)

    bool            mBoardConnected; ///< Indicates if the RoboController board is connected

    bool            mSimulActive; ///< Indicates if RoboController is simulated
};

}

#endif // QROBOCONTROLLERINTERFACE_H
