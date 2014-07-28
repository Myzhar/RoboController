#ifndef QROBOTCTRLSERVER_H
#define QROBOTCTRLSERVER_H

#include <QThread>
#include <QUdpSocket>
#include "qrobocontrollerinterface.h"

namespace roboctrl
{

class QRobotCtrlServer : public QThread
{
    Q_OBJECT
public:
    explicit QRobotCtrlServer(QRoboControllerInterface* robocontroller, quint16 listenPort, QObject *parent = 0);
     virtual ~QRobotCtrlServer(); ///< Destructor

private:
    void openUdpControlSession();   ///< Opens UDP Control socket
    void closeUdpControlSession();  ///< Closes UDP Control socket
    void releaseControl();          ///< Release the control of the robot

protected:
    virtual void run() Q_DECL_OVERRIDE;
    virtual void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

protected slots:
    void onUdpCtrlReadyRead(); ///< Called when a new data from UDP Control socket is available

signals:
    void clientGainedControl( QString clientIp ); ///< Client gained the control of the robot
    void clientLostControl( QString clientIp); ///< Client lost the control of the robot
    void clientControlRefused( QString clientIp); ///< Client control of the robot refused because another client is controlling

private:
    QUdpSocket* mUdpCtrlReceiver; ///< UDP Control Socket Listener
    quint16     mUdpServerCtrlListenPort; ///< Port of the UDP Control Server @note The control server receives without replying, the client can control if a motion command is successfull using the Status UDP Socket.

    int         mControlTimeoutTimerId; ///< If a client does not send control command for @ref SVR_CONTROL_UDP_TIMEOUT
    QString     mControllerClientIp; ///< Ip address of the client that took control for driving the robot using @ref getRobotControl function

    QRoboControllerInterface* mRoboController;
};

}

#endif // QROBOTCTRLSERVER_H
