#ifndef QROBOTTELEMETRYSERVER_H
#define QROBOTTELEMETRYSERVER_H

#include <QThread>
#include <QUdpSocket>
#include "RoboControllerSDK_global.h"
#include <QTimer>
#include "qrobocontrollerinterface.h"

namespace roboctrl
{

class QRobotTelemetryServer : public QThread
{
    Q_OBJECT
public:
    explicit QRobotTelemetryServer( QRoboControllerInterface* robocontroller, quint16 sendPort=14565,  QObject *parent = 0);
    virtual ~QRobotTelemetryServer();

protected:
    void run() Q_DECL_OVERRIDE;

signals:

protected slots:
    void onUpdateTimerTimeout();

public slots:
    void setCtrlIP( QString clientIP );

protected:
    void multicastSendTelemetry(); ///< Send telemetry in multicast
    bool updateTelemetry(); ///< Called to update telemetry from RoboController

    void openUdpServerSession();
    void closeUdpServerSession();


private:
    QUdpSocket*     mUdpMulticastTelemetryServer; ///< Multicast Telemetry Socket

    RobotTelemetry  mTelemetry; ///< Telemetry of the robot, updated every @ref TELEMETRY_UPDATE_MSEC msec

    quint16 mMulticastUdpTelemetryServerPort;

    QTimer* mUpdateTimer;

    QRoboControllerInterface* mRoboController;
};

}

#endif // QROBOTTELEMETRYSERVER_H
