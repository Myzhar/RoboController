#ifndef QRGBDSERVER_H
#define QRGBDSERVER_H

#include <QThread>
#include <QMutex>
#include <QUdpSocket>
#include <RoboControllerSDK_global.h>
#include <qopenni2grabber.h>
#include <opencvtools.h>

using namespace std;

#define MAX_FPS 30

namespace roboctrl
{

class QRgbdServer : public QThread
{
    Q_OBJECT
public:
    explicit QRgbdServer( int sendPort=55554,
                          int listenPort=55555,
                          int udpPacketSize=1500,
                          QObject *parent = 0 );
    virtual ~QRgbdServer();

    void stop();

signals:

protected slots:
    void onReadyRead();
    void onUdpError();

    void onNewColorImage(cv::Mat rgb);
    void onNew2dMap(Q2dMap*);

protected:
    void sendFragmentedData( QByteArray data, char fragID );
    void run();

    bool connectSensor();

private:
    int mSendPort;
    int mListenPort;

    int mMaxPacketSize;
    int mMaxClientCount;

    QUdpSocket *mUdpSocketSender;
    QUdpSocket *mUdpSocketReceiver;

    //QStringList mClientIpList;

    QMutex mStopMutex;
    bool mStopped;

    bool mSensorConnected;

    double mFps;

    QImage mDefImage;

    QOpenNI2Grabber* mRgbdGrabber;

    QMutex mRgbMutex;
    cv::Mat mLastRgb;
    Q2dMap* mLastMap;

    bool mRgbValid;
    bool mMapValid;
};

}

#endif // QRGBDSERVER_H
