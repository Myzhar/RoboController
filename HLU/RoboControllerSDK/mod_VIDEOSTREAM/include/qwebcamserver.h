#ifndef QWEBCAMSERVER_H
#define QWEBCAMSERVER_H

#include <QThread>
#include <QUdpSocket>
#include <QString>
#include <QStringList>
#include <QTimerEvent>
#include <QMutex>
#include <QImage>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "RoboControllerSDK_global.h"

//#define DEFAULT_IMAGE "MyzharBot_DEFAULT.png"
#define DEFAULT_IMAGE "MyzharBot_DEFAULT.jpg"

using namespace std;

namespace roboctrl
{

class 	QWebcamServer : public QThread
{
    Q_OBJECT
public:
    explicit QWebcamServer( int camIdx=-1,
                            int sendPort=55554,
                            int listenPort=55555,
                            int udpPacketSize=1500,
                            int maxClientCount=5,
                            QObject *parent = 0);

    virtual ~QWebcamServer();
    
    void stop();

signals:
    
protected slots:
    void onReadyRead();

protected:
    void sendFragmentedData( QByteArray data, char fragID );
    void run();

    bool connectWebcam();

private:
    int mCamIdx;
    int mSendPort;
    int mListenPort;

    cv::VideoCapture mCap;    

    int mMaxPacketSize;
    int mMaxClientCount;

    QUdpSocket *mUdpSocketSender;
    QUdpSocket *mUdpSocketReceiver;

    //QStringList mClientIpList;

    QMutex mStopMutex;
    bool mStopped;

    bool mWebcamConnected;

    quint8 mFps;

    QImage mDefImage;
};

}

#endif // QWEBCAMSERVER_H
