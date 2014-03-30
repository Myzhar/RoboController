#ifndef QWEBCAMSERVER_H
#define QWEBCAMSERVER_H

#include <QThread>
#include <QUdpSocket>
#include <QString>
#include <QStringList>
#include <QTimerEvent>
#include <QMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// ---> Server Command
#define CMD_ADD_CLIENT      0
#define CMD_REMOVE_CLIENT   1

#define MSG_CONN_REFUSED  tr("@R")
#define MSG_CONN_ACCEPTED tr("@A")
// <--- Server Command

namespace roboctrl
{

class 	QWebcamServer : public QThread
{
    Q_OBJECT
public:
    explicit QWebcamServer( int camIdx=0,
                            int sendPort=55554,
                            int listenPort=55555,
                            int udpPacketSize=4096,
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

private:
    int mCamIdx;
    int mSendPort;
    int mListenPort;

    cv::VideoCapture mCap;    

    int mMaxPacketSize;
    int mMaxClientCount;

    QUdpSocket *mUdpSocketSender;
    QUdpSocket *mUdpSocketReceiver;

    QStringList mClientIpList;

    QMutex mStopMutex;
    bool mStopped;
};

}

#endif // QWEBCAMSERVER_H
