#ifndef QWEBCAMCLIENT_H
#define QWEBCAMCLIENT_H

#include <QThread>
#include <QUdpSocket>
#include <QMutex>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// >>>> Server Command
#define CMD_ADD_CLIENT      ((quint8)0)
#define CMD_REMOVE_CLIENT   ((quint8)1)
// <<<< Server Command

using namespace std;

namespace roboctrl
{

class QWebcamClient : public QThread
{
    Q_OBJECT
public:
    explicit QWebcamClient(int listenPort=55554,
                           int sendPort=55555,
                           QObject *parent = 0);
    virtual ~QWebcamClient();

    bool isConnected(){return mConnected;}

    bool connectToServer(int sendPort, int listenPort);
    void disconnectServer();

    cv::Mat getLastFrame();

signals:
    void newImageReceived();

public slots:
    void processPendingDatagrams();

private:
    QUdpSocket *mUdpSocketSend;
    QUdpSocket *mUdpSocketListen;
    QString mServerIp;
    int mListenPort;
    int mSendPort;

    bool mConnected;

    quint8 mCurrentId;
    int mCurrentFragmentCount;
    vector<uchar> mCurrentBuffer;

    quint16 mPacketSize;
    quint16 mNumFrag;
    quint16 mTailSize;

    QMutex mImgMutex;
    cv::Mat mLastCompleteFrame;
    bool mLastImageState;
};

}

#endif // QWEBCAMCLIENT_H
