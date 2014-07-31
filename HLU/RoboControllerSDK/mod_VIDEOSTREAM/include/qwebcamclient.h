#ifndef QWEBCAMCLIENT_H
#define QWEBCAMCLIENT_H

#include <QThread>
#include <QUdpSocket>
#include <QMutex>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

    cv::Mat getLastFrame();
    void getStats( quint64& frmCount, quint64& frmComplete){frmCount=mFrameReceived;frmComplete=mFrameComplete;}

protected:
    void run() Q_DECL_OVERRIDE;
    void processDatagram(QByteArray &datagram);

    bool connectToServer(int sendPort, int listenPort);
    void disconnectServer();

signals:
    void newImageReceived();

protected slots:
    void onReadyRead();


private:
    QUdpSocket *mUdpSocketSend;
    QUdpSocket *mUdpSocketListen;
    QString mServerIp;
    int mListenPort;
    int mSendPort;

    bool mConnected;
    bool mStopped;

    quint8 mCurrentId;
    int mCurrentFragmentCount;
    vector< vector<uchar> > mRecTripleBuffer;

    quint16 mPacketSize;
    quint16 mNumFrag;
    quint16 mTailSize;

    vector<cv::Mat> mFrmTripleBuf;
    int mFrmBufIdx;

    bool mLastImageState;

    quint64 mFrameReceived;
    quint64 mFrameComplete;
};

}

#endif // QWEBCAMCLIENT_H
