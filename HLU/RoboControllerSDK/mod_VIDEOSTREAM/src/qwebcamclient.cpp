#include "qwebcamclient.h"
#include "RoboControllerSDK_global.h"

#include <vector>
#include <QCoreApplication>
#include <loghandler.h>


using namespace std;

namespace roboctrl
{

QWebcamClient::QWebcamClient( int listenPort, int sendPort, QObject *parent) :
    QThread(parent),
    mUdpSocketSend(NULL)
{
    mCurrentId = 0;
    mCurrentFragmentCount = 0;

    mFrmBufIdx = 0;

    mLastImageState=true;
    mListenPort = listenPort;
    mSendPort = sendPort;
    mServerIp = QString(MULTICAST_WEBCAM_SERVER_IP);

    mFrmTripleBuf.resize(3);
    mRecTripleBuffer.resize(3);

    qRegisterMetaType<cv::Mat>("cv::Mat");

    start();
}

QWebcamClient::~QWebcamClient()
{    
    mStopped = true;
    this->terminate();
    this->wait( 1000 );
}

void QWebcamClient::disconnectServer()
{

    if(mUdpSocketSend)
    {
        delete mUdpSocketSend;
        mUdpSocketSend = NULL;
    }

    if(mUdpSocketListen)
    {
        mUdpSocketListen->leaveMulticastGroup( QHostAddress(MULTICAST_WEBCAM_SERVER_IP) );
        delete mUdpSocketListen;
        mUdpSocketListen = false;
    }

    mConnected = false;
}

bool QWebcamClient::connectToServer(int sendPort,int listenPort)
{
    mListenPort=listenPort;
    mSendPort=sendPort;

    if(mUdpSocketSend)
    {
        mUdpSocketSend->abort();
        delete mUdpSocketSend;
    }

    mUdpSocketSend = new QUdpSocket();

    mUdpSocketListen = new QUdpSocket();

    // To connect to multicast server we need that the client socket is binded
    bool binded = mUdpSocketListen->bind( QHostAddress::AnyIPv4,
                                          mListenPort,
                                          QUdpSocket::ShareAddress|QUdpSocket::ReuseAddressHint );

    // >>>> Trying connection
    if(!binded)
    {
        qDebug() << tr("Server %1 not binded on port %2")
                    .arg(mServerIp).arg(mListenPort);
        return false;
    }
    else
    {
        if( !mUdpSocketListen->joinMulticastGroup( QHostAddress(MULTICAST_WEBCAM_SERVER_IP) ) )
        {
            qDebug() << tr("Connection to Multicast server failed on port %2")
                        .arg(mSendPort);
            mConnected = false;
        }
        else
        {
            qDebug() << tr("Connected to Multicast Webcam Server. Listening on port %1. Sending on port %2")
                        .arg(mListenPort).arg(mSendPort);
            mConnected = true;

            /*connect(mUdpSocketListen, SIGNAL(readyRead()),
                    this, SLOT(onReadyRead()) );*/
        }
    }
    // <<<< Trying connection

    mFrameReceived = 0;
    mFrameComplete = 0;

    mStopped = false;

    return mConnected;
}

void QWebcamClient::run()
{
    connectToServer( mSendPort, mListenPort );

    qDebug() << tr("Webcam Client Thread started");

    forever
    {
        if( mStopped )
            break;

        if( mUdpSocketListen->hasPendingDatagrams() )
        {
            QByteArray datagram;

            qint64 datagramSize = mUdpSocketListen->pendingDatagramSize();

            datagram.resize( datagramSize );

            QDataStream stream( &datagram, QIODevice::ReadOnly );
            stream.setVersion( QDataStream::Qt_5_3 );

            mUdpSocketListen->readDatagram( datagram.data(), datagram.size() );

            processDatagram( datagram );
        }
    }

    disconnectServer();

    qDebug() << tr("Webcam Client Thread finished");
}

void QWebcamClient::onReadyRead()
{
    while(mUdpSocketListen->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize( mUdpSocketListen->pendingDatagramSize() );



        QHostAddress serverAddr;
        mUdpSocketListen->readDatagram( datagram.data(), datagram.size() );

        processDatagram( datagram );
    }
}

void QWebcamClient::processDatagram( QByteArray& datagram )
{
    QDataStream stream( &datagram, QIODevice::ReadOnly );
    stream.setVersion( QDataStream::Qt_5_3 );

    quint8 id;
    stream >> id;

    int infoSize = 9; // Packet info header size

    if( id != mCurrentId )
    {
        if(!mLastImageState)
            qDebug() << PREFIX << tr("Id received: %4 - Frame #%1 lost. Received %2/%3 fragments").arg(mCurrentId).arg(mCurrentFragmentCount).arg(mNumFrag).arg(id);

        mLastImageState = false;

        mFrameReceived++;

        mCurrentId = id;

        //qDebug() << tr( "New frame id: %1").arg(mCurrentId);

        stream >> mPacketSize;
        stream >> mNumFrag;

        //qDebug() << tr(" Received frag #%1").arg((unsigned int)mNumFrag);

        stream >> mTailSize;

        int dataSize;
        if( mTailSize==0 )
            dataSize = mNumFrag*mPacketSize;
        else
            dataSize = (mNumFrag-1)*(mPacketSize-infoSize)+mTailSize;

        mCurrentFragmentCount = 0;
        mRecTripleBuffer[mFrmBufIdx].clear();
        mRecTripleBuffer[mFrmBufIdx].resize(dataSize);

        //            qDebug() << tr( "Fragment count: %1 - Tail Size: %2 - Full Data Size: %3")
        //                        .arg(mNumFrag).arg(mTailSize).arg(dataSize);
    }
    else
    {
        // >>>> Removing info data from stream and verifying of correctness
        quint16 packetSize;
        stream >> packetSize;
        quint16 numFrag;
        stream >> numFrag;
        quint16 tailSize;
        stream >> tailSize;

        if( packetSize!=mPacketSize || numFrag!=mNumFrag || tailSize!=mTailSize )
        {
            qDebug() << tr( "Packet error" );
            return;
        }
        // <<<< Removing info data from stream and verifying of correctness
    }

    quint16 fragIdx;
    stream >> fragIdx;
    int fragDataSize = mPacketSize-infoSize;

    int startIdx = fragIdx*fragDataSize;
    int endIdx = (fragIdx==mNumFrag-1)?(startIdx+mTailSize):(startIdx+fragDataSize);

    for( int d=startIdx; d<endIdx; d++ )
    {
        quint8 elem;
        stream >> elem;

        mRecTripleBuffer[mFrmBufIdx][d] = elem;
    }

    mCurrentFragmentCount++;

    if(mCurrentFragmentCount==mNumFrag) // Image is ready
    {
        mLastImageState = true;
        mFrmTripleBuf[mFrmBufIdx] = cv::imdecode( cv::Mat( mRecTripleBuffer[mFrmBufIdx] ), 1 );

        if(!mFrmTripleBuf[mFrmBufIdx].empty())
        {
            emit newImageReceived( mFrmTripleBuf[mFrmBufIdx] );
            emit newImageReceived( );

            mFrameComplete++;
            mFrmBufIdx = (++mFrmBufIdx)%3;

            //cv::imshow( "Stream", mLastCompleteFrame );


            //cv::waitKey( 1 );
            //QCoreApplication::processEvents( QEventLoop::AllEvents, 50 );
            //qDebug() << tr( "Frame #%1 ready" ).arg((int)id);
        }
        else
            qDebug() << tr( "Frame #%1 error: Wrong encoding" ).arg((int)id);
    }
}

cv::Mat QWebcamClient::getLastFrame()
{
    //qDebug() << Q_FUNC_INFO;

    int idx = mFrmBufIdx-1;
    if(idx<0)
        idx = 2;
    return mFrmTripleBuf[idx];
}

}
