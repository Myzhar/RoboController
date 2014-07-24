#include "qwebcamclient.h"
#include "RoboControllerSDK_global.h"

#include <vector>
#include <QCoreApplication>


using namespace std;

namespace roboctrl
{

QWebcamClient::QWebcamClient( int listenPort, int sendPort, QObject *parent) :
    QThread(parent),
    mUdpSocketSend(NULL)
{
    mCurrentId = 0;
    mCurrentFragmentCount = 0;

    mLastImageState=true;
    mListenPort = listenPort;
    mSendPort = sendPort;
    mServerIp = QString(MULTICAST_WEBCAM_SERVER_IP);

    connectToServer( sendPort, listenPort );
}

QWebcamClient::~QWebcamClient()
{
//    if(mUdpSocketSend)
//    {
//        /*char cmd = CMD_REMOVE_CLIENT;
//        mUdpSocketSend->writeDatagram( &cmd,
//                                       QHostAddress(mServerIp),
//                                       mSendPort );
//        mUdpSocketSend->flush();*/


//        delete mUdpSocketSend;
//    }

//    if(mUdpSocketListen)
//    {
//        mUdpSocketListen->leaveMulticastGroup( QHostAddress(MULTICAST_WEBCAM_SERVER_IP) );
//        delete mUdpSocketListen;
//    }

//    mConnected = false;
//
    disconnectServer();
}

void QWebcamClient::disconnectServer()
{

    if( this->isRunning() )
    {
        this->terminate();
        this->wait( 1000 );
    }

    if(mUdpSocketSend)
    {
        /*char cmd = CMD_REMOVE_CLIENT;
        mUdpSocketSend->writeDatagram( &cmd,
                                       QHostAddress(mServerIp),
                                       mListenPort );
        mUdpSocketSend->flush();*/

        delete mUdpSocketSend;
        mUdpSocketSend = NULL;
    }

    if(mUdpSocketListen)
    {
        mUdpSocketListen->leaveMulticastGroup( QHostAddress(MULTICAST_WEBCAM_SERVER_IP) );
        delete mUdpSocketListen;
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
                                            QUdpSocket::ShareAddress );

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
            qDebug() << tr("Connected to Multicast server. Listening on port %1. Sending on port %2")
                        .arg(mListenPort).arg(mSendPort);
            mConnected = true;

            connect(mUdpSocketListen, SIGNAL(readyRead()),
                    this, SLOT(processPendingDatagrams()));
        }
    }
    // <<<< Trying connection

    if(mConnected)
        start();

    return mConnected;
}

void QWebcamClient::processPendingDatagrams()
{
    while(mUdpSocketListen->hasPendingDatagrams())
    {
        QCoreApplication::processEvents( QEventLoop::AllEvents, 5 );

        QByteArray datagram;
        datagram.resize( mUdpSocketListen->pendingDatagramSize() );

        QDataStream stream( &datagram, QIODevice::ReadOnly );
        stream.setVersion( QDataStream::Qt_4_0 );

        QHostAddress serverAddr;
        mUdpSocketListen->readDatagram( datagram.data(), datagram.size(),
                                        &serverAddr );

        mServerIp = serverAddr.toString();
        //qDebug() << mServerIp;

        quint8 id;
        stream >> id;

        int infoSize = 9; // Packet info header size

        if( id != mCurrentId )
        {
            if(!mLastImageState)
                qDebug() << tr("Frame #%1 lost").arg(mCurrentId);
            mLastImageState = false;

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
            mCurrentBuffer.clear();
            mCurrentBuffer.resize(dataSize);

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
                continue;
                //return;
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

            mCurrentBuffer[d] = elem;
        }

        mCurrentFragmentCount++;

        if(mCurrentFragmentCount==mNumFrag) // Image is ready
        {
            mImgMutex.lock();
            {
                mLastImageState = true;
                mLastCompleteFrame = cv::imdecode( cv::Mat( mCurrentBuffer ), 1 );
            }
            mImgMutex.unlock();

            if(!mLastCompleteFrame.empty())
            {
                //cv::imshow( "Stream", mLastCompleteFrame );
                emit newImageReceived();

                //cv::waitKey( 1 );
                //QCoreApplication::processEvents( QEventLoop::AllEvents, 50 );
                //qDebug() << tr( "Frame #%1 ready" ).arg((int)id);
            }
            else
                qDebug() << tr( "Frame #%1 error: Wrong encoding" ).arg((int)id);
        }
    }
}

cv::Mat QWebcamClient::getLastFrame()
{
    //qDebug() << Q_FUNC_INFO;

    //mImgMutex.lock();
    {
        return mLastCompleteFrame;
    }
    //mImgMutex.unlock();
}

}
