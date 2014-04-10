#include "qwebcamserver.h"
#include <vector>
#include <QDebug>
#include <QByteArray>
#include <QTime>
#include <QCoreApplication>
#include <QTime>

using namespace std;

namespace roboctrl
{

QWebcamServer::QWebcamServer(int camIdx, int sendPort,
                             int listenPort, int udpPacketSize, int maxClientCount,
                             QObject *parent):
    QThread(parent),
    mUdpSocketSender(NULL),
    mUdpSocketReceiver(NULL)
{
    mStopped=true;
    mCamIdx=camIdx;
    mSendPort=sendPort;
    mListenPort=listenPort;
    mMaxPacketSize=udpPacketSize;
    mMaxClientCount=maxClientCount;

    //mClientIpList.push_back( clientIP );

    mUdpSocketSender = new QUdpSocket(this);
    //mUdpSocketSend->bind( mSendPort, QUdpSocket::ShareAddress );

    mUdpSocketReceiver = new QUdpSocket(this);
    mUdpSocketReceiver->bind( mListenPort, QUdpSocket::ShareAddress );

    connect( mUdpSocketReceiver, SIGNAL(readyRead()),
             this, SLOT(onReadyRead() ) );

    if( mCap.open( mCamIdx ) )
    {
        mCap.set( CV_CAP_PROP_FRAME_WIDTH, 640  );
        mCap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

        qDebug() << tr("Camera %1 opened").arg(mCamIdx);
        qDebug() << tr("Server started. Sending on port %1. Listening on port %2")
                    .arg(mSendPort).arg(mListenPort);

        mStopped = false;

        // Starting!
        start();
    }

}

QWebcamServer::~QWebcamServer()
{
    if( !mStopped || this->isRunning() )
        stop();

    qDebug() << tr("Waiting for Webcam server termination...");

    QTime timer;
    timer.start();
    while(isRunning() && timer.elapsed()<2000 );
}

void QWebcamServer::stop()
{
    mStopMutex.lock();
    {
        mStopped = true;
    }
    mStopMutex.unlock();
}

void QWebcamServer::sendFragmentedData( QByteArray data, char fragID )
{
    int infoSize = 9;  // Packet info header size
    int fragDataSize = mMaxPacketSize - infoSize; // Data size in the packet

    int dataSize = data.size();

    int tailSize = dataSize%fragDataSize; // last packet data size
    int numFrag = dataSize/fragDataSize; // packet count

    if(tailSize > 0) // if there is a not complete tail we must send a packet not full
        numFrag++;

    for( int i=0; i<numFrag; i++ )
    {
        QByteArray buffer( mMaxPacketSize, 0 );
        QDataStream stream( &buffer, QIODevice::WriteOnly );
        stream.setVersion( QDataStream::Qt_4_0 );

        stream << (quint8)fragID << (quint16)mMaxPacketSize << (quint16)numFrag << (quint16)tailSize << (quint16)i;

        int startIdx = i*fragDataSize;
        int endIdx = (i==numFrag-1)?(startIdx+tailSize):(startIdx+fragDataSize);
        for( int d=startIdx; d<endIdx; d++ )
        {
            quint8 noise=0;
            quint8 val = (quint8)data[d]+noise;
            stream << val;
        }

        for( int c=0; c<(int)mClientIpList.size(); c++ )
        {
            int res = mUdpSocketSender->writeDatagram( buffer,
                                                       /*QHostAddress::Broadcast*/QHostAddress(mClientIpList[c]),
                                                       mSendPort );
            mUdpSocketSender->flush();
            if( -1==res )
            {
                /*qDebug() << tr("Frame #%3: Missed fragment %1/%2 to Client %4")
                            .arg(i).arg(numFrag).arg((quint8)fragID).arg(mClientIpList[c]);*/
                qDebug() << tr("Frame #%3: Missed fragment %1/%2 to Broadcast")
                                            .arg(i).arg(numFrag).arg((quint8)fragID);
            }
        }
        QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
    }

    //    qDebug() << tr("Sent frame #%1 - size: %2 bytes").arg((int)fragID).arg(data.size() );
    //    qDebug() << tr( "Fragment count: %1 - Tail Size: %2 - Full Data Size: %3")
    //                .arg(numFrag).arg(tailSize).arg(dataSize);
}

void QWebcamServer::run()
{
    vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(75);

    quint8 frameCount = 0;

    qDebug() << tr("Webcam Server Thread started");

    forever
    {
        QTime chrono;
        chrono.start();

        mStopMutex.lock();
        {
            if( mStopped )
            {
                mStopMutex.unlock();
                break;
            }
        }
        mStopMutex.unlock();

        cv::Mat frame;
        vector<uchar> compressed;

        mCap >> frame;
        frameCount++;

        //qDebug() << tr( "Frame %1").arg( frameCount );

        // JPG Compression in memory
        if( !frame.empty() && (mClientIpList.size() > 0 ) )
        {
            cv::imencode( ".jpg", frame, compressed, params );

            // ---> UDP Sending
            QByteArray fullDatagram;
            fullDatagram.setRawData( (char*)compressed.data(), compressed.size() );

            sendFragmentedData( fullDatagram, frameCount );
            // <--- UDP Sending
        }

#ifdef WIN32
        if( !frame.empty() )
        {
            cv::imshow( "Frame", frame );
            //cv::waitKey(1);
            //qDebug() << "frame";
        }
#endif

        QCoreApplication::processEvents( QEventLoop::AllEvents, 50 );

        int wait = 100 - chrono.elapsed(); // 10 fps
        if( wait>0 )
            msleep(wait);

        //qDebug() << QTime::currentTime().toString("hh:mm:ss.zzz") << tr("Wait: %1msec").arg(wait);
    }

    qDebug() << tr("Webcam Server Thread finished");

}

void QWebcamServer::onReadyRead()
{
    if( !mUdpSocketReceiver->hasPendingDatagrams() )
    {
        qDebug() << tr( "No Datagrams" );
        return;
    }

    while(mUdpSocketReceiver->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize( mUdpSocketReceiver->pendingDatagramSize() );

        QDataStream stream( &datagram, QIODevice::ReadOnly );
        stream.setVersion( QDataStream::Qt_4_0 );

        QHostAddress senderIP;
        mUdpSocketReceiver->readDatagram( datagram.data(), datagram.size(),
                                          &senderIP );

        quint8 cmd;
        stream >> cmd;

        switch( cmd )
        {
        case CMD_ADD_CLIENT:
        {
            if( mClientIpList.contains( senderIP.toString() ) )
            {
                mUdpSocketSender->writeDatagram( MSG_CONN_ACCEPTED.toLocal8Bit().constData(), MSG_CONN_ACCEPTED.size(), senderIP, mSendPort );
                mUdpSocketSender->flush();
                qDebug() << tr("Client %1 already connected").arg( senderIP.toString() );
            }
            else if(mClientIpList.size()<mMaxClientCount)
            {
                int res = mUdpSocketSender->writeDatagram( MSG_CONN_ACCEPTED.toLocal8Bit().constData(), MSG_CONN_ACCEPTED.size(), senderIP, mSendPort );
                if(MSG_CONN_ACCEPTED.size()==res)
                {
                    mClientIpList.push_back( senderIP.toString() );
                    qDebug() << tr("Client %1 connected").arg( senderIP.toString() );
                }
                else
                    qDebug() << tr("Unable to accept client %1 - Error: %2").arg( senderIP.toString() )
                                .arg(mUdpSocketSender->errorString() ) ;
            }
            else
            {
                mUdpSocketSender->writeDatagram( MSG_CONN_REFUSED.toLocal8Bit().constData(), MSG_CONN_REFUSED.size(), senderIP, mSendPort );
                mUdpSocketSender->flush();
                qDebug() << tr("Client %1 refused, too much client connected").arg( senderIP.toString() );
            }
        }
            break;

        case CMD_REMOVE_CLIENT:
        {
            mClientIpList.removeAll( senderIP.toString() );
            qDebug() << tr("Client %1 disconnected").arg( senderIP.toString() );
        }
            break;

        default:
            qDebug() << tr("Command not recognized: %1").arg( (int)cmd );
        }
    }
}

}
