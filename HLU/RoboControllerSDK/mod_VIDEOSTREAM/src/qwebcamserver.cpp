#include "qwebcamserver.h"
#include <vector>
#include <QDebug>
#include <QByteArray>
#include <QTime>
#include <QCoreApplication>
#include <QTime>
#include <loghandler.h>

using namespace std;
#define MAX_FPS 25.0

namespace roboctrl
{

QWebcamServer::QWebcamServer(int camIdx, int sendPort,
                             int listenPort, int udpPacketSize, int maxClientCount,
                             QObject *parent):
    QThread(parent),
    mUdpSocketSender(NULL),
    mUdpSocketReceiver(NULL)
{
    qDebug() << " ";
    qDebug() << " Robot Webcam Server ";
    qDebug() << "=====================";
    qDebug() << " ";

    mStopped=true;
    mCamIdx=camIdx;
    mSendPort=sendPort;
    mListenPort=listenPort;
    mMaxPacketSize=udpPacketSize;
    mMaxClientCount=maxClientCount;

    mWebcamConnected = false;

    QImage img( ":/img/MyzharBot_DEFAULT.jpg" );
    mDefImage = img.convertToFormat( QImage::Format_RGB888 );
    mDefImage = mDefImage.rgbSwapped();

    mFps = MAX_FPS;

    // Starting!
    start();
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

bool QWebcamServer::connectWebcam()
{
    static bool infoOut = false;
    if( mCap.open( mCamIdx ) )
    {
        mCap.set( CV_CAP_PROP_FRAME_WIDTH, 640  );
        mCap.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

        qDebug() << tr("Camera %1 opened").arg(mCamIdx);
        qDebug() << tr("Server started. Sending on port %1. Listening on port %2")
                    .arg(mSendPort).arg(mListenPort);

        mStopped = false;
        infoOut = false;

        return true;
    }
    else
    {
        if( !infoOut )
        {
            QString err = tr("Webcam not found (%1). Please connect a Webcam to USB").arg(mCamIdx);
            qWarning() << err;

            infoOut = true;
        }

        return false;
    }
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
        stream.setVersion( QDataStream::Qt_5_3 );

        stream << (quint8)fragID << (quint16)mMaxPacketSize << (quint16)numFrag << (quint16)tailSize << (quint16)i;

        int startIdx = i*fragDataSize;
        int endIdx = (i==numFrag-1)?(startIdx+tailSize):(startIdx+fragDataSize);
        for( int d=startIdx; d<endIdx; d++ )
        {
            stream << (quint8)data[d];
        }

        int res = mUdpSocketSender->writeDatagram( buffer, QHostAddress(MULTICAST_WEBCAM_SERVER_IP), mSendPort );
        mUdpSocketSender->flush();

        if( -1==res )
        {
            qDebug() << tr("Frame #%3: Missed fragment %1/%2 to Broadcast. Error: %1")
                        .arg(i).arg(numFrag).arg((quint8)fragID).arg(mUdpSocketSender->errorString());
        }

        usleep(1);
        //msleep(1);
    }

    //    qDebug() << tr("Sent frame #%1 - size: %2 bytes").arg((int)fragID).arg(data.size() );
    //    qDebug() << tr( "Fragment count: %1 - Tail Size: %2 - Full Data Size: %3")
    //                .arg(numFrag).arg(tailSize).arg(dataSize);
}

void QWebcamServer::run()
{    
    qDebug() << tr("Webcam Server Thread started");

    // >>>>> Frame compression parameters
    vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(85);
    // <<<<< Frame compression parameters

    quint8 frameCount = 0;

    // >>>>> Thread Initialization
    mUdpSocketSender = new QUdpSocket();
    // Multicast on the subnet
    mUdpSocketSender->setSocketOption( QAbstractSocket::MulticastTtlOption, 1 );

    mUdpSocketReceiver = new QUdpSocket();
    mUdpSocketReceiver->bind( mListenPort, QUdpSocket::ShareAddress );

    connect( mUdpSocketReceiver, SIGNAL(readyRead()),
             this, SLOT(onReadyRead() ) );

    mStopped = false;
    // <<<<< Thread Initialization

    cv::Mat frame;
    vector<uchar> compressed;

    setPriority( QThread::TimeCriticalPriority );

    // >>>>> Thread Loop
    forever
    {
        // Waiting between loops according to FPS
        double sleepMsec = 1000.0/((double)mFps);

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

        if( !mWebcamConnected )
            mWebcamConnected = connectWebcam();

        if(mWebcamConnected)
            mCap >> frame;
        else
        {
            int h = mDefImage.height();
            int w = mDefImage.width();

            cv::Mat img( h, w, CV_8UC3, mDefImage.bits() );
            frame = img.clone();
        }

        frameCount++;

        //qDebug() << tr( "Frame %1").arg( frameCount );

#ifdef WIN32
        if( !frame.empty() )
        {
            cv::imshow( "Frame", frame );
        }
#endif

        //QTime chrono_elab;
        //chrono_elab.start();
        // JPG Compression in memory
        if( !frame.empty() )
        {
            cv::imencode( ".jpg", frame, compressed, params );

            // >>>>> UDP Sending
            QByteArray fullDatagram;
            fullDatagram.setRawData( (char*)compressed.data(), compressed.size() );

            sendFragmentedData( fullDatagram, frameCount );
            // <<<<< UDP Sending
        }
        //qDebug() << tr("Elaboration time: %1 msec").arg(chrono_elab.elapsed());

        QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );

        int elapsed = chrono.elapsed();
        //qDebug() << tr("Elapsed: %1 msec").arg(elapsed);

        int sleep = sleepMsec - elapsed;

       // >>>>> FPS auto-tuning
        // Note FPS are tuned such to give to client at 10msec between
        // two consecutive frames
        if( sleep<0 )
        {
            mFps -= 1.0;
            qDebug() << PREFIX << tr("New FPS %1 (Sleep: %2)").arg(mFps).arg(sleep);
        }
        else if( sleep<1 )
        {
            mFps -= 0.05;
            qDebug() << PREFIX << tr("New FPS %1 (Sleep: %2)").arg(mFps).arg(sleep);
        }
        else if( sleep>3 )
        {
            mFps += 0.05;
            if( mFps>MAX_FPS )
                mFps = MAX_FPS;
            qDebug() << PREFIX << tr("New FPS %1 (Sleep: %2)").arg(mFps).arg(sleep);
        }
        // <<<<< FPS auto-tuning

        if( sleep>0 )
            msleep(sleep);


        //qDebug() << QTime::currentTime().toString("hh:mm:ss.zzz") << tr("Wait: %1msec").arg(wait);
    }
    // <<<<< Thread Loop

    // >>>>> Thread deinitialization
    disconnect( mUdpSocketReceiver, SIGNAL(readyRead()),
                this, SLOT(onReadyRead() ) );

    if(mUdpSocketSender)
        delete mUdpSocketSender;
    mUdpSocketSender = NULL;

    if(mUdpSocketReceiver)
        delete mUdpSocketReceiver;
    mUdpSocketReceiver = NULL;
    // <<<<< Thread deinitialization

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
        stream.setVersion( QDataStream::Qt_5_3 );

        QHostAddress senderIP;
        mUdpSocketReceiver->readDatagram( datagram.data(), datagram.size(),
                                          &senderIP );

        quint8 cmd;
        stream >> cmd;

        switch( cmd )
        {

        // TODO: Process webcam commands

        // TODO: Create command to set FPS

        default:
            qDebug() << tr("Command not recognized: %1").arg( (int)cmd );
        }
    }
}

}
