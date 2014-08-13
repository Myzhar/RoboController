#include <qrgbdserver.h>
#include <QCoreApplication>
#include <QTime>
#include <loghandler.h>

#define MIN_FPS 5

namespace roboctrl
{

QRgbdServer::QRgbdServer(int sendPort, int listenPort,
                         int udpPacketSize, QObject *parent) :
    QThread(parent),
    mRgbdGrabber(NULL)
{
    qDebug() << " ";
    qDebug() << " Robot RGB-D Server ";
    qDebug() << "=====================";
    qDebug() << " ";

    mStopped=true;
    mSendPort=sendPort;
    mListenPort=listenPort;
    mMaxPacketSize=udpPacketSize;

    mSensorConnected = false;

    QImage img( ":/img/MyzharBot_DEFAULT.jpg" );
    mDefImage = img.convertToFormat( QImage::Format_RGB888 );
    mDefImage = mDefImage.rgbSwapped();

    mFps = MAX_FPS;

    // Starting!
    start();
}

QRgbdServer::~QRgbdServer()
{
    if( !mStopped || this->isRunning() )
        stop();

    qDebug() << tr("Waiting for RGB-D server termination...");

    wait(2000);
}

void QRgbdServer::stop()
{
    if(mRgbdGrabber)
        delete mRgbdGrabber;
    mRgbdGrabber = NULL;

    mStopMutex.lock();
    {
        mStopped = true;
    }
    mStopMutex.unlock();
}

bool QRgbdServer::connectSensor()
{
    mRgbValid = false;
    mMapValid = false;

    mRgbdGrabber = new QOpenNI2Grabber();

    sleep(1);

    if( !mRgbdGrabber->isRunning() )
        return false;

    connect( mRgbdGrabber, SIGNAL(newBgrImageAvailable(cv::Mat)),
             this, SLOT(onNewColorImage(cv::Mat)) );
    connect( mRgbdGrabber, SIGNAL(new2dMat(Q2dMap*)),
             this, SLOT(onNew2dMap(Q2dMap*)) );

    mSensorConnected = true;
    mStopped = false;

    qDebug() << "Connected";


    return true;
}

void QRgbdServer::onNew2dMap( Q2dMap* map)
{
    mLastMap = map;
    mMapValid = true;
}

void QRgbdServer::onNewColorImage(cv::Mat rgb)
{
    mRgbMutex.lock();
    {
        rgb.copyTo(mLastRgb);
        mRgbValid = true;
    }
    mRgbMutex.unlock();
}

void QRgbdServer::sendFragmentedData( QByteArray data, char fragID )
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

void QRgbdServer::run()
{
    qDebug() << tr("RGB-D Server Thread started");

    // >>>>> RGB Frame compression parameters
    vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(85);
    // <<<<< RGB Frame compression parameters

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

    qDebug() << "Starting";

    // >>>>> Thread Loop
    forever
    {
        mStopMutex.lock();
        {
            if( mStopped )
            {
                mStopMutex.unlock();
                break;
            }
        }
        mStopMutex.unlock();

        if( !mSensorConnected )
            mSensorConnected = connectSensor();

        if(mSensorConnected )
        {
            if(mRgbValid)
            {
                mRgbMutex.lock();
                {
                    frame = mLastRgb.clone();
                    mRgbValid = false;
                }
                mRgbMutex.unlock();
            }
            else
                continue;
        }
        else
        {
            int h = mDefImage.height();
            int w = mDefImage.width();

            cv::Mat img( h, w, CV_8UC3, mDefImage.bits() );
            frame = img.clone();
        }

        frameCount++;

        // JPG Compression in memory
        if( !frame.empty() )
        {

#ifdef WIN32
            cv::imshow( "Frame", frame );
            //cv::waitKey();
#endif
            //qDebug() << tr( "Frame %1 - %2x%3x%4").arg( frameCount ).arg(frame.cols).arg(frame.rows).arg(frame.channels());

            cv::imencode( ".jpg", frame, compressed, params );

            // >>>>> UDP Sending
            QByteArray fullDatagram;
            fullDatagram.setRawData( (char*)compressed.data(), compressed.size() );

            sendFragmentedData( fullDatagram, frameCount );
            // <<<<< UDP Sending
        }

        QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );
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

    qDebug() << tr("RGB-D Server Thread finished");
}

void QRgbdServer::onUdpError()
{
    if( mUdpSocketSender->error() == QUdpSocket::DatagramTooLargeError )
    {
        mMaxPacketSize -= 512;
        if( mMaxPacketSize<512 )
            mMaxPacketSize = 512;

        qDebug() << tr("New packet size: %1 bytes").arg(mMaxPacketSize);
    }
}

void QRgbdServer::onReadyRead()
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
