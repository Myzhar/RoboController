#include "qopenni2grabber.h"
#include <QApplication>

#define DEF_WIDTH   640
#define DEF_HEIGHT  480

namespace roboctrl
{

QOpenNI2Grabber::QOpenNI2Grabber(QObject *parent) :
    QThread(parent),
    mStreams(NULL)
{
    mCvBgrBuf.resize(BUFFER_SIZE);
    mCvDepth16uBuf.resize(BUFFER_SIZE);
    mCvDepth8uBuf.resize(BUFFER_SIZE);
    mDepthTimeStamp.resize(BUFFER_SIZE);
    mRgbTimeStamp.resize(BUFFER_SIZE);
    m2dMapBuf.resize(BUFFER_SIZE);

    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<Q2dMap*>("Q2dMap*");    
    start();
}

QOpenNI2Grabber::~QOpenNI2Grabber()
{
    if( !mStopped )
        mStopped=true;

    wait(2000);

    //openni::OpenNI::shutdown();
    //qDebug() << tr("OpenNI2 deinitialized");
}

bool QOpenNI2Grabber::setNewVideomode(int width, int height, int fps/*=30*/  )
{
    if( !mDepthVs.isValid() || !mRgbVs.isValid() )
    {
        qDebug() << tr("There is no valid stream to set resolution");
        return false;
    }

    // >>>>> Stop new frames generation
    mDepthVs.stop();
    mRgbVs.stop();
    // <<<<< Stop new frames generation

    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;

    depthVideoMode = mDepthVs.getVideoMode();
    colorVideoMode = mRgbVs.getVideoMode();

    depthVideoMode.setResolution( width, height );
    colorVideoMode.setResolution( width, height );

    depthVideoMode.setFps( fps );
    colorVideoMode.setFps( fps );

    depthVideoMode.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );
    colorVideoMode.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );

    openni::Status rcD, rcC;

    rcD = mDepthVs.setVideoMode( depthVideoMode );
    rcC = mRgbVs.setVideoMode( colorVideoMode );

    if(rcD != openni::STATUS_OK || rcC != openni::STATUS_OK)
    {
        qDebug() << tr("New Videomodes not set");
        return false;
    }

    // >>>>> Restart new frames generation
    rcD = mDepthVs.start();
    rcC = mRgbVs.start();

    if(rcD != openni::STATUS_OK || rcC != openni::STATUS_OK)
    {
        qDebug() << tr("RGB and Depth streams not restarted after resolution change");
        return false;
    }
    // <<<<< Restart new frames generation

    return true;
}

void QOpenNI2Grabber::outputInfo( QString infoStr )
{
    emit newInfoString( infoStr );
    qDebug() << infoStr;
}

bool QOpenNI2Grabber::initSensor()
{
    mStopped = true;
    QString infoStr;

    openni::Status rc = openni::STATUS_OK;
    openni::Status rcC = openni::STATUS_OK;
    openni::Status rcD = openni::STATUS_OK;

    const char* deviceURI = openni::ANY_DEVICE;
    rc = openni::OpenNI::initialize();

    if (rc != openni::STATUS_OK)
    {
        outputInfo( tr("OpenNI2 initialization failed: %1").arg(openni::OpenNI::getExtendedError()) );
        return false;
    }

    qDebug() << tr("OpenNI2 initialized");

    rc = mDevice.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        outputInfo( tr("OpenNI2: Device open failed: %1", openni::OpenNI::getExtendedError()) );
        openni::OpenNI::shutdown();
        return false;
    }

    openni::DeviceInfo info = mDevice.getDeviceInfo();

    outputInfo( tr("========================") );
    outputInfo( tr("Sensor Info:") );
    outputInfo( tr("Name: %1").arg( info.getName() ) );
    outputInfo( tr("Vendor: %1").arg( info.getVendor() ) );
    outputInfo( tr("USB ID: %1/%2 ").arg( info.getUsbVendorId() ).arg( info.getUsbProductId() ) );

    // Force Registration of Depth to Color
    mDevice.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    if (rc != openni::STATUS_OK)
    {
        outputInfo( tr("OpenNI2: Depth/Color Registration not available").arg( openni::OpenNI::getExtendedError()) );
    }

    // >>>>> Creation of the streams
    rcD = mDepthVs.create(mDevice, openni::SENSOR_DEPTH);
    rcC = mRgbVs.create(mDevice, openni::SENSOR_COLOR);
    // <<<<< Creation of the streams

    // Forcing Resolutions
    setNewVideomode( DEF_WIDTH, DEF_HEIGHT);

    // >>>>> Starting streams
    if (rcD == openni::STATUS_OK)
    {
        rc = mDepthVs.start();
        if (rc != openni::STATUS_OK)
        {
            outputInfo(  tr("OpenNI2: Couldn't start depth stream: %1").arg( openni::OpenNI::getExtendedError() ) );
            mDepthVs.destroy();
        }
        else
            qDebug() << tr("OpenNI2: Depth Stream started");

        mMinDepth = mDepthVs.getMinPixelValue();
        mMaxDepth = mDepthVs.getMaxPixelValue();

        outputInfo( tr("Depth range: [%1,%2] mm").arg(mMinDepth).arg(mMaxDepth) );
    }
    else
    {
        outputInfo( tr("OpenNI2: Couldn't find depth stream: %1").arg(openni::OpenNI::getExtendedError()) );
    }

    if (rcC == openni::STATUS_OK)
    {
        rc = mRgbVs.start();
        if (rc != openni::STATUS_OK)
        {
            outputInfo( tr("OpenNI2: Couldn't start color stream: %1").arg(openni::OpenNI::getExtendedError()) );
            mRgbVs.destroy();
        }
        else
            qDebug() << tr("OpenNI2: Color Stream started");
    }
    else
    {
        outputInfo( tr("OpenNI2: Couldn't find color stream: %1").arg(openni::OpenNI::getExtendedError()) );
    }
    // <<<<< Starting streams

    // >>>>> Working tests
    if( !mDepthVs.isValid() || !mRgbVs.isValid() )
    {
        outputInfo( tr("OpenNI2: No valid streams. Exiting") );
        openni::OpenNI::shutdown();
        return false;
    }

    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;

    if (mDepthVs.isValid() && mRgbVs.isValid())
    {
        depthVideoMode = mDepthVs.getVideoMode();
        colorVideoMode = mRgbVs.getVideoMode();

        int depthWidth = depthVideoMode.getResolutionX();
        int depthHeight = depthVideoMode.getResolutionY();
        int colorWidth = colorVideoMode.getResolutionX();
        int colorHeight = colorVideoMode.getResolutionY();

        if (depthWidth == colorWidth &&
                depthHeight == colorHeight)
        {
            mWidth = depthWidth;
            mHeight = depthHeight;

            if( depthVideoMode.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM )
                outputInfo( tr("Depth unit: 1mm (16 bit unsigned) @%1FPS").arg(depthVideoMode.getFps()) );

            if( colorVideoMode.getPixelFormat() == openni::PIXEL_FORMAT_RGB888 )
                outputInfo( tr("RGB format: RGB888 @%1FPS").arg(colorVideoMode.getFps()) );

            outputInfo( tr("Depth and RGB resolution: %1x%2").arg(mWidth).arg(mHeight) );
        }
        else
        {
            outputInfo( tr("OpenNI2: expected color and depth to be in same resolution: D: %1x%2 C: %3x%4")
                        .arg(depthWidth).arg(depthHeight)
                        .arg(colorWidth).arg(colorHeight) );

            return false;
        }
    }
    // <<<<< Working tests

    // >>>>> Streams vector (used for synchronization)
    mStreams = new openni::VideoStream*[2];
    mStreams[0] = &mDepthVs;
    mStreams[1] = &mRgbVs;
    // <<<<< Streams vector (used for synchronization)

    // >>>>> OpenCV initialization
    mCvBgr.create(mHeight,mWidth,CV_8UC3);
    mCvDepth16u.create(mHeight,mWidth,CV_16U);
    //mCvIr16u.create(mHeight,mWidth,CV_16U);
    // <<<<< OpenCV initialization

    outputInfo( tr("========================") );

    mBufIdx = 0;
    mDepthValid = false;
    mRgbValid = false;

    return true;
}

void QOpenNI2Grabber::run()
{
    qDebug() << tr("QOpenNI2Server thread started");

    if( !initSensor() )
        return;

    mStopped = false;

    forever
    {
        if( mStopped )
            break;

        int changedIndex;
        openni::Status rc = openni::OpenNI::waitForAnyStream(mStreams, 2, &changedIndex);
        if (rc != openni::STATUS_OK)
        {
            qDebug() << tr("QOpenNI2Server: Wait failed");
            return;
        }

        switch (changedIndex)
        {
        case 0:
        {
            mDepthVs.readFrame(&mDepthFrame);
            const uint16_t* imgBuf = (const uint16_t*)mDepthFrame.getData();
            memcpy(mCvDepth16u.data, imgBuf, mHeight*mWidth*sizeof(uint16_t));

            mCvDepth16u.convertTo( mCvDepth8u, CV_8U, 256.0/(mMaxDepth-mMinDepth) );

            //qDebug() << tr("%2 - Depth idx: %1").arg(mDepthFrame.getFrameIndex()).arg(mDepthFrame.getTimestamp());

            mCvDepth8u.copyTo(mCvDepth8uBuf[mBufIdx]);
            mCvDepth16u.copyTo(mCvDepth16uBuf[mBufIdx]);
            mDepthTimeStamp[mBufIdx]=mDepthFrame.getTimestamp();
            mDepthValid = true;

            create2dMap();

        }
            break;

        case 1:
        {
            mRgbVs.readFrame(&mRgbFrame);
            const uint8_t* imgBuf = (const uint8_t*)mRgbFrame.getData();
            memcpy(mCvBgr.data, imgBuf, 3*mHeight*mWidth*sizeof(uint8_t));
            cv::cvtColor( mCvBgr, mCvBgr, CV_RGB2BGR );

            //qDebug() << tr("%2 - RGB idx: %1").arg(mRgbFrame.getFrameIndex()).arg(mRgbFrame.getTimestamp());

            mCvBgr.copyTo(mCvBgrBuf[mBufIdx]);
            mRgbTimeStamp[mBufIdx]=mRgbFrame.getTimestamp();

            mRgbValid = true;
        }
            break;

        default:
            qDebug() << tr("QOpenNI2Server: Error in Wait");
        }

        // >>>>> Synchronized data emission
        if( mDepthValid && mRgbValid )
        {
            emit newDepthAvailable( mCvDepth16uBuf[mBufIdx] );
            emit newDepthImageAvailable( mCvDepth8uBuf[mBufIdx] );
            emit newBgrImageAvailable( mCvBgrBuf[mBufIdx] );
            emit new2dMat( &m2dMapBuf[mBufIdx] );

            //qDebug() << tr("Delta TimeStamp: %1").arg((int)(mRgbTimeStamp[bufIdx]-mDepthTimeStamp[bufIdx])/1000);

            mBufIdx = (++mBufIdx)%BUFFER_SIZE;

            mDepthValid = false;
            mRgbValid = false;
        }
        // <<<<< Synchronized data emission

        //QApplication::processEvents( QEventLoop::AllEvents, 10 );
    }
    cv::destroyAllWindows();

    qDebug() << tr("QOpenNI2Server thread finished");
}

void QOpenNI2Grabber::create2dMap()
{
    openni::Status rc = openni::STATUS_OK;

    if(mDepthValid)
    {
        Q2dMap map;
        map.resize(mWidth);

        for( int i=0; i<mWidth; i++ )
        {
            float Y = (mHeight-1)/2;
            float wX,wY,wZ;
            float valUp = (float)(mCvDepth16uBuf[mBufIdx].at<uint16_t>(((mHeight-1)/2-1),i));
            float valMd = (float)(mCvDepth16uBuf[mBufIdx].at<uint16_t>(((mHeight-1)/2+0),i));
            float valDn = (float)(mCvDepth16uBuf[mBufIdx].at<uint16_t>(((mHeight-1)/2+1),i));

            float meanDepth = (valUp+valMd+valDn)/3.0f;

            rc = openni::CoordinateConverter::convertDepthToWorld( mDepthVs, (float)i, Y, meanDepth, &wX, &wY, &wZ );

            map[i].x = wX;
            map[i].y = wY;
            map[i].z = wZ;

            //qDebug() << map[i].x << map[i].y << map[i].z;
        }

        m2dMapBuf[mBufIdx] = map;
        //qDebug() << mBufIdx;
    }
}

}

