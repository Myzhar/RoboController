#ifndef QOPENNI2SERVER_H
#define QOPENNI2SERVER_H

#include <QThread>
#include <QDebug>

#include "OpenNI.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencvtools.h>

#define BUFFER_SIZE 10

namespace roboctrl
{

class QOpenNI2Grabber : public QThread
{
    Q_OBJECT
public:
    explicit QOpenNI2Grabber(QObject *parent = 0);
    virtual ~QOpenNI2Grabber();

    /*! Set new resolutions and FPS for Depth and RGB streams
     * @param width new Width
     * @param height new Height
     * @param fps new FPS
     *
     * @note the function will set default
     * PIXEL_FORMAT_DEPTH_1_MM for Depth stream
     * and PIXEL_FORMAT_RGB888 for RGB stream     *
     *
     * @return true if correct
     */
    bool setNewVideomode( int width, int height, int fps=30 );

private:
    bool initSensor(); ///< Initializes the sensor
    void create2dMap(); ///< Creates a 2D map from depth map

    void outputInfo( QString infoStr ); ///< Emit a debug string

protected:
    void run() Q_DECL_OVERRIDE;

signals:
    void newBgrImageAvailable( cv::Mat bgr );
    void newDepthAvailable( cv::Mat depth16u );
    void newDepthImageAvailable( cv::Mat depth8u );
    void new2dMat( Q2dMap* map );
    void newInfoString( QString infoStr );

public slots:

private:
    bool mStopped; ///< Used to exit the thread loop

    openni::Device mDevice; ///< OpenNI2 Device

    openni::VideoStream mDepthVs;   ///< VideoStream for depth
    openni::VideoStream mRgbVs;     ///< VideoStream for RGB

    int mWidth;     ///< Width of the frames
    int mHeight;    ///< Heigh tof the frames

    openni::VideoFrameRef mDepthFrame;    ///< Depth OpenNI2 frame
    openni::VideoFrameRef mRgbFrame;      ///< RGB OpenNI2 frame

    openni::VideoStream** mStreams;       ///< Vector of the streams

    cv::Mat mCvBgr;         ///< buffer OpenCV BGR Image (convertion from RGB to BGR is automatically done)
    cv::Mat mCvDepth16u;    ///< buffer OpenCV Depth Image 16bit
    cv::Mat mCvDepth8u;     ///< buffer OpenCV Depth Image 8bit (used for visualization)

    QVector< cv::Mat > mCvBgrBuf;         ///< buffer OpenCV BGR Image (convertion from RGB to BGR is automatically done)
    QVector< cv::Mat > mCvDepth16uBuf;    ///< buffer OpenCV Depth Image 16bit
    QVector< cv::Mat > mCvDepth8uBuf;     ///< buffer OpenCV Depth Image 8bit (used for visualization)

    QVector< quint64 > mDepthTimeStamp;
    QVector< quint64 > mRgbTimeStamp;

    QVector< Q2dMap > m2dMapBuf;

    int mBufIdx;

    bool mDepthValid;
    bool mRgbValid;

    double mMaxDepth; ///< Max depth available from depth stream
    double mMinDepth; ///< Min depth available from depth stream
};

}

#endif // QOPENNI2SERVER_H
