#ifndef OPENCVTOOLS_H
#define OPENCVTOOLS_H

#include <opencv2/core/core.hpp>
#include <QImage>

class OpenCVTools
{
public:
    OpenCVTools();

    static cv::Mat QImageToCvMat( const QImage &inImage, bool inCloneImageData = true );
};

#endif // OPENCVTOOLS_H
