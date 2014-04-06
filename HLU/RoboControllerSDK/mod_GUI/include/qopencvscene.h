#ifndef QOPENCVSCENE_H
#define QOPENCVSCENE_H

#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>

#include "opencv2/core/core.hpp"

class QOpenCVScene : public QGraphicsScene
{
    Q_OBJECT
public:
    /// Default constructor
    explicit QOpenCVScene(QObject *parent = 0);

public slots:
    /// Sets Background Image from OpenCV cv::Mat
    void setBgImage( cv::Mat& cvImg );
    /// Sets dimension of the Joypad
    void setJoypadSize( QSize bgSize, QSize padSize );
    /// Draws the joypad at button down
    void buttonDown(QPointF mBnDownPos );

private:
    /// Converts cv::Mat to QImage
    QImage  cvMatToQImage( const cv::Mat &inMat );
    /// Converts cv::Mat to QPixmap
    QPixmap cvMatToQPixmap( const cv::Mat &inMat );

signals:

private:    
    QGraphicsPixmapItem* mBgPixmapItem; ///< Background image

    QGraphicsEllipseItem* mJoypadBgItem; ///< Background of the joypad
    QGraphicsEllipseItem* mJoypadPadItem; ///< Pad of the joypad

};

#endif // QOPENCVSCENE_H
