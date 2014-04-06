#include "qopencvscene.h"
#include <QDebug>
#include <QGraphicsSceneMouseEvent>

#include <opencv2/highgui/highgui.hpp>

QOpenCVScene::QOpenCVScene(QObject *parent) :
    QGraphicsScene(parent),
    mBgPixmapItem(NULL)
{
    setBackgroundBrush( QBrush(QColor(50,50,50)));
}

void QOpenCVScene::setBgImage( cv::Mat& cvImg )
{
    if(!mBgPixmapItem)
    {
        mBgPixmapItem = new QGraphicsPixmapItem( cvMatToQPixmap(cvImg) );
        //cv::imshow( "Test", cvImg );
        mBgPixmapItem->setPos( 0,0 );

        mBgPixmapItem->setZValue( 0.0 );
        addItem( mBgPixmapItem );
    }
    else
        mBgPixmapItem->setPixmap( cvMatToQPixmap(cvImg) );

    setSceneRect( 0,0, cvImg.cols, cvImg.rows );

    update();
}

void QOpenCVScene::setJoypadSize( QSize bgSize, QSize padSize )
{
    /*QRectF rect = mJoypadBgItem->rect();
    mJoypadBgItem->setRect( rect.x()+(rect.width()-bgSize.width())/2,
                            rect.y()+(rect.height()-bgSize.height())/2,
                            bgSize.width(), bgSize.height() );

    rect = mJoypadPadItem->rect();
    mJoypadPadItem->setRect( rect.x()+(rect.width()-padSize.width())/2,
                             rect.y()+(rect.height()-padSize.height())/2,
                             padSize.width(), padSize.height() );*/
    mJoypadBgItem = new QGraphicsEllipseItem(QRectF(0,0,100,100));
    mJoypadPadItem = new QGraphicsEllipseItem(QRectF(35,35,30,30));
    addItem(mJoypadBgItem);
    addItem(mJoypadPadItem);

}

void QOpenCVScene::buttonDown( QPointF mBnDownPos )
{
    mJoypadBgItem->setPos( mBnDownPos.x()-mJoypadBgItem->rect().width()/2,
                           mBnDownPos.y()-mJoypadBgItem->rect().height()/2);

    addItem( mJoypadBgItem );
}

QImage QOpenCVScene::cvMatToQImage( const cv::Mat &inMat )
{
    switch ( inMat.type() )
    {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );

        return image;
    }

        // 8-bit, 3 channel
    case CV_8UC3:
    {
        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );

        return image.rgbSwapped();
    }

        // 8-bit, 1 channel
    case CV_8UC1:
    {
        static QVector<QRgb>  sColorTable;

        // only create our color table once
        if ( sColorTable.isEmpty() )
        {
            for ( int i = 0; i < 256; ++i )
                sColorTable.push_back( qRgb( i, i, i ) );
        }

        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );

        image.setColorTable( sColorTable );

        return image;
    }

    default:
        qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
        break;
    }

    return QImage();
}

QPixmap QOpenCVScene::cvMatToQPixmap( const cv::Mat &inMat )
{
    return QPixmap::fromImage( cvMatToQImage( inMat ) );
}
