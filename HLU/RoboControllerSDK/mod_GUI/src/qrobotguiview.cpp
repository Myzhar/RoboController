#include "qrobotguiview.h"
#include <QDebug>
#include <QtCore>

#ifndef QT_NO_OPENGL
#include <QGLWidget>
#endif

QRobotGUIView::QRobotGUIView(QWidget *parent) :
    QGraphicsView(parent),
    mLcdFwSpeed(NULL),
    mLcdRotSpeed(NULL)
{
#ifndef QT_NO_OPENGL
    mOpenGLActive = true;
    setViewport( new QGLWidget );

    qDebug() << tr("OpenGL Vieport enabled");
#else
    mOpenGLActive = false;
    setViewport( new QWidget );

    qDebug() << tr("OpenGL Vieport disabled");
#endif

    setInteractive( true );

    mScene = new QOpenCVScene();

    setScene( mScene );

    mLastJoyX = 0.0;
    mLastJoyY = 0.0;

    mLcdFwSpeed = new QLCDNumber(8);
    mLcdFwSpeed = new QLCDNumber(8);

    mProxyLcdFwSpeed = mScene->addWidget(mLcdFwSpeed);
    mProxyLcdRotSpeed = mScene->addWidget(mLcdRotSpeed);
}

void QRobotGUIView::setJoypadSize( QSize bgSize, QSize thumbSize )
{
    double maxPos = (bgSize.width()-thumbSize.width()/2)/2;

    mMaxJoypadMove = maxPos;

    // >>>>> Let's keep the size of the Joypad constant
    if( transform().isScaling() )
    {
        qreal scale = transform().m11();
        bgSize.scale( bgSize.width()/scale, bgSize.height()/scale,
                      Qt::KeepAspectRatio );
        thumbSize.scale( thumbSize.width()/scale, thumbSize.height()/scale,
                         Qt::KeepAspectRatio );

        mMaxJoypadMove /= scale;

        //qDebug() << scale;
    }
    // <<<<< Let's keep the size of the Joypad constant

    mScene->setJoypadSize( bgSize, thumbSize );

    QPointF origin = mapToScene( bgSize.width()/10.0, bgSize.height()/10.0 );

    mProxyLcdFwSpeed->setPos( origin);
    //mProxyLcdFwSpeed->setMinimumSize( bgSize.width(), bgSize.height()/3 );
    mProxyLcdRotSpeed->setPos( origin + QPointF( 0.0f, bgSize.height()/3+5 ));
    //mProxyLcdRotSpeed->setMinimumSize( bgSize.width(), bgSize.height()/3 );
}

void QRobotGUIView::mousePressEvent(QMouseEvent *event)
{
    mBnDownPos = event->pos();
    mLastPos = mBnDownPos;

    QPointF posScene = mapToScene( mBnDownPos );

    mScene->buttonDown( posScene );

    //qDebug() << Q_FUNC_INFO;
}

void QRobotGUIView::mouseReleaseEvent(QMouseEvent *event)
{
    mScene->buttonUp();

    //qDebug() << Q_FUNC_INFO;

    emit newJoypadValues(0.0, 0.0);

    mLastJoyX = 0.0;
    mLastJoyY = 0.0;
}

//void QRobotGUIView::resizeEvent(QResizeEvent * ev)
//{

//}

void QRobotGUIView::mouseMoveEvent(QMouseEvent *event)
{
    mLastPos = event->pos();

    QPointF posScene = mapToScene( mLastPos );
    QPointF centerPosScene = mapToScene( mBnDownPos );

    // >>>>> Let's keep the thumb inside the Joypad area
    double rho = qSqrt( (centerPosScene.x()-posScene.x())*(centerPosScene.x()-posScene.x()) +
                        (centerPosScene.y()-posScene.y())*(centerPosScene.y()-posScene.y()) );

    double x = posScene.x()-centerPosScene.x();
    double y = posScene.y()-centerPosScene.y();

    if( rho>mMaxJoypadMove )
    {
        /*double alpha = qAtan2( posScene.y()- centerPosScene.y(), posScene.x()- centerPosScene.x() );
        double x = mMaxJoypadMove*qCos(alpha);
        double y = mMaxJoypadMove*qSin(alpha);*/

        double ratio = mMaxJoypadMove/rho;
        //x = (posScene.x()-centerPosScene.x())*ratio;
        //y = (posScene.y()-centerPosScene.y())*ratio;
        x *= ratio;
        y *= ratio;

        posScene.setX( centerPosScene.x() + x );
        posScene.setY( centerPosScene.y() + y );
    }
    // <<<<< Let's keep the thumb inside the Joypad area

    mScene->mouseMove( posScene );

    float joyX = (float)x/(float)mMaxJoypadMove*100.0f;
    float joyY = (float)y/(float)mMaxJoypadMove*100.0f;

    float dX = qAbs(mLastJoyX-joyX);
    float dY = qAbs(mLastJoyY-joyY);

    if( dX>JOY_MOVE_STEP || dY>JOY_MOVE_STEP )
    {
        mLastJoyX = joyX;
        mLastJoyY = joyY;

        emit newJoypadValues(joyX, -joyY); // Remember that Y axis is inverted in Scene coordinates
    }
}


