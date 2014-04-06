#include "qrobotguiview.h"
#include <QDebug>

#ifndef QT_NO_OPENGL
#include <QGLWidget>
#endif

QRobotGUIView::QRobotGUIView(QWidget *parent) :
    QGraphicsView(parent)
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
}

void QRobotGUIView::mousePressEvent(QMouseEvent *event)
{
    mBnDownPos = event->pos();
    mLastPos = mBnDownPos;

    qDebug() << tr("Mouse pressed %1 %2").arg(mBnDownPos.x() ).arg(mBnDownPos.y() );

    mScene->buttonDown( mBnDownPos );
}

void QRobotGUIView::mouseReleaseEvent(QMouseEvent *event)
{
    qDebug() << tr("Mouse released %1 %2").arg(event->pos().x() ).arg(event->pos().y() );
}

void QRobotGUIView::mouseMoveEvent(QMouseEvent *event)
{
    if( qAbs(mLastPos.x()-event->x())>=MIN_MOUSE_STEP ||
            qAbs(mLastPos.y()-event->y())>=MIN_MOUSE_STEP)
    {
        qDebug() << tr("Mouse moved %1 %2").arg(event->pos().x() ).arg(event->pos().y() );
        mLastPos = event->pos();
    }
}

