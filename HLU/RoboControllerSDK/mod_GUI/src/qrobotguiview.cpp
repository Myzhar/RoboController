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

    QPointF posScene = mapToScene( mBnDownPos );

    mScene->buttonDown( posScene );
}

void QRobotGUIView::mouseReleaseEvent(QMouseEvent *event)
{
    mScene->buttonUp();
}

void QRobotGUIView::mouseMoveEvent(QMouseEvent *event)
{
    mLastPos = event->pos();

    QPointF posScene = mapToScene( mLastPos );
    mScene->mouseMove( posScene );
}

