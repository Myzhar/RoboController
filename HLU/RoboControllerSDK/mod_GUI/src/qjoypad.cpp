#include <qjoypad.h>
#include <QPainter>
#include <QDebug>

#include <stdlib.h>
#include <stdio.h>
#include <QApplication>
#include <QDesktopWidget>
#include <math.h>
#include <QResource>
#include <QString>
#include <qmath.h>

namespace roboctrl
{

QJoypad::QJoypad(QWidget *parent) :
    QWidget(parent)
{
    mJoyRelPos.setX( 0.0f );
    mJoyRelPos.setY( 0.0f );

    mPxScaleX = 1.0f;
    mPxScaleY = 1.0f;

    mMaxAxis = 100.0f;

    if( !this->hasHeightForWidth() )
    {
        QSizePolicy sizePolicy =
                QSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setWidthForHeight(true);
        this->setSizePolicy( sizePolicy );
    }
    

}

void QJoypad::setJoypadValues( float x, float y )
{
    //    if( !isVisible() )
    //        return;

    static float oldX=0.0f,oldY=0.0f;

    if( fabs(x-oldX)<20.0f || fabs(y-oldY)<20.0f )
        return;

    oldX = x; oldY = y;

    if( x < -mMaxAxis )
        mJoyRelPos.setX( -mMaxAxis );
    else if( x > mMaxAxis )
        mJoyRelPos.setX( mMaxAxis );
    else
        mJoyRelPos.setX( x );

    if( y < -mMaxAxis )
        mJoyRelPos.setY( -mMaxAxis );
    else if( y > mMaxAxis )
        mJoyRelPos.setY( mMaxAxis );
    else
        mJoyRelPos.setY( y );

    if( isVisible() )
        update();

    emit newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );
    //qDebug() /*<< PREFIX*/ << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void QJoypad::resizeEvent( QResizeEvent* event )
{
    QSize newSize = event->size();

    QPixmap bgPm = QPixmap(":/joypad/images/joystick_background.png");

    mJoyBg = bgPm.scaled( newSize,
                          Qt::IgnoreAspectRatio,
                          Qt::SmoothTransformation );

    float imScaleX = (float)mJoyBg.width()/bgPm.width();
    float imScaleY = (float)mJoyBg.height()/bgPm.height();

    QPixmap pm = QPixmap(":/joypad/images/joystick_thumb.png");
    QSize scaledPadSize;
    int newW = ((int)((float)pm.width()*imScaleX)+0.5f);
    scaledPadSize.setWidth( newW );
    int newH = ((int)((float)pm.height()*imScaleY)+0.5f);
    scaledPadSize.setHeight( newH );
    mJoypad = pm.scaled( scaledPadSize,
                         Qt::IgnoreAspectRatio,
                         Qt::SmoothTransformation  );

    mBgSize = newSize.width();
    mPadSize = mJoypad.width();

    // ---> Pixel scale according to range
    mPxScaleX = (mMaxAxis*2.0f/*+1.0f*/)/(float)newSize.width();
    mPxScaleY = (mMaxAxis*2.0f/*+1.0f*/)/(float)newSize.height();
    // <--- Pixel scale according to range

    emit newJoypadValues( 0.0, 0.0 );
    //qDebug() /*<< PREFIX*/ << "emitted newJoypadValues( 0.0, 0.0 );";

    //qDebug() << newSize;

    QWidget::resizeEvent(event);
}

void QJoypad::mousePressEvent( QMouseEvent *event )
{
    QPoint pos = event->pos();


    //    fprintf( stderr, "Pos: (%3d,%3d)/n", (int)pos.x(), (int)pos.y());
    //    fflush(stderr);

    float posX = pos.x() - width()/2;
    mJoyRelPos.setX( posX*mPxScaleX );

    float posY = (pos.y() - height()/2);
    mJoyRelPos.setY( posY*mPxScaleY );

    //    fprintf( stderr, "Rel Pos: (%g,%g)/n", posX, posY );
    //    fflush(stderr);

    update();

    emit mouseButtonDown( event );
    emit newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );

    //qDebug() /*<< PREFIX*/ << "emitted mouseButtonDown( event );";
    //qDebug() /*<< PREFIX*/ << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void QJoypad::mouseMoveEvent( QMouseEvent *event )
{
    QPoint pos = event->pos();

    //    fprintf( stderr, "Pos: (%3d,%3d)/n", (int)pos.x(), (int)pos.y());
    //    fflush(stderr);

    double minPos = (mBgSize-mPadSize)/4;
    double maxPos = width()/2-minPos;

    QPointF centerWidget;
    centerWidget.setX( width()/2);
    centerWidget.setY( height()/2 );

    // >>>>> Let's keep the thumb inside the Joypad area
    double rho = qSqrt( (centerWidget.x()-pos.x())*(centerWidget.x()-pos.x()) +
                        (centerWidget.y()-pos.y())*(centerWidget.y()-pos.y()) );

    //qDebug() << centerWidget << rho << maxPos;

    double X = pos.x()-centerWidget.x();
    double Y = pos.y()-centerWidget.y();

    if( rho>maxPos )
    {
        double ratio = maxPos/rho;

        X *= ratio;
        Y *= ratio;

    }
    // <<<<< Let's keep the thumb inside the Joypad area

    mJoyRelPos.setX( X * mPxScaleX);

    mJoyRelPos.setY( Y * mPxScaleY );

    //    fprintf( stderr, "Rel Pos: (%g,%g)/n", posX, posY );
    //    fflush(stderr);

    update();

    emit newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );
    //qDebug() /*<< PREFIX*/ << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void QJoypad::mouseReleaseEvent( QMouseEvent *event )
{
    mJoyRelPos.setX( 0.0 );
    mJoyRelPos.setY( 0.0 );

    update();

    emit mouseButtonUp( event );
    emit newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );

    //qDebug() /*<< PREFIX*/ << "emitted mouseButtonUp( event );";
    //qDebug() /*<< PREFIX*/ << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void QJoypad::paintEvent(QPaintEvent *event)
{
    event->accept();

    int realJoyPosX = mJoyRelPos.x()/mPxScaleX;
    int realJoyPosY = mJoyRelPos.y()/mPxScaleY;

    int padOrigX = (int)((realJoyPosX-mJoypad.width()/2)+0.5f) + width()/2;
    int padOrigY = (int)((realJoyPosY-mJoypad.height()/2)+0.5f) + height()/2;

    QPainter painter(this);

    painter.setOpacity( 0.7 );
    painter.drawLine( width()/2, height()/2,
                      realJoyPosX+width()/2, realJoyPosY+height()/2 );

    // Background
    painter.setOpacity(0.4);
    painter.drawPixmap( 0, 0, mJoyBg);

    // Pad
    painter.setOpacity( 0.7 );
    painter.drawPixmap( padOrigX, padOrigY, mJoypad );
}

/*int QJoypad::heightForWidth( int w )
{
    return w;
}*/

}

