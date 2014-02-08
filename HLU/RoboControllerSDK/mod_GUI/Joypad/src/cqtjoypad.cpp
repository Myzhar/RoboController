#include <cqtjoypad.h>
#include <QPainter>
#include <QDebug>

#include <stdlib.h>
#include <stdio.h>
#include <QApplication>
#include <QDesktopWidget>
#include <math.h>

CQtJoypad::CQtJoypad(QWidget *parent) :
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

void CQtJoypad::setJoypadValues( float x, float y )
{
//    if( !isVisible() )
//        return;

    static float oldX=0.0f,oldY=0.0f;

    if( fabs(x-oldX)<10.0f || fabs(y-oldY)<10.0f )
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
    //qDebug() << PREFIX << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void CQtJoypad::resizeEvent( QResizeEvent* event )
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
    //qDebug() << PREFIX << "emitted newJoypadValues( 0.0, 0.0 );";

    qDebug() << newSize;

    QWidget::resizeEvent(event);
}

void CQtJoypad::mousePressEvent( QMouseEvent *event )
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

    //qDebug() << PREFIX << "emitted mouseButtonDown( event );";
    //qDebug() << PREFIX << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void CQtJoypad::mouseMoveEvent( QMouseEvent *event )
{
    QPoint pos = event->pos();

//    fprintf( stderr, "Pos: (%3d,%3d)/n", (int)pos.x(), (int)pos.y());
//    fflush(stderr);

    int minPos = (mBgSize-mPadSize)/2;
    int maxPos = width()-minPos;

    int X,Y;
    if( pos.x()<minPos )
        X=minPos;
    else if( pos.x()>maxPos )
        X = maxPos;
    else
        X = pos.x();

    if( pos.y()<minPos )
        Y=minPos;
    else if( pos.y()>maxPos )
        Y = maxPos;
    else
        Y = pos.y();

    float posX = X - width()/2  ;
    mJoyRelPos.setX( posX * mPxScaleX);

    float posY = (Y - height()/2);
    mJoyRelPos.setY( posY * mPxScaleY );

//    fprintf( stderr, "Rel Pos: (%g,%g)/n", posX, posY );
//    fflush(stderr);

    update();

    emit newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );
    //qDebug() << PREFIX << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void CQtJoypad::mouseReleaseEvent( QMouseEvent *event )
{
    mJoyRelPos.setX( 0.0 );
    mJoyRelPos.setY( 0.0 );

    update();

    emit mouseButtonUp( event );
    emit newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );

    //qDebug() << PREFIX << "emitted mouseButtonUp( event );";
    //qDebug() << PREFIX << "emitted newJoypadValues( mJoyRelPos.x(), -mJoyRelPos.y() );";
}

void CQtJoypad::paintEvent(QPaintEvent *event)
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

/*int CQtJoypad::heightForWidth( int w )
{
    return w;
}*/
