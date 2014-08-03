#ifndef QROBOTGUIVIEW_H
#define QROBOTGUIVIEW_H

#include <QGraphicsView>
#include <QMouseEvent>
#include <QPoint>
#include <QLCDNumber>
#include <QLabel>
#include <QGraphicsProxyWidget>
#include "qopencvscene.h"

#include <opencv2/core/core.hpp>

#define JOY_MOVE_STEP 5 ///< The virtual joypad sends new speed values only if different from the latest of at less 5

class QRobotGUIView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit QRobotGUIView(QWidget *parent = 0);

    QOpenCVScene* scene(){return mScene;}

protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

    //virtual void resizeEvent(QResizeEvent * ev);

signals:
    void newJoypadValues( float x, float y ); // Joypad values in range [-100,+100]

public slots:
    void setJoypadSize( QSize bgSize, QSize thumbSize );

private:
    bool mOpenGLActive; ///< Indicates if OpenGL is active

    bool mButtonDown;
    QPoint mBnDownPos; ///< Holds the position of the mouse at ButtonDown event
    QPoint mLastPos; ///< Holds the last taken position of the mouse

    QOpenCVScene* mScene; ///< Default scene to be rendered

    double mMaxJoypadMove; ///< Max distance of the Thumb of the joypad from its center

    double mLastJoyX; ///< Last Joypad X value in the range [-100,100]
    double mLastJoyY; ///< Last Joypad Y value in the range [-100,100]

    QLCDNumber* mLcdFwSpeed;
    QLCDNumber* mLcdRotSpeed;
    QGraphicsProxyWidget* mProxyLcdFwSpeed;
    QGraphicsProxyWidget* mProxyLcdRotSpeed;
};

#endif // QROBOTGUIVIEW_H
