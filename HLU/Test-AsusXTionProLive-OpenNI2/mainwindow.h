#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qopencvscene.h>
#include <q2dmapscene.h>
#include "qopenni2grabber.h"
#include <opencvtools.h>

#include <OpenNI.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace Ui {
class MainWindow;
}

using namespace roboctrl;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool connectSensor();

protected:
    void changeEvent(QEvent *e);
    void resizeEvent(QResizeEvent *e) Q_DECL_OVERRIDE;

private slots:
    void on_actionConnect_triggered();

    void onNewColorImage( cv::Mat bgr );
    void onNewDepthImage( cv::Mat depth );
    void onNew2dMap( Q2dMap* map );
    void onNewInfoStr( QString infoStr );

    void on_splitter_2_splitterMoved(int pos, int index);
    void on_splitter_3_splitterMoved(int pos, int index);

private:
    Ui::MainWindow *ui;

    QOpenNI2Grabber* mOpenNI2Grabber;

    QOpenCVScene* mRgbScene;
    QOpenCVScene* mDepthScene;
    Q2dMapScene* m2dMapScene;

    QGraphicsLineItem* mScanLine;
};

#endif // MAINWINDOW_H
