#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qopencvscene.h>
#include <qwebcamclient.h>
#include <QLabel>
#include <QProgressBar>
#include <QTime>

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

protected:
    void changeEvent(QEvent *e);    

protected slots:
    void onNewImageReceived(cv::Mat frame );

private slots:
    void on_actionConnect_triggered();

    void on_actionDisconnect_triggered();

private:
    Ui::MainWindow *ui;

    QOpenCVScene* mScene; ///< Default scene to be rendered
    QWebcamClient* mWebcamClient;

    QLabel* mFpsInfo;
    QProgressBar* mStatusProgr;

    QTime mTime;
    qint64 mLastFrmTime;

    QVector<double> mFpsVec;
    double mFpsSum;
    int mFpsIdx;
};

#endif // MAINWINDOW_H
