#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qopencvscene.h>
#include <qwebcamclient.h>
#include <QLabel>

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
    void onNewImageReceived();

private slots:
    void on_actionConnect_triggered();

private:
    Ui::MainWindow *ui;

    QOpenCVScene* mScene; ///< Default scene to be rendered
    QWebcamClient* mWebcamClient;

    QLabel* mStatusInfo;
};

#endif // MAINWINDOW_H
