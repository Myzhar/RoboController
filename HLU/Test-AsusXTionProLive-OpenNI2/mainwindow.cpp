#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    mOpenNI2Grabber(NULL),
    mRgbScene(NULL),
    mDepthScene(NULL)
{
    ui->setupUi(this);

    mRgbScene = new QOpenCVScene();
    mDepthScene = new QOpenCVScene();
    m2dMapScene = new Q2dMapScene();

    ui->graphicsView_rgb->setScene( mRgbScene );
    ui->graphicsView_depth->setScene( mDepthScene );

    m2dMapScene->setSceneRect( -5000, 0, 10000, 10000 );
    ui->graphicsView_map_2D->scale( 1, -1);
    ui->graphicsView_map_2D->setScene( m2dMapScene );
}

MainWindow::~MainWindow()
{
    if(mOpenNI2Grabber)
        delete mOpenNI2Grabber;

    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

bool MainWindow::connectSensor()
{
    mOpenNI2Grabber = new QOpenNI2Grabber();

    connect( mOpenNI2Grabber, SIGNAL(newBgrImageAvailable(cv::Mat)),
             this, SLOT(onNewColorImage(cv::Mat)) );
    connect( mOpenNI2Grabber, SIGNAL(newDepthImageAvailable(cv::Mat)),
             this, SLOT(onNewDepthImage(cv::Mat)) );
    connect( mOpenNI2Grabber, SIGNAL(new2dMat(Q2dMap*)),
             this, SLOT(onNew2dMap(Q2dMap*)) );
    connect( mOpenNI2Grabber, SIGNAL(newInfoString(QString)),
             this, SLOT(onNewInfoStr(QString)) );

    return true;
}

void MainWindow::on_actionConnect_triggered()
{
    connectSensor();
}

void MainWindow::onNewColorImage( cv::Mat bgr )
{
    mRgbScene->setBgImage( bgr );

    ui->graphicsView_rgb->fitInView(QRectF(0,0, bgr.cols, bgr.rows),
                                          Qt::KeepAspectRatio );
}

void MainWindow::onNewDepthImage( cv::Mat depth )
{
    mDepthScene->setBgImage( depth );

    ui->graphicsView_depth->fitInView(QRectF(0,0, depth.cols, depth.rows),
                                          Qt::KeepAspectRatio );
}

void MainWindow::onNew2dMap(Q2dMap *map )
{
    m2dMapScene->setMap( *map );
}

void MainWindow::onNewInfoStr( QString infoStr)
{
    ui->textEdit_info->insertPlainText( infoStr );
}

void MainWindow::on_splitter_2_splitterMoved(int pos, int index)
{
    QRectF sceneRect = m2dMapScene->sceneRect();
    ui->graphicsView_map_2D->fitInView( -(sceneRect.width()/5), 0, sceneRect.width()/2.5, sceneRect.width()/2.5,
                                        Qt::KeepAspectRatio );
}

void MainWindow::on_splitter_3_splitterMoved(int pos, int index)
{
    QRectF sceneRect = m2dMapScene->sceneRect();
    ui->graphicsView_map_2D->fitInView( -(sceneRect.width()/5), 0, sceneRect.width()/2.5, sceneRect.width()/2.5,
                                        Qt::KeepAspectRatio );
}

void MainWindow::resizeEvent(QResizeEvent *e)
{
    QRectF sceneRect = m2dMapScene->sceneRect();
    ui->graphicsView_map_2D->fitInView( -(sceneRect.width()/5), 0, sceneRect.width()/2.5, sceneRect.width()/2.5,
                                        Qt::KeepAspectRatio );
}
