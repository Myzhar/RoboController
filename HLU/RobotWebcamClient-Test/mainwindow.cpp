#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    mScene(NULL),
    mWebcamClient(NULL)
{
    ui->setupUi(this);

    mScene = new QOpenCVScene();

    ui->graphicsView->setScene( mScene );
}

MainWindow::~MainWindow()
{
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

void MainWindow::onNewImageReceived()
{
    cv::Mat img = mWebcamClient->getLastFrame();

    mScene->setBgImage( img );
}

void MainWindow::on_actionConnect_triggered()
{
    if( mWebcamClient!=NULL )
        delete mWebcamClient;

    mWebcamClient = new QWebcamClient( );

    connect( mWebcamClient, SIGNAL(newImageReceived()),
             this, SLOT(onNewImageReceived()) );


}
