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

    mFpsInfo = new QLabel( tr("FPS: ----") );
    ui->statusBar->addWidget(mFpsInfo);

    ui->actionDisconnect->setEnabled(false);
    ui->actionConnect->setEnabled(true);

    mStatusProgr = new QProgressBar();
    mStatusProgr->setRange(0,100);
    mStatusProgr->setTextVisible( true );
    mStatusProgr->setMaximumWidth( 200 );
    mStatusProgr->setAlignment(Qt::AlignCenter);
    ui->statusBar->addPermanentWidget( mStatusProgr );
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

    quint64 frm,frmComplete;
    qint64 totElapsed = mTime.elapsed();
    qint64 elapsed = totElapsed-mLastFrmTime;
    double fps = 1000.0/elapsed;
    mLastFrmTime = totElapsed;

    mWebcamClient->getStats( frm, frmComplete );    
    mFpsInfo->setText(tr("FPS: %1").arg(fps,2,'f',3,'0'));

    int perc = (int)(((double)frmComplete/(double)frm*100.0)+0.5);

    QString info = tr("Video stats: %1/%2 - %p%").arg(frmComplete).arg(frm);
    mStatusProgr->setFormat( info );
    mStatusProgr->setValue(perc);
}

void MainWindow::on_actionConnect_triggered()
{
    if( mWebcamClient!=NULL )
        delete mWebcamClient;

    mWebcamClient = new QWebcamClient( );

    connect( mWebcamClient, SIGNAL(newImageReceived()),
             this, SLOT(onNewImageReceived()) );

    ui->actionDisconnect->setEnabled(true);
    ui->actionConnect->setEnabled(false);

    mTime.start();
    mLastFrmTime = 0;
}

void MainWindow::on_actionDisconnect_triggered()
{
    disconnect( mWebcamClient, SIGNAL(newImageReceived()),
             this, SLOT(onNewImageReceived()) );

    if( mWebcamClient!=NULL )
        delete mWebcamClient;
    mWebcamClient=NULL;

    ui->actionDisconnect->setEnabled(false);
    ui->actionConnect->setEnabled(true);
}
