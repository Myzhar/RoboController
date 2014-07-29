#include "cmainwindow.h"
#include "ui_cmainwindow.h"
#include <QApplication>
#include "rcexception.h"
#include <QMessageBox>
#include <QLabel>
#include <QDesktopWidget>
#include <QDir>
#include <QScreen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <loghandler.h>

#include "qrobotconfigdialog.h"
#include "qbatterycalibdialog.h"
#include "qcommon.h"
#include "opencvtools.h"

#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_TCP_PORT 14500
#define DEFAULT_UDP_CTRL_PORT 14560
#define DEFAULT_MULTICAST_TELEM_PORT 14565
#define DEFAULT_UDP_STAT_PORT_SEND 14550
#define DEFAULT_UDP_STAT_PORT_LISTEN 14555

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow),
    mIniSettings(NULL),
    mRobIpLineEdit(NULL),
    mPushButtonConnect(NULL),
    mPushButtonFindServer(NULL),
    mRoboCtrl(NULL),
    mWebcamClient(NULL)
{
    ui->setupUi(this);

    mHasRobotCtrl = false;

    //mOpenCVWidget = NULL;

    // >>>>> Video Widget
    /*#ifdef ANDROID
    //TODO add widget OpenCV not OpenGL
#else
    mOpenCVWidget = new QGlOpenCVWidget( this );
    mOpenCVWidget->setSizePolicy( QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    ((QGridLayout*)(this->centralWidget()->layout()))->addWidget(mOpenCVWidget,0,0);

    ui->widget_video_container->setLayout( new QGridLayout(ui->widget_video_container) );
    ui->widget_video_container->layout()->setContentsMargins(0,0,0,0);
    ui->widget_video_container->layout()->addWidget(mOpenCVWidget);

#endif*/
    // <<<<< Video Widget

    // >>>>> Board Status Widgets
    QPalette pal = ui->widget_PID_status->palette();
    pal.setColor( QPalette::Window, Qt::gray );
    ui->widget_PID_status->setPalette( pal );
    ui->widget_ramp_status->setPalette( pal );
    ui->widget_save_EEPROM_status->setPalette( pal );
    ui->widget_WD_status->setPalette( pal );
    ui->widget_robot_ctrl->setPalette( pal );
    // >>>>> Board Status Widgets

    // >>>>> File INI
    QString iniPath;

#ifndef ANDROID
    iniPath = QApplication::applicationDirPath();
    iniPath += tr("%1.ini").arg(QApplication::applicationName());
#else
    // under Android it is not possible to write in the binary folder. We must store our "ini" file in a new folder under /sdcard/
    iniPath = tr("/sdcard/%1/").arg(INI_FOLDER);
    QDir dir(iniPath);
    if( !dir.exists() )
    {
        dir.mkdir(iniPath);
    }
    iniPath += tr("%1.ini").arg(INI_FOLDER);
#endif

    mIniSettings = new QSettings( iniPath, QSettings::IniFormat, this );

    qDebug() << iniPath;
    // <<<<< File INI

    // >>>>> Robot IP Address from INI File
    mRobIpAddress = mIniSettings->value( ROB_IP, QString() ).toString();
    if(mRobIpAddress.isEmpty())
    {
        mRobIpAddress = DEFAULT_IP;
        mIniSettings->setValue( ROB_IP, mRobIpAddress );
        mIniSettings->sync();
    }

    mRobTcpPort = mIniSettings->value( ROB_TCP_PORT, "-1" ).toInt();
    if(mRobTcpPort == -1 )
    {
        mRobTcpPort = DEFAULT_TCP_PORT;
        mIniSettings->setValue( ROB_TCP_PORT, mRobTcpPort );
        mIniSettings->sync();
    }

    mRobUdpControlPort = mIniSettings->value( ROB_UDP_CTRL_PORT, "-1" ).toInt();
    if(mRobUdpControlPort == -1 )
    {
        mRobUdpControlPort = DEFAULT_UDP_CTRL_PORT;
        mIniSettings->setValue( ROB_UDP_CTRL_PORT, mRobUdpControlPort );
        mIniSettings->sync();
    }

    mRobUdpStatusPortSend = mIniSettings->value( ROB_UDP_STAT_PORT_SEND, "-1" ).toInt();
    if(mRobUdpStatusPortSend == -1 )
    {
        mRobUdpStatusPortSend = DEFAULT_UDP_STAT_PORT_SEND;
        mIniSettings->setValue( ROB_UDP_STAT_PORT_SEND, mRobUdpStatusPortSend );
        mIniSettings->sync();
    }

    mRobUdpStatusPortListen = mIniSettings->value( ROB_UDP_STAT_PORT_LISTEN, "-1" ).toInt();
    if(mRobUdpStatusPortListen == -1 )
    {
        mRobUdpStatusPortListen = DEFAULT_UDP_STAT_PORT_LISTEN;
        mIniSettings->setValue( ROB_UDP_STAT_PORT_LISTEN, mRobUdpStatusPortListen );
        mIniSettings->sync();
    }

    mRobUdpTelemetryPortListen = mIniSettings->value( ROB_UDP_TELEM_PORT_LISTEN, "-1" ).toInt();
    if(mRobUdpTelemetryPortListen == -1 )
    {
        mRobUdpTelemetryPortListen = DEFAULT_MULTICAST_TELEM_PORT;
        mIniSettings->setValue( ROB_UDP_TELEM_PORT_LISTEN, mRobUdpTelemetryPortListen );
        mIniSettings->sync();
    }

    // <<<<< Robot IP Address from INI File

    // >>>>> PID State from INI File
    mPidEnabled = mIniSettings->value( PID_ENABLED, false ).toBool();

    ui->actionPidEnabled->setChecked( mPidEnabled );
    // <<<<< PID State from INI File

    // >>>>> Main Toolbar
    mPushButtonFindServer = new QPushButton( tr("Find Server"), ui->mainToolBar );
    ui->mainToolBar->addWidget( mPushButtonFindServer );

    QLabel* lab = new QLabel(tr("TCP address:"));
    ui->mainToolBar->addWidget( lab );
    mRobIpLineEdit = new QLineEdit( mRobIpAddress, ui->mainToolBar );
    ui->mainToolBar->addWidget( mRobIpLineEdit );

    mRobIpLineEdit->installEventFilter(this);

    mRobIpLineEdit->setInputMethodHints(Qt::ImhUrlCharactersOnly);

    mPushButtonConnect = new QPushButton( tr("Connect"), ui->mainToolBar );
    ui->mainToolBar->addWidget( mPushButtonConnect );


    mPushButtonFindServer->setDefault(true);
    mPushButtonFindServer->setFocus();
    // <<<<< Main Toolbar

    // >>>>> Status Bar
    mStatusLabel = new QLabel(tr("Unconnected"));
    ui->statusBar->addPermanentWidget( mStatusLabel );

    mBatteryLabel = new QLabel(tr("Battery: --.--V/--.--V"));
    ui->statusBar->addWidget( mBatteryLabel );

    mStatusBattLevelProgr = new QProgressBar(ui->statusBar);
    ui->statusBar->addWidget( mStatusBattLevelProgr);
    mStatusBattLevelProgr->setTextVisible(false);
    mStatusBattLevelProgr->setRange(0,100);

    mVideoStats = new QLabel(tr("Video: ---/---"));
    ui->statusBar->addWidget( mVideoStats );
    // <<<<< Status Bar

    // >>>>> Fonts
    int fontPx = COMMON->mScreen.cvtMm2Px( 3 );

#ifndef ANDROID
    QFont font = this->font( );
#else
    QFont font("Roboto");
#endif
    font.setPixelSize( fontPx );
    mStatusLabel->setFont( font );
    mBatteryLabel->setFont( font );
    mPushButtonConnect->setFont( font );
    mPushButtonFindServer->setFont( font );
    mRobIpLineEdit->setFont( font );

#ifndef ANDROID
    QFont unitFont("Monospace");
#else
    QFont unitFont("Roboto");
#endif

    unitFont.setStyleHint(QFont::TypeWriter);
    fontPx = COMMON->mScreen.cvtMm2Px( 3 );
    unitFont.setPixelSize( fontPx );
    ui->label_fw_speed->setFont( unitFont );
    ui->label_rot_speed->setFont( unitFont );

    fontPx = COMMON->mScreen.cvtMm2Px( 2 );
    font.setPixelSize( fontPx );
    font.setBold( false );
    ui->label_PID_status->setFont( font );
    ui->label_ramp_status->setFont( font );
    ui->label_WD_status->setFont( font );
    ui->label_save_EEPROM_status->setFont( font );
    ui->label_robot_control->setFont( font );
    // <<<<< Fonts

    connect( mPushButtonConnect, SIGNAL(clicked()),
             this, SLOT(onConnectButtonClicked()) );
    connect( mPushButtonFindServer, SIGNAL(clicked()),
             this, SLOT(onFindServerButtonClicked()) );

    connect( ui->widget_joypad, SIGNAL(newJoypadValues(float,float)),
             this, SLOT(onNewJoypadValues(float,float)) );
    connect( ui->widget_video_container, SIGNAL(newJoypadValues(float,float)),
             this, SLOT(onNewJoypadValues(float,float)) );

    mMaxMotorSpeed = 2.0f; // m/sec
    mOpenRobotConfig = false;
    mRobotConfigValid = false;

    mNewImageAvailable = false;
    mNewTelemetryAvailable = false;

    QImage img( ":/icon/images/MyzharBot_favicon_512x512.png" );
    mDefaultBgImg = OpenCVTools::QImageToCvMat( img, true );
    //ui->widget_video_container->scene()->setBgImage( mDefaultBgImg );

    mFirstResize = true;
}

CMainWindow::~CMainWindow()
{
    if(mRoboCtrl)
        delete mRoboCtrl;

    if(mWebcamClient)
        delete mWebcamClient;

    delete ui;
}

bool CMainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::FocusIn)
    {
        QTimer::singleShot(0, obj, SLOT(selectAll()));
        return false;
    }
    else
    {
        return QObject::eventFilter(obj, event);
    }
}

void CMainWindow::timerEvent( QTimerEvent* event )
{
    if( event->timerId() == mSpeedSendTimer )
    {
        if(!mRoboCtrl)
            return;

        //qDebug() << Q_FUNC_INFO <<  "mSpeedSendTimer";

        float scale = ui->widget_joypad->getMaxAbsAxisValue();

        if(mPidEnabled) // Speed commands to be sent
        {
            try
            {
                if( mJoyMot[0] != mLastJoyMot[0] || mJoyMot[1] != mLastJoyMot[1] )
                {
                    // >>>>> Taking robot control
                    if( !mHasRobotCtrl )
                        mRoboCtrl->getRobotControl();
                    // <<<<< Taking robot control

                    double speed0 = mJoyMot[0]/scale*mMaxMotorSpeed;
                    double speed1 = mJoyMot[1]/scale*mMaxMotorSpeed;


                    {
                        mRoboCtrl->setMotorSpeeds( speed0, speed1 );
                        mLastJoyMot[0] = mJoyMot[0];
                        mLastJoyMot[1] = mJoyMot[1];
                    }

                    mSpeedRequested = true; // Must be set because setMotorSpeeds replies on Status UDP with the speeds of the motors
                }

            }
            catch( RcException &e)
            {
                qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
            }
        }
        else // PWM commands to be sent
        {
            try
            {
                if( mJoyMot[0] != mLastJoyMot[0] || mJoyMot[1] != mLastJoyMot[1] )
                {
                    // >>>>> Taking robot control
                    if( !mHasRobotCtrl )
                        mRoboCtrl->getRobotControl();
                    // <<<<< Taking robot control

                    int pwm0 = (int)(mJoyMot[0]/scale*2047.0f+0.5f);
                    int pwm1 = (int)(mJoyMot[1]/scale*2047.0f+0.5f);

                    mRoboCtrl->setMotorPWMs( pwm0, pwm1 );
                    mLastJoyMot[0] = mJoyMot[0];
                    mLastJoyMot[1] = mJoyMot[1];
                }
            }
            catch( RcException &e)
            {
                qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
            }
        }
    }
    else if( event->timerId() == mFrameReqTimer )
    {
        if( !mWebcamClient )
            return;

        //qDebug() << Q_FUNC_INFO <<  "mFrameReqTimer";

        if( mWebcamClient!=NULL )
        {
            quint64 frmCount,frmComplete;
            mWebcamClient->getStats( frmCount, frmComplete );

            mVideoStats->setText( tr("Video: %1/%2").arg(frmComplete).arg(frmCount) );

            if( mNewImageAvailable )
            {
                mNewImageAvailable = false;
                cv::Mat frame = mWebcamClient->getLastFrame();

                ui->widget_video_container->scene()->setBgImage( frame );

                if( mDefaultBgImg.rows!=frame.rows || mDefaultBgImg.cols!=frame.cols  )
                {
                    mDefaultBgImg = frame;
                    ui->widget_video_container->fitInView(QRectF(0,0, frame.cols, frame.rows),
                                                          Qt::KeepAspectRatio );
                }
            }
        }
    }
    else if( event->timerId() == mStatusReqTimer )
    {
        if(!mRoboCtrl)
            return;

        //qDebug() << Q_FUNC_INFO <<  "mStatusReqTimer";

        try
        {
            mRoboCtrl->getBoardStatus();
        }
        catch( RcException &e)
        {
            qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
        }

        if( mNewTelemetryAvailable )
        {
            mNewTelemetryAvailable = false;
            mRoboCtrl->getLastTelemetry( mTelemetry );

            updateBatteryInfo();
            updateSpeedInfo();
        }
    }
}

void CMainWindow::resizeEvent(QResizeEvent * ev)
{
    QMainWindow::resizeEvent( ev );

    int jw = COMMON->mScreen.cvtMm2Px(30); // Joypad sized 3 cm
    ui->widget_joypad->setFixedWidth( jw );
    ui->widget_joypad->setFixedHeight( jw );

    if( !mDefaultBgImg.empty() )
        ui->widget_video_container->fitInView( QRectF(0,0, mDefaultBgImg.cols, mDefaultBgImg.rows ),
                                               Qt::KeepAspectRatio );

    ui->widget_video_container->setJoypadSize( QSize(jw,jw),
                                               QSize(jw/2,jw/2) );

    if( !mDefaultBgImg.empty() && mFirstResize )
    {
        mFirstResize = false;
        ui->widget_video_container->scene()->setBgImage(mDefaultBgImg);
    }

}

void CMainWindow::onFindServerButtonClicked()
{
    QString serverIp = RoboControllerSDK::findServer( DEFAULT_UDP_STAT_PORT_SEND );

    if( serverIp.isEmpty() )
    {
        mRobIpLineEdit->setText( tr("No Robot Server found. Enter IP manually.") );
        return;
    }

    mRobIpLineEdit->setText( serverIp );
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question( this, tr("Robot Server found"),
                                   tr("Do you want to estabilish\na connection?") );

    if( reply==QMessageBox::Yes )
        onConnectButtonClicked();
}

void CMainWindow::onConnectButtonClicked()
{
    try
    {
        if(mRoboCtrl)
            delete mRoboCtrl;

        mRobIpAddress = mRobIpLineEdit->text();

        mRoboCtrl = new RoboControllerSDK( mRobIpAddress, mRobUdpStatusPortSend, mRobUdpStatusPortListen,
                                           mRobUdpTelemetryPortListen, mRobUdpControlPort, mRobTcpPort );

        mIniSettings->setValue( ROB_IP, mRobIpAddress );
        mIniSettings->setValue( ROB_TCP_PORT, mRobTcpPort );
        mIniSettings->setValue( ROB_UDP_STAT_PORT_SEND, mRobUdpStatusPortSend );
        mIniSettings->setValue( ROB_UDP_STAT_PORT_LISTEN, mRobUdpStatusPortListen );
        mIniSettings->setValue( ROB_UDP_TELEM_PORT_LISTEN, mRobUdpTelemetryPortListen );
        mIniSettings->setValue( ROB_UDP_CTRL_PORT, mRobUdpControlPort );
        mIniSettings->sync();

        mJoyMot[0] = 0.0;
        mJoyMot[1] = 0.0;

        mLastJoyMot[0] = 0.0;
        mLastJoyMot[1] = 0.0;
    }
    catch( RcException &e)
    {
        QMessageBox::critical(  this, tr("Connection error"), QString(e.getExcMessage()) );
        return;
    }

    // >>>>> Signals/Slots connections
    connect( mRoboCtrl, SIGNAL(newRobotConfiguration(RobotConfiguration&)) ,
             this, SLOT(onNewRobotConfiguration(RobotConfiguration&)) );
    connect( mRoboCtrl, SIGNAL(newBoardStatus(BoardStatus&)),
             this, SLOT(onNewBoardStatus(BoardStatus&)) );
    connect( mRoboCtrl, SIGNAL(newTelemetryAvailable()),
             this, SLOT(onNewTelemetryAvailable() ));

    connect( mRoboCtrl, SIGNAL(robotControlTaken()),
             this, SLOT(onRobotControlTaken()) );
    connect( mRoboCtrl, SIGNAL(robotControlReleased()),
             this, SLOT(onRobotControlReleased()) );
    connect( mRoboCtrl, SIGNAL(robotControlNotTaken()),
             this, SLOT(onRobotControlNotTaken()) );

    // <<<<< Signals/Slots connections

    // >>>>> Setting last PID state
    BoardStatus status;

    status.accelRampEnable = false;
    status.pidEnable = mPidEnabled;
    status.saveToEeprom = true;
    status.wdEnable = true;

    mSpeedRequested = false;
    mMotorSpeedLeftValid=false;
    mMotorSpeedRightValid=false;
    mMotorSpeedRight=0;
    mMotorSpeedLeft=0;

    try
    {
        mRoboCtrl->setBoardStatus( status );
    }
    catch( RcException &e)
    {
        qWarning() << tr("setBoardStatus Exception error: %1").arg(e.getExcMessage() );
        QMessageBox::warning( this, tr("Communication Error"), tr("There was an error configuring Robot communication.\rPlease try again to connect."));
        return;
    }
    // <<<<< Setting last PID state

    QThread::msleep( 500 );

    // >>>>> Requesting Robot Configuration
    try
    {
        mRoboCtrl->getRobotConfigurationFromEeprom();
    }
    catch( RcException &e)
    {
        qWarning() << tr("getRobotConfigurationFromEeprom Exception error: %1").arg(e.getExcMessage() );
        QMessageBox::warning( this, tr("Communication Error"), tr("There was an error retrieving Robot configuration.\rPlease try again to connect."));
        return;
    }
    // <<<<< Requesting Robot Configuration

    QThread::msleep( 500 );

    // >>>>> Requesting Robot Configuration
    try
    {
        mRoboCtrl->getBoardStatus();
    }
    catch( RcException &e)
    {
        qWarning() << tr("getBoardStatus Exception error: %1").arg(e.getExcMessage() );
        QMessageBox::warning( this, tr("Communication Error"), tr("There was an error retrieving RoboController configuration.\rPlease try again to connect."));
        return;
    }
    // <<<<< Requesting Robot Configuration


    // >>>>> Taking robot control
    mRoboCtrl->getRobotControl();
    // <<<<< Taking robot control

    ui->actionRobot_Configuration->setEnabled(true);
    ui->actionBattery_Calibration->setEnabled(true);
    mStatusLabel->setText( tr("Connected to robot on IP: %1").arg(mRobIpAddress) );

    mPushButtonFindServer->setEnabled(false);
    mPushButtonConnect->setEnabled(false);

    // >>>>> Webcam Client
    QCoreApplication::processEvents( QEventLoop::AllEvents, 1500 );
    QThread::msleep(500);

    mWebcamClient = new QWebcamClient( /*mRobIpAddress,*/ 55554, 55555, this );

    connect( mWebcamClient, SIGNAL(newImageReceived()),
             this, SLOT(onNewImage()) );
    // <<<<< Webcam Client

    startTimers();
}

void CMainWindow::updateSpeedInfo( )
{
    if(mRobotConfigValid)
    {
        mMotorSpeedLeft = mTelemetry.LinSpeedLeft;
        mMotorSpeedLeftValid = true;
        mMotorSpeedRight = mTelemetry.LinSpeedLeft;
        mMotorSpeedRightValid = true;

        mSpeedRequested = false;

        double fwSpeed = mMotorSpeedLeft+mMotorSpeedRight/2.0;
        ui->lcdNumber_fw_speed->display( fwSpeed ); // m/sec

        double rotSpeed = (mMotorSpeedLeft-mMotorSpeedRight)/(mRoboConf.WheelBase/1000.0); // Wheelbase is in mm
        ui->lcdNumber_rot_speed->display( rotSpeed*RAD2DEG ); // deg/sec
    }
    else
    {
        ui->lcdNumber_fw_speed->display("------");
        ui->lcdNumber_rot_speed->display("------");
    }
}

void CMainWindow::onNewJoypadValues(float x, float y)
{
    //qDebug() << tr("Joypad: (%1,%2)").arg(x).arg(y);

    mJoyMot[0] = y + x; // Left Motor
    mJoyMot[1] = y - x; // Right Motor
}

void CMainWindow::onNewBoardStatus(BoardStatus& status)
{
    QPalette pal;
    pal = ui->widget_PID_status->palette();

    if(status.pidEnable)
        pal.setColor( QPalette::Window, Qt::green );
    else
        pal.setColor( QPalette::Window, Qt::red );
    ui->widget_PID_status->setPalette( pal );

    if(status.accelRampEnable)
        pal.setColor( QPalette::Window, Qt::green );
    else
        pal.setColor( QPalette::Window, Qt::red );
    ui->widget_ramp_status->setPalette( pal );

    if(status.saveToEeprom)
        pal.setColor( QPalette::Window, Qt::green );
    else
        pal.setColor( QPalette::Window, Qt::red );
    ui->widget_save_EEPROM_status->setPalette( pal );

    if(status.wdEnable)
        pal.setColor( QPalette::Window, Qt::green );
    else
        pal.setColor( QPalette::Window, Qt::red );
    ui->widget_WD_status->setPalette( pal );
}

void CMainWindow::onRobotControlTaken()
{
    QPalette pal;
    pal = ui->widget_robot_ctrl->palette();

    pal.setColor( QPalette::Window, Qt::green );
    ui->widget_robot_ctrl->setPalette( pal );

    mHasRobotCtrl = true;
}

void CMainWindow::onRobotControlReleased()
{
    QPalette pal;
    pal = ui->widget_robot_ctrl->palette();

    pal.setColor( QPalette::Window, Qt::gray );
    ui->widget_robot_ctrl->setPalette( pal );

    mHasRobotCtrl = false;
}

void CMainWindow::onRobotControlNotTaken()
{
    QPalette pal;
    pal = ui->widget_robot_ctrl->palette();

    pal.setColor( QPalette::Window, Qt::red );
    ui->widget_robot_ctrl->setPalette( pal );

    mHasRobotCtrl = false;
}

void CMainWindow::on_actionPidEnabled_triggered()
{
    mPidEnabled = ui->actionPidEnabled->isChecked();

    mIniSettings->setValue( PID_ENABLED, mPidEnabled );
    mIniSettings->sync();

    if(!mRoboCtrl)
        return;

    BoardStatus status;

    status.accelRampEnable = false;
    status.pidEnable = mPidEnabled;
    status.saveToEeprom = true;
    status.wdEnable = true;

    try
    {
        mRoboCtrl->setBoardStatus( status );
    }
    catch( RcException &e)
    {
        qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
    }
}

void CMainWindow::stopTimers()
{
    this->killTimer( mSpeedSendTimer );
    this->killTimer( mStatusReqTimer );
    this->killTimer( mFrameReqTimer );
}

void CMainWindow::startTimers()
{
#ifndef ANDROID
    mSpeedSendTimer = this->startTimer( 30, Qt::PreciseTimer );
    mStatusReqTimer = this->startTimer( 50, Qt::CoarseTimer );
    mFrameReqTimer = this->startTimer( 100, Qt::PreciseTimer );
#else
    mSpeedSendTimer = this->startTimer( 60, Qt::PreciseTimer );
    mStatusReqTimer = this->startTimer( 100, Qt::CoarseTimer );
    mFrameReqTimer = this->startTimer( 200, Qt::PreciseTimer );
#endif
}

void CMainWindow::on_actionRobot_Configuration_triggered()
{
    if(!mRoboCtrl)
        return;

    stopTimers();

    mOpenRobotConfig = true;

    try
    {
        mRoboCtrl->getRobotConfigurationFromEeprom();
    }
    catch( RcException &e)
    {
        qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
    }
}

void CMainWindow::onNewRobotConfiguration( RobotConfiguration& robConf )
{
    memcpy( &mRoboConf, &robConf, sizeof(RobotConfiguration) );

    mRobotConfigValid = true;

    if( mOpenRobotConfig )
    {
        mOpenRobotConfig = false;

        QRobotConfigDialog dlg(mRoboConf,this);

#ifdef ANDROID
        dlg.setWindowState(dlg.windowState() | Qt::WindowMaximized);
#endif

        int res = dlg.exec();

        if( res==QDialog::Accepted )
        {
            dlg.getRobotConfiguration( mRoboConf );
            try
            {
                mRoboCtrl->setRobotConfiguration( mRoboConf );
                mRoboCtrl->saveRobotConfigurationToEeprom();
            }
            catch( RcException &e)
            {
                qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
            }
        }

        startTimers();
    }

    //mStatusBattLevelProgr->setRange( mRoboConf.MinChargedBatteryLevel, mRoboConf.MaxChargedBatteryLevel );
}

void CMainWindow::updateBatteryInfo( )
{
    mBatteryLabel->setText( tr("Battery: %1V/%2V")
                            .arg( mTelemetry.Battery, 5,'f', 2, ' ' )
                            .arg( (double)(mRoboConf.MaxChargedBatteryLevel)/100.0f, 5,'f', 2, ' ' ));

    double perc = 100.0*(mTelemetry.Battery-((double)mRoboConf.MinChargedBatteryLevel/100.0))/
            ((double)(mRoboConf.MaxChargedBatteryLevel-mRoboConf.MinChargedBatteryLevel)/100.0);

    perc = qMin( perc, 100.0 );

    //qDebug() << tr("Battery: %2V %1%").arg(perc).arg(battVal);

    mStatusBattLevelProgr->setValue(perc);

    mStatusBattLevelProgr->setProperty("defaultStyleSheet",
                                       mStatusBattLevelProgr->styleSheet());

    if(perc<=10.0)
    {
        mStatusBattLevelProgr->setStyleSheet(mStatusBattLevelProgr->property("defaultStyleSheet").toString() +
                                             " QProgressBar::chunk { background: red; }");
    }
    else if(perc<=25)
    {
        mStatusBattLevelProgr->setStyleSheet(mStatusBattLevelProgr->property("defaultStyleSheet").toString() +
                                             " QProgressBar::chunk { background: orange; }");
    }
    else
    {
        mStatusBattLevelProgr->setStyleSheet(mStatusBattLevelProgr->property("defaultStyleSheet").toString() +
                                             " QProgressBar::chunk { background: green; }");    }
}

void CMainWindow::onNewImage()
{
    mNewImageAvailable = true;
}

void CMainWindow::onNewTelemetryAvailable()
{
    //qDebug() << PREFIX << tr("New Telemetry available");

    mNewTelemetryAvailable = true;
}

void CMainWindow::on_actionBattery_Calibration_triggered()
{
    if(!mRoboCtrl)
        return;

    QBatteryCalibDialog dlg(mRoboCtrl, this);

#ifdef ANDROID
    dlg.setWindowState(dlg.windowState() | Qt::WindowMaximized);
#endif

    dlg.exec();
}
