#include "cmainwindow.h"
#include "ui_cmainwindow.h"
#include <QApplication>
#include "exception.h"
#include <QMessageBox>
#include <QLabel>
#include <QDesktopWidget>
#include <QDir>
#include <QScreen>

#include "qrobotconfigdialog.h"
#include "qbatterycalibdialog.h"
#include "qcommon.h"

#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_TCP_PORT 14500
#define DEFAULT_UDP_CTRL_PORT 14560
#define DEFAULT_UDP_STAT_PORT_SEND 14550
#define DEFAULT_UDP_STAT_PORT_LISTEN 14555

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow),
    mIniSettings(NULL),
    mRobIpLineEdit(NULL),
    mPushButtonConnect(NULL),
    mPushButtonFindServer(NULL),
    mRoboCtrl(NULL)
{
    ui->setupUi(this);

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
    // <<<<< Robot IP Address from INI File

    // >>>>> PID State from INI File
    mPidEnabled = mIniSettings->value( PID_ENABLED, false ).toBool();

    ui->actionPidEnabled->setChecked( mPidEnabled );
    // <<<<< PID State from INI File

    // >>>>> Main Toolbar
    QLabel* lab = new QLabel(tr("TCP address:"));
    ui->mainToolBar->addWidget( lab );
    mRobIpLineEdit = new QLineEdit( mRobIpAddress, ui->mainToolBar );
    ui->mainToolBar->addWidget( mRobIpLineEdit );

    mRobIpLineEdit->installEventFilter(this);

    mRobIpLineEdit->setInputMethodHints(Qt::ImhUrlCharactersOnly);

    mPushButtonConnect = new QPushButton( tr("Connect"), ui->mainToolBar );
    ui->mainToolBar->addWidget( mPushButtonConnect );

    mPushButtonFindServer = new QPushButton( tr("Find Server"), ui->mainToolBar );
    ui->mainToolBar->addWidget( mPushButtonFindServer );
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
    // <<<<< Status Bar

    connect( mPushButtonConnect, SIGNAL(clicked()),
             this, SLOT(onConnectButtonClicked()) );
    connect( mPushButtonFindServer, SIGNAL(clicked()),
             this, SLOT(onFindServerButtonClicked()) );

    connect( ui->widget_joypad, SIGNAL(newJoypadValues(float,float)),
             this, SLOT(onNewJoypadValues(float,float)) );

    mMaxMotorSpeed = 2.0f; // m/sec
    mOpenRobotConfig = false;
    mRobotConfigValid = false;
}

CMainWindow::~CMainWindow()
{
    if(mRoboCtrl)
        delete mRoboCtrl;

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
    if( event->timerId() == mStatusReqTimer )
    {
        // TODO Request battery charge
        try
        {
            mRoboCtrl->getBatteryChargeValue();
        }
        catch( RcException &e)
        {
            qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
        }
    }
    else if( event->timerId() == mSpeedReqTimer )
    {
        try
        {
            mRoboCtrl->getMotorSpeed( 0 );
            mRoboCtrl->getMotorSpeed( 1 );
        }
        catch( RcException &e)
        {
            qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
        }
    }
}

void CMainWindow::resizeEvent(QResizeEvent * ev)
{
    QMainWindow::resizeEvent( ev );

    int jw = COMMON->mScreen.cvtMm2Px(30); // Joypad sized 3 cm
    ui->widget_joypad->setFixedWidth( jw );
    ui->widget_joypad->setFixedHeight( jw );

    int fontPx = COMMON->mScreen.cvtMm2Px( 3 );
    QFont font = this->font();
    font.setPixelSize( fontPx );
    this->setFont( font );

    QFont unitFont("Monospace");
    unitFont.setStyleHint(QFont::TypeWriter);
    fontPx = COMMON->mScreen.cvtMm2Px( 3 );
    unitFont.setPixelSize( fontPx );
    ui->label_fw_speed->setFont( unitFont );
    ui->label_rot_speed->setFont( unitFont );
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
                                           mRobUdpControlPort, mRobTcpPort );

        mIniSettings->setValue( ROB_IP, mRobIpAddress );
        mIniSettings->setValue( ROB_TCP_PORT, mRobTcpPort );
        mIniSettings->setValue( ROB_UDP_STAT_PORT_SEND, mRobUdpStatusPortSend );
        mIniSettings->setValue( ROB_UDP_STAT_PORT_LISTEN, mRobUdpStatusPortListen );
        mIniSettings->setValue( ROB_UDP_CTRL_PORT, mRobUdpControlPort );
        mIniSettings->sync();
    }
    catch( RcException &e)
    {
        QMessageBox::critical( this, tr("Connection error"), QString(e.getExcMessage()) );
        return;
    }

    // >>>>> Signals/Slots connections
    connect( mRoboCtrl, SIGNAL(newMotorSpeedValue(quint16,double)),
             this, SLOT(onNewMotorSpeed(quint16,double)) );
    connect( mRoboCtrl, SIGNAL(newRobotConfiguration(RobotConfiguration&)) ,
             this, SLOT(onNewRobotConfiguration(RobotConfiguration&)) );
    connect( mRoboCtrl, SIGNAL(newBatteryValue(double)),
             this, SLOT(onNewBatteryValue(double)) );
    // <<<<< Signals/Slots connections

    // >>>>> Setting last PID state
    BoardStatus status;

    status.accelRampEnable = mPidEnabled;
    status.pidEnable = mPidEnabled;
    status.saveToEeprom = true;
    status.wdEnable = true;

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
        qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
    }
    // <<<<< Setting last PID state

    // >>>>> Requesting Robot Configuration
    try
    {
        mRoboCtrl->getRobotConfigurationFromEeprom();
    }
    catch( RcException &e)
    {
        qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
    }
    // <<<<< Requesting Robot Configuration

    ui->actionRobot_Configuration->setEnabled(true);
    ui->actionBattery_Calibration->setEnabled(true);
    mStatusLabel->setText( tr("Connected to robot on IP: %1").arg(mRobIpAddress) );

    mSpeedReqTimer = this->startTimer( 50, Qt::PreciseTimer);
    mStatusReqTimer = this->startTimer( 500, Qt::CoarseTimer );

    mPushButtonFindServer->setEnabled(false);
    mPushButtonConnect->setEnabled(false);
}

void CMainWindow::onNewMotorSpeed( quint16 mot, double speed )
{
    if(mRobotConfigValid)
    {
        if( mot==0 )
        {
            mMotorSpeedLeft = speed;
            mMotorSpeedLeftValid = true;
        }
        else
        {
            mMotorSpeedRight = speed;
            mMotorSpeedRightValid = true;
        }

        if(mMotorSpeedLeftValid&&mMotorSpeedRightValid)
        {
            double fwSpeed = mMotorSpeedLeft+mMotorSpeedRight/2.0;
            ui->lcdNumber_fw_speed->display( fwSpeed ); // m/sec

            double rotSpeed = (mMotorSpeedLeft-mMotorSpeedRight)/(mRoboConf.WheelBase/1000.0); // Wheelbase is in mm
            ui->lcdNumber_rot_speed->display( rotSpeed*RAD2DEG ); // deg/sec
        }
    }
    else
    {
        ui->lcdNumber_fw_speed->display("------");
        ui->lcdNumber_rot_speed->display("------");
    }
}

void CMainWindow::onNewJoypadValues(float x, float y)
{
    qDebug() << tr("Joypad: (%1,%2)").arg(x).arg(y);

    if(!mRoboCtrl)
        return;

    float motSx = y - x;
    float motDx = y + x;

    float scale = ui->widget_joypad->getMaxAbsAxisValue();

    if(mPidEnabled)
    {
        motSx = motSx/scale*mMaxMotorSpeed;
        motDx = motDx/scale*mMaxMotorSpeed;

        try
        {
            mRoboCtrl->setMotorSpeed(0, motSx);
            mRoboCtrl->setMotorSpeed(1, motDx);
        }
        catch( RcException &e)
        {
            qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
        }

        //qDebug() << tr("Motor speeds: (%1,%2)").arg(motSx).arg(motDx);
    }
    else
    {
        motSx = motSx/scale*2047.0f;
        motDx = motDx/scale*2047.0f;

        try
        {
            mRoboCtrl->setMotorPWM( 0, (int)(motSx+0.5f) );
            mRoboCtrl->setMotorPWM( 1, (int)(motDx+0.5f));
        }
        catch( RcException &e)
        {
            qWarning() << tr("Exception error: %1").arg(e.getExcMessage() );
        }

        //qDebug() << tr("Motor PWMs: (%1,%2)").arg(motSx).arg(motDx);
    }
}

void CMainWindow::on_actionPidEnabled_triggered()
{
    mPidEnabled = ui->actionPidEnabled->isChecked();

    if(!mRoboCtrl)
        return;

    mIniSettings->setValue( PID_ENABLED, mPidEnabled );
    mIniSettings->sync();

    BoardStatus status;

    status.accelRampEnable = mPidEnabled;
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

void CMainWindow::on_actionRobot_Configuration_triggered()
{
    if(!mRoboCtrl)
        return;

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
    }

    mStatusBattLevelProgr->setRange( mRoboConf.MinChargedBatteryLevel, mRoboConf.MaxChargedBatteryLevel );
}

void CMainWindow::onNewBatteryValue( double battVal )
{
    mBatteryLabel->setText( tr("Battery: %1V/%2V")
                            .arg( battVal, 5,'f', 2, ' ' )
                            .arg( (double)(mRoboConf.MaxChargedBatteryLevel)/1000.0f, 5,'f', 2, ' ' ));

    mStatusBattLevelProgr->setValue( (int)(battVal*1000.0) );

    double min = mStatusBattLevelProgr->minimum();
    double max = mStatusBattLevelProgr->maximum();

    double perc = (max-min)/(battVal*1000.0);

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
                                       " QProgressBar::chunk { background: green; }");
    }
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
