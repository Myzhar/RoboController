#include "cmainwindow.h"
#include "ui_cmainwindow.h"
#include <QApplication>
#include "exception.h"
#include <QMessageBox>
#include <QLabel>
#include <QDesktopWidget>
#include <Qdir>
#include <QScreen>

#include "qcommon.h"

#define DEFAULT_IP "localhost"
#define DEFAULT_TCP_PORT 4500

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow),
    mIniSettings(NULL),
    mRobIpLineEdit(NULL),
    mConnectButton(NULL),
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

    ui->mainToolBar->addWidget( new QLabel(tr("Port:")));
    mRobTcpPortLineEdit = new QLineEdit( tr("%1").arg(mRobTcpPort), ui->mainToolBar );
    ui->mainToolBar->addWidget( mRobTcpPortLineEdit );

    mRobIpLineEdit->installEventFilter(this);
    mRobTcpPortLineEdit->installEventFilter(this);

    mRobIpLineEdit->setInputMethodHints(Qt::ImhUrlCharactersOnly);
    mRobTcpPortLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);

    mConnectButton = new QPushButton( tr("Connect"), ui->mainToolBar );
    ui->mainToolBar->addWidget( mConnectButton );
    mConnectButton->setDefault(true);
    mConnectButton->setFocus();
    // <<<<< Main Toolbar

    // >>>>> Status Bar
    mStatusLabel = new QLabel(tr("Unconnected"));
    ui->statusBar->addPermanentWidget( mStatusLabel );
    // <<<<< Status Bar

    connect( mConnectButton, SIGNAL(clicked()),
             this, SLOT(onConnectButtonClicked()) );

    connect( ui->widget_joypad, SIGNAL(newJoypadValues(float,float)),
             this, SLOT(onNewJoypadValues(float,float)) );


    mMaxMotorSpeed = 2.0f; // m/sec

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
    }
    else if( event->timerId() == mSpeedReqTimer )
    {
        mRoboCtrl->getMotorSpeed( 0 );
        mRoboCtrl->getMotorSpeed( 1 );
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
    fontPx = COMMON->mScreen.cvtMm2Px( 2 );
    unitFont.setPixelSize( fontPx );
    ui->label_fw_speed->setFont( unitFont );
    ui->label_rot_speed->setFont( unitFont );
}

void CMainWindow::onConnectButtonClicked()
{
    try
    {
        if(mRoboCtrl)
            delete mRoboCtrl;

        mRobIpAddress = mRobIpLineEdit->text();
        mRobTcpPort = mRobTcpPortLineEdit->text().toInt();
        mRoboCtrl = new RoboControllerSDK( 4550, 4560, mRobIpAddress, mRobTcpPort );

        mIniSettings->setValue( ROB_IP, mRobIpAddress );
        mIniSettings->setValue( ROB_TCP_PORT, mRobTcpPort );
        mIniSettings->sync();
    }
    catch( RcException &e)
    {
        QMessageBox::critical( this, tr("Connection error"), QString(e.getExcMessage()) );
        return;
    }

    // >>>>> Signals/Slots connections
    connect( mRoboCtrl, SIGNAL(newMotorSpeedValue(quint16,double)),
             this, SLOT(onNewMotorSpeed(int,double)) );
    // <<<<< Signals/Slots connections

    // >>>>> Setting last PID state
    BoardStatus status;

    status.accelRampEnable = mPidEnabled;
    status.pidEnable = mPidEnabled;
    status.saveToEeprom = true;
    status.wdEnable = true;

    mRoboCtrl->setBoardStatus( status );
    // <<<<< Setting last PID state

    mStatusLabel->setText( tr("Connected to robot") );

    mSpeedReqTimer = this->startTimer( 50, Qt::PreciseTimer);
    mStatusReqTimer = this->startTimer( 500, Qt::CoarseTimer );
}

void CMainWindow::onNewMotorSpeed( int mot, double speed )
{
    // TODO calculate the forward and rotation speed according to robot parameters
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

        mRoboCtrl->setMotorSpeed(0, motSx);
        mRoboCtrl->setMotorSpeed(1, motDx);

        //qDebug() << tr("Motor speeds: (%1,%2)").arg(motSx).arg(motDx);
    }
    else
    {
        motSx = motSx/scale*2047.0f;
        motDx = motDx/scale*2047.0f;

        mRoboCtrl->setMotorPWM( 0, (int)(motSx+0.5f) );
        mRoboCtrl->setMotorPWM( 1, (int)(motDx+0.5f));

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

    mRoboCtrl->setBoardStatus( status );
}
