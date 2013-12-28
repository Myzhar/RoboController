#include "cmainwindow.h"
#include "ui_cmainwindow.h"

#include <QMessageBox>
#include "macros.h"
#include "cselectipdlg.h"
#include <network_msg.h>
#include <modbus_registers.h>
#include <exception.h>
#include "cconfiginidialog.h"

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow),
    mTcpServerPort(4500),
    mTcpServerAddr(tr("localhost")),
    mSettings(INI_FILE,QSettings::IniFormat),
    mRcComm(NULL)
{
    ui->setupUi(this);

    ui->combo_motor_idx->addItem( "Motor 0");
    ui->combo_motor_idx->addItem( "Motor 1");

    mCurrMotorIdx = 0;
    mUpdateTimeMsec = 50;

    // >>>>> Plot Configuration
    ui->widget_plot->addGraph(); // Graph 0: SetPoint
    ui->widget_plot->graph(0)->setName( tr("SetPoint") );
    ui->widget_plot->addGraph(); // Graph 1: Motor Value
    ui->widget_plot->graph(1)->setName( tr("Motor Value") );
    ui->widget_plot->addGraph(); // Graph 2: Error
    ui->widget_plot->graph(2)->setName( tr("Error") );

    QPen pen0,pen1,pen2;
    pen0.setColor( QColor(0,255,0,180) );
    pen0.setWidth(5);
    pen1.setColor( QColor(0,0,255,180) );
    pen1.setWidth(3);
    pen2.setColor( QColor(255,0,0,255) );
    pen2.setWidth(1);
    ui->widget_plot->graph(0)->setPen( pen0 );
    ui->widget_plot->graph(1)->setPen( pen1 );
    ui->widget_plot->graph(2)->setPen( pen2 );

    ui->widget_plot->setupFullAxesBox();

    ui->widget_plot->setRangeDrag(Qt::Horizontal | Qt::Vertical);
    ui->widget_plot->setRangeZoom(Qt::Horizontal | Qt::Vertical);

    ui->widget_plot->legend->setVisible(true);
    ui->widget_plot->legend->setFont(QFont("Helvetica",8));
    ui->widget_plot->xAxis->setLabel( tr("Time (msec)") );


    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->widget_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->widget_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_plot->yAxis2, SLOT(setRange(QCPRange)));

    mGraphRange = 2000.0;
    ui->widget_plot->xAxis->setRange(0,mGraphRange);
    ui->widget_plot->yAxis->setRange(-1,1);
    // <<<<< Plot Configuration

    // Data Timer
    connect(&mDataTimer, SIGNAL(timeout()), this, SLOT(onDataTimerTimeout()));
}

CMainWindow::~CMainWindow()
{
    delete ui;
}

void CMainWindow::on_verticalSlider_SetPoint_valueChanged(int value)
{
    try
    {
        if(ui->checkBox_PID_enable->isChecked())
        {
            double setPoint = (double)value/100.0;
            ui->lineEdit_setPoint->setText( tr("%1").arg(setPoint,0,'g'));
            mSetPoint[mCurrMotorIdx] = setPoint;

            int tooltipVal;
            if(setPoint >= 0)
                tooltipVal = (quint16)(setPoint*1000.0);
            else
                tooltipVal = (quint16)(setPoint*1000.0+65536.0);

            QString tooltip = tr("0x%1").arg( tooltipVal, 4, 16, QChar('0') );
            ui->lineEdit_setPoint->setToolTip( tooltip );
            ui->verticalSlider_SetPoint->setToolTip( tooltip);

            if(mRcComm)
                mRcComm->setMotorSpeed( mCurrMotorIdx, setPoint );
        }
        else
        {
            ui->lineEdit_setPoint->setText( tr("%1").arg(value) );

            QString tooltip = tr("0x%1").arg( value, 4, 16, QChar('0') );
            ui->lineEdit_setPoint->setToolTip( tooltip );
            ui->verticalSlider_SetPoint->setToolTip( tooltip);

            if(mRcComm)
                mRcComm->setMotorPWM( mCurrMotorIdx, value );
        }
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::on_verticalSlider_Kp_valueChanged(int value)
{
    ui->lineEdit_Kp->setText( tr("%1").arg(value));

    QString tooltip = tr("0x%1").arg( value, 4, 16, QChar('0') );
    ui->lineEdit_Kp->setToolTip( tooltip );
    ui->verticalSlider_Kp->setToolTip( tooltip);

    mKp[mCurrMotorIdx] = value;
}

void CMainWindow::on_verticalSlider_Ki_valueChanged(int value)
{
    ui->lineEdit_Ki->setText( tr("%1").arg(value));

    QString tooltip = tr("0x%1").arg( value, 4, 16, QChar('0') );
    ui->lineEdit_Ki->setToolTip( tooltip );
    ui->verticalSlider_Ki->setToolTip( tooltip);

    mKi[mCurrMotorIdx] = value;
}

void CMainWindow::on_verticalSlider_Kd_valueChanged(int value)
{
    ui->lineEdit_Kd->setText( tr("%1").arg(value));

    QString tooltip = tr("0x%1").arg( value, 4, 16, QChar('0') );
    ui->lineEdit_Kd->setToolTip( tooltip );
    ui->verticalSlider_Kd->setToolTip( tooltip);

    mKd[mCurrMotorIdx] = value;
}

void CMainWindow::on_pushButton_SetPoint_clicked()
{
    bool ok;
    double value = ui->lineEdit_setPoint->text().toDouble( &ok );

    if(!ok)
    {
        QMessageBox::critical( this, tr("Error"),
                               tr("Please verify SetPoint value") );
        return;
    }

    int sliderVal;

    bool pidEnabled = ui->checkBox_PID_enable->isChecked() ;
    if( pidEnabled )
    {
        if(value>=0)
            sliderVal = ((int)(value*100.0+0.5));
        else
            sliderVal = ((int)(value*100.0-0.5));

        ui->verticalSlider_SetPoint->setValue(sliderVal);
    }

    mSetPoint[mCurrMotorIdx] = value;

    try
    {
        if(mRcComm)
        {
            if( pidEnabled )
                mRcComm->setMotorSpeed( mCurrMotorIdx, value );
            else
                mRcComm->setMotorPWM( mCurrMotorIdx, value );
        }
    }
    catch( RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::on_pushButton_SetGains_clicked()
{
    updateGains();
}

void CMainWindow::updateGains()
{
    bool ok;

    // >>>>> Kp value
    int value = ui->lineEdit_Kp->text().toInt( &ok );
    if(!ok)
    {
        QMessageBox::critical( this, tr("Error"),
                               tr("Please verify Kp value") );
        return;
    }

    ui->verticalSlider_Kp->setValue(value);
    mKp[mCurrMotorIdx] = value;
    // <<<<< Kp value

    // >>>>> Ki value
    value = ui->lineEdit_Ki->text().toInt( &ok );
    if(!ok)
    {
        QMessageBox::critical( this, tr("Error"),
                               tr("Please verify Ki value") );
        return;
    }

    ui->verticalSlider_Ki->setValue(value);
    mKi[mCurrMotorIdx] = value;
    // <<<<< Ki value

    // >>>>> Kd value
    value = ui->lineEdit_Kd->text().toInt( &ok );
    if(!ok)
    {
        QMessageBox::critical( this, tr("Error"),
                               tr("Please verify Kd value") );
        return;
    }

    ui->verticalSlider_Kd->setValue(value);
    mKd[mCurrMotorIdx] = value;
    // <<<<< Kd value

    // >>>>> New K to RoboController
    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    try
    {
        mRcComm->setMotorPidGains( mCurrMotorIdx,
                                   mKp[mCurrMotorIdx],
                                   mKi[mCurrMotorIdx],
                                   mKd[mCurrMotorIdx] );
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
    // <<<<< New K to RoboController
}

void CMainWindow::on_combo_motor_idx_currentIndexChanged(int index)
{
    mCurrMotorIdx = index;

    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    try
    {
        if( index==0 || index==1)
            mRcComm->getMotorPidGains( index );
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::on_pushButton_Reset_clicked()
{
    mTimeVec.clear();
    mSetPointVec.clear();
    mCurrMotorValVec.clear();
    mErrorVec.clear();

    updatePlot();
}

void CMainWindow::updatePlot()
{
    if( mTimeVec.isEmpty() )
    {
        ui->widget_plot->graph(0)->clearData();
        ui->widget_plot->graph(1)->clearData();
        ui->widget_plot->graph(2)->clearData();

        mGraphRange = 5000.0;
        ui->widget_plot->xAxis->setRange(0, mGraphRange );

        ui->widget_plot->replot();
        return;
    }

    quint64 time = mTimeVec.last();
    double setPoint = mSetPointVec.last();
    double motorVal = mCurrMotorValVec.last();
    double error = mErrorVec.last();

    ui->widget_plot->graph(0)->addData( (double)time, (double)setPoint );
    ui->widget_plot->graph(1)->addData( (double)time, (double)motorVal );
    ui->widget_plot->graph(2)->addData( (double)time, (double)error );

    ui->widget_plot->graph(0)->removeDataBefore( (double)time-mGraphRange );
    ui->widget_plot->graph(1)->removeDataBefore( (double)time-mGraphRange );
    ui->widget_plot->graph(2)->removeDataBefore( (double)time-mGraphRange );

    ui->widget_plot->graph(0)->rescaleValueAxis(true);
    ui->widget_plot->graph(1)->rescaleValueAxis(true);
    ui->widget_plot->graph(2)->rescaleValueAxis(true);

    ui->widget_plot->xAxis->setRange( time+mUpdateTimeMsec*2, mGraphRange, Qt::AlignRight);

    ui->widget_plot->replot();
}

void CMainWindow::on_pushButton_StartStop_clicked()
{
    if( ui->pushButton_StartStop->isChecked() )
    {
        ui->pushButton_Reset->setEnabled( false );
        ui->pushButton_StartStop->setText( tr("Stop Acquiring"));

        mSetPoint[mCurrMotorIdx] = ui->verticalSlider_SetPoint->value();
        mKp[mCurrMotorIdx] = ui->verticalSlider_Kp->value();
        mKi[mCurrMotorIdx] = ui->verticalSlider_Ki->value();
        mKd[mCurrMotorIdx] = ui->verticalSlider_Kd->value();

        // >>>>> Send parameters to RoboController
        if(!mRcComm)
        {
            qWarning() << tr("SDK not connected to Server");
            return;
        }

        try
        {
            mRcComm->setMotorPidGains( mCurrMotorIdx,
                                       mKp[mCurrMotorIdx],
                                       mKi[mCurrMotorIdx],
                                       mKd[mCurrMotorIdx] );
        }
        catch( roboctrl::RcException &e)
        {
            onSdkServerDisconnected();
            QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
        }
        // <<<<< Send parameters to RoboController

        mStartTime = QDateTime::currentDateTime().toMSecsSinceEpoch();
        mDataTimer.start(mUpdateTimeMsec);
    }
    else
    {
        mDataTimer.stop();
        ui->pushButton_Reset->setEnabled( true );
        ui->pushButton_StartStop->setText( tr("Start Acquiring"));

        ui->widget_plot->graph(0)->clearData();
        ui->widget_plot->graph(1)->clearData();
        ui->widget_plot->graph(2)->clearData();

        ui->widget_plot->graph(0)->addData( mTimeVec, mSetPointVec );
        ui->widget_plot->graph(1)->addData( mTimeVec, mCurrMotorValVec );
        ui->widget_plot->graph(2)->addData( mTimeVec, mErrorVec );
    }
}

void CMainWindow::onDataTimerTimeout()
{
    requestCurrentMotorVal();
}


void CMainWindow::onNewMotorSpeedValue( quint16 motorIdx, double value )
{
    // >>>>> Data Update
    quint64 time = QDateTime::currentDateTime().toMSecsSinceEpoch() - mStartTime;

    double error = mSetPoint[motorIdx] - value;
    // <<<<< Data Update

    if( ui->pushButton_StartStop->isChecked() )
    {
        mTimeVec << (double)time;
        mCurrMotorValVec << value;
        mSetPointVec << mSetPoint[motorIdx];
        mErrorVec << error;

        updatePlot();

        qDebug() << tr( "Speed: %1 m/sec" ).arg(value);
    }
}

void CMainWindow::onNewMotorPIDGains( quint16 motorIdx,
                                      quint16 Kp, quint16 Ki, quint16 Kd )
{
    // >>>>> Data Update
    mKp[motorIdx] = Kp;
    mKi[motorIdx] = Ki;
    mKd[motorIdx] = Kd;
    // <<<<< Data Update

    // >>>>> GUI Update
    if( motorIdx == ui->combo_motor_idx->currentIndex() )
    {
        ui->lineEdit_Kp->setText( tr("%1").arg(Kp));
        ui->lineEdit_Ki->setText( tr("%1").arg(Ki));
        ui->lineEdit_Kd->setText( tr("%1").arg(Kd));

        ui->verticalSlider_Kp->setValue( Kp );
        ui->verticalSlider_Ki->setValue( Ki );
        ui->verticalSlider_Kd->setValue( Kd );
    }
    // <<<<< GUI Update
}

void CMainWindow::requestCurrentMotorVal()
{
    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    try
    {
        mRcComm->getMotorSpeed( (quint16)mCurrMotorIdx );
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::on_action_Connect_TCP_Server_triggered()
{
    // >>>>> IP Dialog
    CSelectIpDlg ipDlg(this);
    ipDlg.setModal(true);
    ipDlg.exec();

    if(ipDlg.result()==QDialog::Rejected)
        return;
    // <<<<< IP Dialog

    // >>>>> IP from INI file
    mSettings.beginGroup( QString("SERVER_INFO"));
    {
        mTcpServerAddr = mSettings.value( "Server_address", tr("") ).toString();
        if( mTcpServerAddr.compare("")==0 )
        {
            QMessageBox::critical( this,
                                   tr("Error"),
                                   tr("Unable to retrieve Server Address from %1 configuration file").arg(INI_FILE) );
            return;
        }

        mTcpServerPort = mSettings.value( "Server_port", "0" ).toUInt();
        if( mTcpServerPort==0 )
        {
            QMessageBox::critical( this,
                                   tr("Error"),
                                   tr("Unable to retrieve Server Port from %1 configuration file").arg(INI_FILE));
            return;
        }
    }
    mSettings.endGroup();
    // <<<<< IP from INI file

    if(mRcComm)
        delete mRcComm;

    try
    {
        mRcComm = new RoboControllerSDK( 4550, 4560, mTcpServerAddr, mTcpServerPort );

        connect( mRcComm, SIGNAL(tcpConnected()),
                 this, SLOT(onSdkServerConnected()));
        connect( mRcComm, SIGNAL(tcpDisconnected()),
                 this, SLOT(onSdkServerDisconnected()));
        connect( mRcComm, SIGNAL(newMotorSpeedValue(quint16,double)),
                 this, SLOT(onNewMotorSpeedValue(quint16,double)));
        connect( mRcComm, SIGNAL(newMotorPIDGains(quint16,quint16,quint16,quint16)),
                 this, SLOT(onNewMotorPIDGains(quint16,quint16,quint16,quint16)) );
        connect( mRcComm, SIGNAL(newRobotConfiguration(RobotConfiguration&)),
                 this, SLOT(onNewRobotConfiguration(RobotConfiguration&)) );
        connect( mRcComm, SIGNAL(newBoardStatus(BoardStatus&)),
                 this, SLOT(onNewBoardStatus(BoardStatus&)) );
    }
    catch(RcException &e)
    {
        if(e.GetType()==excTcpNotConnected || e.GetType()==extTcpConnectionRefused)
        {
            QMessageBox::critical( this, tr("Error"),
                                   tr("Unable to connect to Server:\r\n%1")
                                   .arg( e.getExcMessage() ) );
            onSdkServerDisconnected();
            return;
        }
    }

    onSdkServerConnected();
}

void CMainWindow::onSdkServerConnected()
{
    ui->action_Connect_TCP_Server->setEnabled( false );
    ui->action_Disconnect->setEnabled( true );
    // TODO ui->actionConfig_File_to_EEPROM->setEnabled( true );
    ui->actionEEPROM_to_Config_File->setEnabled( true );

    ui->groupBox_controls->setEnabled( true );
    ui->groupBox_options->setEnabled( true );

    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    try
    {
        mRcComm->setCommMode( cmConfiguration );
        mRcComm->getMotorPidGains( 0 );
        mRcComm->getMotorPidGains( 1 );
        mRcComm->getBoardStatus();
        //mRcComm->getRobotConfigurationFromEeprom();
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::onSdkServerDisconnected()
{
    disconnectServer();

    // TODO ui->actionConfig_File_to_EEPROM->setEnabled( false );
    ui->actionEEPROM_to_Config_File->setEnabled( false );

    ui->groupBox_controls->setEnabled( false );
    ui->groupBox_options->setEnabled( false );

    ui->action_Connect_TCP_Server->setEnabled( true );
    ui->action_Disconnect->setEnabled( false );
}

void CMainWindow::on_action_Disconnect_triggered()
{
    onSdkServerDisconnected();
}

void CMainWindow::disconnectServer()
{
    if(mRcComm)
        delete mRcComm;
    mRcComm = NULL;
}

void CMainWindow::on_actionEEPROM_to_Config_File_triggered()
{
    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    mConfToIni = true;
    mConfToEeprom = false;


    try
    {
        mRcComm->getRobotConfigurationFromEeprom();
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::on_actionConfig_File_to_EEPROM_triggered()
{
    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    mConfToIni = false;
    mConfToEeprom = true;

    try
    {
        mRcComm->getRobotConfigurationFromIni();
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::onNewBoardStatus(BoardStatus& status)
{
    ui->checkBox_PID_enable->setChecked( status.pidEnable );
    ui->checkBox_ramps_enable->setChecked( status.accelRampEnable );
    ui->checkBox_save_eeprom_enable->setChecked( status.saveToEeprom );
    ui->checkBox_WD_enable->setChecked( status.wdEnable );

    if(status.pidEnable)
    {
        ui->verticalSlider_SetPoint->setRange(-100, 100 );
        ui->verticalSlider_SetPoint->setTickInterval(10);
        ui->verticalSlider_SetPoint->setSingleStep(5);
        ui->label_SetPoint->setText( tr("SetPoint"));
    }
    else
    {
        ui->verticalSlider_SetPoint->setRange(-2048, 2047 );
        ui->verticalSlider_SetPoint->setTickInterval(100);
        ui->verticalSlider_SetPoint->setSingleStep(10);
        ui->label_SetPoint->setText( tr("PWM"));
    }
}

void CMainWindow::onNewRobotConfiguration( RobotConfiguration& robConf )
{
    if(!mRcComm)
    {
        qWarning() << tr("SDK not connected to Server");
        return;
    }

    try
    {
        if(mConfToIni)
            mRcComm->saveRobotConfigurationToIni();
        else if(mConfToEeprom)
            mRcComm->saveRobotConfigurationToEeprom();

        mConfToIni = false;
        mConfToEeprom = false;
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}

void CMainWindow::on_actionRead_Config_File_triggered()
{
    QFile file(ROBOT_CONFIG_INI_FILE);

    if(!file.exists())
    {
        QMessageBox::critical( this, tr("Error"),
                               tr("%1 file does not exist.\r\n " \
                                  "To create one choose \"EEPROM " \
                                  "to Config File\" from \"RobotConfiguration\" menu")
                               .arg(ROBOT_CONFIG_INI_FILE) );
        return;
    }

    CConfigIniDialog dlg( ROBOT_CONFIG_INI_FILE, this );
    dlg.setWindowTitle( tr("Robot Configuration file: %1").arg(ROBOT_CONFIG_INI_FILE) );
    dlg.setModal( true );
    dlg.exec();
}

void CMainWindow::on_checkBox_WD_enable_clicked()
{
    SendOptionsToBoard();
}

void CMainWindow::on_checkBox_PID_enable_clicked()
{
    ui->verticalSlider_SetPoint->setValue(0);
    ui->lineEdit_setPoint->setText( tr("0") );

    SendOptionsToBoard();
}

void CMainWindow::on_checkBox_save_eeprom_enable_clicked()
{
    SendOptionsToBoard();
}

void CMainWindow::on_checkBox_ramps_enable_clicked()
{
    SendOptionsToBoard();
}

void CMainWindow::SendOptionsToBoard()
{
    try
    {
        BoardStatus status;
        status.accelRampEnable = ui->checkBox_ramps_enable->isChecked();
        status.pidEnable = ui->checkBox_PID_enable->isChecked();
        status.saveToEeprom = ui->checkBox_save_eeprom_enable->isChecked();
        status.wdEnable = ui->checkBox_WD_enable->isChecked();

        mRcComm->setBoardStatus( status );

        mRcComm->getBoardStatus();
    }
    catch( roboctrl::RcException &e)
    {
        onSdkServerDisconnected();
        QMessageBox::critical( this, tr("Error"), tr("Communication error: %1").arg(e.getExcMessage()) );
    }
}
