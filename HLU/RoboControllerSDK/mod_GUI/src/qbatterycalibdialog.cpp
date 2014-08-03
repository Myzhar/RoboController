#include <qbatterycalibdialog.h>
#include <ui_qbatterycalibdialog.h>
#include <robocontrollersdk.h>
#include <QMessageBox>

namespace roboctrl
{

QBatteryCalibDialog::QBatteryCalibDialog( RoboControllerSDK* robSdk, QWidget *parent ) :
    QDialog(parent),
    ui(new Ui::QBatteryCalibDialog)
{
    ui->setupUi(this);

    connect( robSdk, SIGNAL(newTelemetryAvailable()),
             this, SLOT(onNewTelemetry()) );

    connect(ui->pushButton_ok, SIGNAL(clicked()),
                     this, SLOT(accept()));

    mRobCom = robSdk;
}

QBatteryCalibDialog::~QBatteryCalibDialog()
{
    delete ui;
}

void QBatteryCalibDialog::onNewTelemetry()
{
    RobotTelemetry telemetry;
    mRobCom->getLastTelemetry( telemetry );
    ui->lcdNumber_value->display( telemetry.Battery );
    qDebug() << telemetry.Battery;
}

void QBatteryCalibDialog::on_pushButton_ok_clicked()
{
    emit accepted();
}

void QBatteryCalibDialog::on_pushButton_set_lower_clicked()
{
    bool ok1;
    double val;
    val = ui->lineEdit_low_calib_value->text().toDouble( &ok1 );

    if(!ok1)
    {
        QMessageBox::warning( this, tr("Error"), tr("Please verify the value and retry") );
        return;
    }

    mRobCom->setBatteryCalibrationParams( CalLow, val );
}


void QBatteryCalibDialog::on_pushButton_set_upper_clicked()
{
    bool ok1;
    double val;
    val = ui->lineEdit_high_calib_value->text().toDouble( &ok1 );

    if(!ok1)
    {
        QMessageBox::warning( this, tr("Error"), tr("Please verify the value and retry") );
        return;
    }

    mRobCom->setBatteryCalibrationParams( CalHigh, val );
}

}
