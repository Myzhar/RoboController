#include <qbatterycalibdialog.h>
#include <ui_qbatterycalibdialog.h>

namespace roboctrl
{


QBatteryCalibDialog::QBatteryCalibDialog( RoboControllerSDK* robSdk, QWidget *parent ) :
    QDialog(parent),
    ui(new Ui::QBatteryCalibDialog)
{
    ui->setupUi(this);

    connect( robSdk, SIGNAL(newBatteryValue(double)),
             this, SLOT(onNewBatteryValue(double)) );
}

QBatteryCalibDialog::~QBatteryCalibDialog()
{
    delete ui;
}

void QBatteryCalibDialog::onNewBatteryValue(double val )
{
    ui->lineEdit_batt_read_value->setText( tr("%1").arg(val,5,'f',2));
}

void QBatteryCalibDialog::on_pushButton_set_bat_calib_value_clicked()
{

}

void QBatteryCalibDialog::on_pushButton_ok_clicked()
{
    emit accepted();
}


}
