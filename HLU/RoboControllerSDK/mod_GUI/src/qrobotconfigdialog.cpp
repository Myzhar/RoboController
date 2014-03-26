#include <qrobotconfigdialog.h>
#include <ui_qrobotconfigdialog.h>

//#include <modbus_registers.h>
#include <QMessageBox>

#include <loghandler.h>


//#define REG_COUNT 19

namespace roboctrl
{

QRobotConfigDialog::QRobotConfigDialog( RobotConfiguration& robotConfig, QWidget *parent ) :
    QDialog(parent),
    ui(new Ui::QRobotConfigDialog)
{
    ui->setupUi(this);

    QScreenTools scrT;
    int pxSize = scrT.cvtMm2Px( 5 ); // Vertical scroll bar 5 mm width

    QString barStyle = tr("QScrollBar:horizontal {min-height: %1px;}" "QScrollBar:vertical {min-width: %1px;}").arg(pxSize);
    ui->scrollArea->setStyleSheet(barStyle);

    memcpy( &mRobotConfig, &robotConfig, sizeof(RobotConfiguration) );

    // >>>>> GUI initialization
    if(mRobotConfig.EncoderPosition==Motor)
    {
        ui->pushButton_check_motor->setChecked(true);
        ui->pushButton_check_wheel->setChecked(false);
    } else if(mRobotConfig.EncoderPosition==Wheel)
    {
        ui->pushButton_check_motor->setChecked(false);
        ui->pushButton_check_wheel->setChecked(true);
    }

    if(mRobotConfig.MotorEnableLevel==Low)
    {
        ui->pushButton_check_en_low->setChecked(true);
        ui->pushButton_check_en_high->setChecked(false);
    } else if(mRobotConfig.MotorEnableLevel==High)
    {
        ui->pushButton_check_en_low->setChecked(false);
        ui->pushButton_check_en_high->setChecked(true);
    }

    ui->lineEdit_robot_cpr_left->setText( tr("%1").arg(mRobotConfig.EncoderCprLeft));
    ui->lineEdit_robot_cpr_right->setText( tr("%1").arg(mRobotConfig.EncoderCprRight));
    ui->lineEdit_robot_height->setText( tr("%1").arg(mRobotConfig.Height));
    ui->lineEdit_robot_lenght->setText( tr("%1").arg(mRobotConfig.Lenght));
    ui->lineEdit_robot_max_i_left->setText( tr("%1").arg(mRobotConfig.MaxAmpereMotorLeft));
    ui->lineEdit_robot_max_i_right->setText( tr("%1").arg(mRobotConfig.MaxAmpereMotorRight));
    ui->lineEdit_robot_max_rpm_left->setText( tr("%1").arg(mRobotConfig.MaxRpmMotorLeft));
    ui->lineEdit_robot_max_rpm_right->setText( tr("%1").arg(mRobotConfig.MaxRpmMotorRight));
    ui->lineEdit_robot_max_torque_left->setText( tr("%1").arg(mRobotConfig.MaxTorqueMotorLeft));
    ui->lineEdit_robot_max_torque_right->setText( tr("%1").arg(mRobotConfig.MaxTorqueMotorRight));
    ui->lineEdit_robot_rad_left->setText( tr("%1").arg(mRobotConfig.WheelRadiusLeft));
    ui->lineEdit_robot_rad_right->setText( tr("%1").arg(mRobotConfig.WheelRadiusRight));
    ui->lineEdit_robot_ratio_shaft_left->setText( tr("%1").arg(mRobotConfig.RatioShaftLeft));
    ui->lineEdit_robot_ratio_shaft_right->setText( tr("%1").arg(mRobotConfig.RatioShaftRight));
    ui->lineEdit_robot_ratio_motor_left->setText( tr("%1").arg(mRobotConfig.RatioMotorLeft));
    ui->lineEdit_robot_ratio_motor_right->setText( tr("%1").arg(mRobotConfig.RatioMotorRight));
    ui->lineEdit_robot_weight->setText( tr("%1").arg(mRobotConfig.Weight));
    ui->lineEdit_robot_wheelbase->setText( tr("%1").arg(mRobotConfig.WheelBase));
    ui->lineEdit_robot_width->setText( tr("%1").arg(mRobotConfig.Width));
    ui->lineEdit_robot_max_batt_charge->setText( tr("%1").arg((double)(mRobotConfig.MaxChargedBatteryLevel)/1000.0));
    ui->lineEdit_robot_min_batt_charge->setText( tr("%1").arg((double)(mRobotConfig.MinChargedBatteryLevel)/1000.0));

    ui->lineEdit_robot_cpr_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_cpr_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_height->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_lenght->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_i_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_i_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_rpm_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_rpm_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_torque_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_torque_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_rad_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_rad_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_ratio_shaft_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_ratio_shaft_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_ratio_motor_left->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_ratio_motor_right->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_weight->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_wheelbase->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_width->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_batt_charge->setInputMethodHints(Qt::ImhDigitsOnly);
    ui->lineEdit_robot_max_batt_charge->setInputMethodHints(Qt::ImhDigitsOnly);
    // <<<<< GUI initialization
}

QRobotConfigDialog::~QRobotConfigDialog()
{
    delete ui;
}



void QRobotConfigDialog::on_buttonBox_accepted()
{
    mRobotConfig.EncoderPosition = ui->pushButton_check_motor->isChecked()?Motor:Wheel;
    mRobotConfig.MotorEnableLevel = ui->pushButton_check_en_low->isChecked()?Low:High;

    mRobotConfig.EncoderCprLeft = ui->lineEdit_robot_cpr_left->text().toInt();
    mRobotConfig.EncoderCprRight = ui->lineEdit_robot_cpr_right->text().toInt();
    mRobotConfig.Height = ui->lineEdit_robot_height->text().toInt();
    mRobotConfig.Lenght = ui->lineEdit_robot_lenght->text().toInt();
    mRobotConfig.MaxAmpereMotorLeft = ui->lineEdit_robot_max_i_left->text().toInt();
    mRobotConfig.MaxAmpereMotorRight = ui->lineEdit_robot_max_i_right->text().toInt();
    mRobotConfig.MaxRpmMotorLeft = ui->lineEdit_robot_max_rpm_left->text().toInt();
    mRobotConfig.MaxRpmMotorRight = ui->lineEdit_robot_max_rpm_right->text().toInt();
    mRobotConfig.MaxTorqueMotorLeft = ui->lineEdit_robot_max_torque_left->text().toInt();
    mRobotConfig.MaxTorqueMotorRight = ui->lineEdit_robot_max_torque_right->text().toInt();
    mRobotConfig.WheelRadiusLeft = ui->lineEdit_robot_rad_left->text().toInt();
    mRobotConfig.WheelRadiusRight = ui->lineEdit_robot_rad_right->text().toInt();
    mRobotConfig.RatioShaftLeft = ui->lineEdit_robot_ratio_shaft_left->text().toInt();
    mRobotConfig.RatioShaftRight = ui->lineEdit_robot_ratio_shaft_right->text().toInt();
    mRobotConfig.RatioMotorLeft = ui->lineEdit_robot_ratio_motor_left->text().toInt();
    mRobotConfig.RatioMotorRight =  ui->lineEdit_robot_ratio_motor_right->text().toInt();
    mRobotConfig.Weight = ui->lineEdit_robot_weight->text().toInt();
    mRobotConfig.WheelBase = ui->lineEdit_robot_wheelbase->text().toInt();
    mRobotConfig.Width = ui->lineEdit_robot_width->text().toInt();

    mRobotConfig.MaxChargedBatteryLevel = (quint16)(ui->lineEdit_robot_max_batt_charge->text().toDouble()*1000.0);
    mRobotConfig.MinChargedBatteryLevel = (quint16)(ui->lineEdit_robot_min_batt_charge->text().toDouble()*1000.0);

    emit accepted();
}

void QRobotConfigDialog::getRobotConfiguration( RobotConfiguration& robotConfig )
{
    memcpy( &robotConfig, &mRobotConfig, sizeof(RobotConfiguration) );
}

}
