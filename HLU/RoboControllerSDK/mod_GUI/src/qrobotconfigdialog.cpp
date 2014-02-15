#include <qrobotconfigdialog.h>
#include <ui_crobotconfigdialog.h>

#include <modbus_registers.h>
#include <QMessageBox>

#include <loghandler.h>


#define REG_COUNT 19

namespace roboctrl
{

QRobotConfigDialog::QRobotConfigDialog( QWidget *parent ) :
    QDialog(parent),
    ui(new Ui::CRobotConfigDialog)
{
    ui->setupUi(this);

    QScreenTools scrT;
    int pxSize = scrT.cvtMm2Px( 5 ); // Vertical scroll bar 5 mm width

    QString barStyle = tr("QScrollBar:horizontal {min-height: %1px;}" "QScrollBar:vertical {min-width: %1px;}").arg(pxSize);
    ui->scrollArea->setStyleSheet(barStyle);

    pxSize = scrT.cvtMm2Px( 3 ); // Radio button indicators 3 mm

    QString indicStyle = tr("QRadioButton::indicator{width:%1px; height:%1px;}").arg(pxSize);

    ui->radioButton_encoder_motor_2->setStyleSheet( indicStyle );
    ui->radioButton_encoder_shaft_2->setStyleSheet( indicStyle );
    ui->radioButton_motor_enable_high_2->setStyleSheet( indicStyle );
    ui->radioButton_motor_enable_low_2->setStyleSheet( indicStyle );
}

QRobotConfigDialog::~QRobotConfigDialog()
{
    delete ui;
}

}
