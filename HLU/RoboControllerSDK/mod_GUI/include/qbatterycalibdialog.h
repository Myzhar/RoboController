#ifndef QBATTERYCALIBDIALOG_H
#define QBATTERYCALIBDIALOG_H

#include <QDialog>

#include <robocontrollersdk.h>

namespace Ui {
class QBatteryCalibDialog;
}

namespace roboctrl
{

class QBatteryCalibDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QBatteryCalibDialog( RoboControllerSDK* robSdk, QWidget *parent = 0 );
    ~QBatteryCalibDialog();

signals:

private slots:
    void onNewBatteryValue(double val);
    void on_pushButton_set_bat_calib_value_clicked();
    void on_pushButton_ok_clicked();

private:
    Ui::QBatteryCalibDialog *ui;
};

}

#endif // QBATTERYCALIBDIALOG_H
