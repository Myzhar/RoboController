#ifndef CROBOTCONFIGDIALOG_H
#define CROBOTCONFIGDIALOG_H

#include <QDialog>
#include <QVector>
#include <QScrollBar>

#include <qscreentools.h>

#include <robocontrollersdk.h>

namespace Ui {
class CRobotConfigDialog;
}

namespace roboctrl
{

class QRobotConfigDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit QRobotConfigDialog( RobotConfiguration& robotConfig, QWidget *parent = 0 );
    ~QRobotConfigDialog();

    void getRobotConfiguration( RobotConfiguration& robotConfig );

signals:
    
private slots:
    void on_buttonBox_accepted();

private:
    Ui::CRobotConfigDialog *ui;

    RobotConfiguration mRobotConfig;
};

}

#endif // CROBOTCONFIGDIALOG_H
