#ifndef CROBOTCONFIGDIALOG_H
#define CROBOTCONFIGDIALOG_H

#include <QDialog>
#include <QVector>
#include <QScrollBar>

#include <qscreentools.h>



typedef unsigned short uint16_t;
typedef QVector<int> IntVector;




namespace Ui {
class CRobotConfigDialog;
}

namespace roboctrl
{

class QRobotConfigDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit QRobotConfigDialog(QWidget *parent = 0);
    ~QRobotConfigDialog();

signals:
    
private slots:
private:
    Ui::CRobotConfigDialog *ui;
};

}

#endif // CROBOTCONFIGDIALOG_H
