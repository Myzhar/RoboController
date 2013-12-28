#ifndef CCONFIGINIDIALOG_H
#define CCONFIGINIDIALOG_H

#include <QDialog>

namespace Ui {
class CConfigIniDialog;
}

class CConfigIniDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit CConfigIniDialog(QString iniFile,QWidget *parent = 0);
    ~CConfigIniDialog();
    
private:
    Ui::CConfigIniDialog *ui;
};

#endif // CCONFIGINIDIALOG_H
