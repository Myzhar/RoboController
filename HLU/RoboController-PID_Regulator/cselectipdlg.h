#ifndef CSELECTIPDLG_H
#define CSELECTIPDLG_H

#include <QDialog>
#include <QSettings>

namespace Ui {
class CSelectIpDlg;
}

class CSelectIpDlg : public QDialog
{
    Q_OBJECT
    
public:
    explicit CSelectIpDlg(QWidget *parent = 0);
    ~CSelectIpDlg();
    
private slots:
    void on_buttonBox_accepted();

private:
    Ui::CSelectIpDlg *ui;
    QSettings       mSettings;

    unsigned int    mServerPort;
    QString         mServerAddr;
};

#endif // CSELECTIPDLG_H
