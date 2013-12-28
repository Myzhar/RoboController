#include "cconfiginidialog.h"
#include "ui_cconfiginidialog.h"
#include <QFile>
#include <QDebug>

CConfigIniDialog::CConfigIniDialog(QString iniFile, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CConfigIniDialog)
{
    ui->setupUi(this);

    QFile file(iniFile);

    if(!file.exists())
    {
        qCritical() << tr("%1 file does not exists!").arg(iniFile);
        return;
    }

    file.open( QIODevice::ReadOnly );

    QString content(file.readAll());

    ui->textEdit_iniContent->setText(content);
}

CConfigIniDialog::~CConfigIniDialog()
{
    delete ui;
}
