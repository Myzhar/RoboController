#include "cselectipdlg.h"
#include "ui_cselectipdlg.h"
#include "macros.h"

CSelectIpDlg::CSelectIpDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CSelectIpDlg),
    mSettings(INI_FILE,QSettings::IniFormat),
    mServerPort(4500),
    mServerAddr(tr("localhost"))
{
    ui->setupUi(this);

    mSettings.beginGroup( QString("SERVER_INFO"));
    {
        mServerAddr = mSettings.value( "Server_address", tr("") ).toString();
        if( mServerAddr.compare("")==0 )
        {
            mServerAddr = QString("localhost");
            mSettings.setValue( "Server_address", mServerAddr );
            mSettings.sync();
        }

        mServerPort = mSettings.value( "Server_port", "0" ).toUInt();
        if( mServerPort==0 )
        {
            mServerPort = 4500;
            mSettings.setValue( "Server_port", QString("%1").arg(mServerPort) );
            mSettings.sync();
        }
    }
    mSettings.endGroup();

    ui->lineEdit_serverIP->setText( mServerAddr );
    ui->lineEdit_serverPort->setText( tr("%1").arg(mServerPort) );
}

CSelectIpDlg::~CSelectIpDlg()
{
    delete ui;
}

void CSelectIpDlg::on_buttonBox_accepted()
{
    this->setResult( QDialog::Accepted );

    mServerAddr = ui->lineEdit_serverIP->text();
    mServerPort = ui->lineEdit_serverPort->text().toInt();

    // >>>>> Saving on INI file
    mSettings.beginGroup( QString("SERVER_INFO"));
    {
        mSettings.setValue( "Server_address", mServerAddr );
        mSettings.sync();
        mSettings.setValue( "Server_port", QString("%1").arg(mServerPort) );
        mSettings.sync();
    }
    mSettings.endGroup();
    // <<<<< Saving on INI file
}
