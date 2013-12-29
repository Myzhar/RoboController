#include <QtCore/QCoreApplication>
#include <QSettings>
#include <QDebug>

#include "qrobottcpserver.h"

using namespace roboctrl;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);    

    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, ".");

    try
    {
        QRobotTcpServer* server = new QRobotTcpServer();
    }
    catch(roboctrl::RcException &e)
    {
        qCritical() << QObject::tr("Server Exception: %1").arg(e.getExcMessage());
        exit( EXIT_FAILURE );
    }
    
    return a.exec();
}
