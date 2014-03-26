#include <QtCore/QCoreApplication>
#include <QSettings>
#include <QDebug>

#include <qrobotserver.h>

using namespace roboctrl;

class MyApplication : public QCoreApplication
{
public:
    MyApplication(int& argc, char** argv) :
        QCoreApplication(argc, argv) {}

    virtual ~MyApplication() { }

    virtual bool notify(QObject* receiver, QEvent* event)
    {
        bool done = true;
        try
        {
            done = QCoreApplication::notify(receiver, event);
        }
        catch(roboctrl::RcException &e)
        {
            qCritical() << QObject::tr("Robot Server Exception: %1").arg(e.getExcMessage());
            exit( EXIT_FAILURE );
        }
        catch(...)
        {
            qCritical() << tr("Unhandled exception!");
            exit( EXIT_FAILURE );
        }
        return done;
    }
};

int main(int argc, char *argv[])
{
    MyApplication a(argc, argv);

    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, ".");

    bool test = false;

    QString param = QObject::tr("%1").arg(argv[1]);

    if( argc==2 && param.compare( QObject::tr("test"), Qt::CaseInsensitive)==0 )
        test = true;

    try
    {
        QRobotServer* server = new QRobotServer(14560, 14550, 14555, 14500, test, NULL);
        if( server->isRunning() )
        {
            qDebug() << QObject::tr("Server has been correctly started.");
        }
    }
    catch(roboctrl::RcException &e)
    {
        qCritical() << QObject::tr("Server Exception: %1").arg(e.getExcMessage());
        exit( EXIT_FAILURE );
    }

    return a.exec();
}
