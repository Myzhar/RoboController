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

    try
    {
        QRobotServer* server = new QRobotServer();
    }
    catch(roboctrl::RcException &e)
    {
        qCritical() << QObject::tr("Server Exception: %1").arg(e.getExcMessage());
        exit( EXIT_FAILURE );
    }

    return a.exec();
}
