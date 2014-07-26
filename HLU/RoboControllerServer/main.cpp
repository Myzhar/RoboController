#include <QtCore/QCoreApplication>
#include <QSettings>
#include <QDebug>
#include <QCommandLineParser>
#include <QCommandLineOption>

#include <qrobotserver.h>

#include <QTime>

using namespace roboctrl;

class MyApplication : public QCoreApplication
{
public:
    MyApplication(int& argc, char** argv) :
        QCoreApplication(argc, argv)
    {
        setApplicationName( "RoboController Server" );
        setApplicationVersion( "0.2");
    }

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
    MyApplication app(argc, argv);

    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, ".");

    QCommandLineParser parser;
    parser.setApplicationDescription("RoboController Server helper");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption testOpt( "t", QObject::tr("Test Mode. The Server simulates a connection to a Virtual RoboController.") );

    parser.addOption( testOpt );

    parser.process( app );

    bool test = parser.isSet( testOpt );

    try
    {
        QRobotServer* server = new QRobotServer(14560, 14565, 14550, 14555, 14500, test );

        QTime time;
        time.start();
        while( !server->isRunning() )
        {
            if( time.elapsed() > 5000 )
            {
                qDebug() << QObject::tr("Control Server not ready after one second. Verify System status.");
                exit( EXIT_FAILURE );
            }
        }
    }
    catch(roboctrl::RcException &e)
    {
        qCritical() << QObject::tr("Server Exception: %1").arg(e.getExcMessage());
        exit( EXIT_FAILURE );
    }

    return app.exec();
}
