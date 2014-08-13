#include <QCoreApplication>
#include <qrgbdserver.h>
#include <QSettings>
#include <QDebug>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QTime>

using namespace roboctrl;

class MyApplication : public QCoreApplication
{
public:
    MyApplication(int& argc, char** argv) :
        QCoreApplication(argc, argv)
    {
        setApplicationName( "Robot RGB-D Server" );
        setApplicationVersion( "0.1");
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
    MyApplication a(argc, argv);

    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, ".");

    QCommandLineParser parser;
    parser.setApplicationDescription("Robot RGB-D Server");
    parser.addHelpOption();
    parser.addVersionOption();

    parser.process( a );

    try
    {
        QRgbdServer* server = new QRgbdServer(  55554, 55555, 8192, NULL );

        QTime time;
        time.start();

        while( !server->isRunning() )
        {
            if( time.elapsed() > 5000 )
            {
                qDebug() << QObject::tr("RGB-D Server not ready after five second. Verify System status.");
                exit( EXIT_FAILURE );
            }
        }
    }
    catch(roboctrl::RcException &e)
    {
        qCritical() << QObject::tr("Server Exception: %1").arg(e.getExcMessage());
        exit( EXIT_FAILURE );
    }

    return a.exec();
}
