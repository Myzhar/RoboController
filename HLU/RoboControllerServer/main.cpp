#include <QtCore/QCoreApplication>
#include <QSettings>
#include <QDebug>

#include <qrobotserver.h>
#include <qwebcamserver.h>

#include <QTime>

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

        QTime time;
        time.start();
        while( !server->isRunning() )
        {
            if( time.elapsed() > 1000 )
            {
                qDebug() << QObject::tr("Control Server not ready after one second. Verify System status.");
                exit( EXIT_FAILURE );
            }
        }

        QWebcamServer* webcamServer = new QWebcamServer(0, 55554, 55555, 512, 5, NULL );
        time.restart();

        while( !webcamServer->isRunning() )
        {
            if( time.elapsed() > 1000 )
            {
                qDebug() << QObject::tr("Webcam Server not ready after one second. Verify System status.");
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
