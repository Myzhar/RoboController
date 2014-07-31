#include <QtCore/QCoreApplication>
#include <QSettings>
#include <QDebug>
#include <QCommandLineParser>
#include <QCommandLineOption>

#include <qwebcamserver.h>

#include <QTime>

using namespace roboctrl;

class MyApplication : public QCoreApplication
{
public:
    MyApplication(int& argc, char** argv) :
        QCoreApplication(argc, argv)
    {
        setApplicationName( "Robot Webcam Server" );
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
    MyApplication app(argc, argv);

    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, ".");

    QCommandLineParser parser;
    parser.setApplicationDescription("Robot Webcam Server");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption cameraOpt( QStringList() << "c" << "camera", QObject::tr("Camera selection"), "-1" );

    parser.addOption( cameraOpt );

    parser.process( app );

    QString cameraStr = parser.value( cameraOpt );
    int camera = -1;

    if( !cameraStr.isEmpty() )
        camera = cameraStr.toInt();

    try
    {
        QWebcamServer* webcamServer = new QWebcamServer(camera, 55554, 55555, 8192, 5, NULL );

        QTime time;
        time.start();

        while( !webcamServer->isRunning() )
        {
            if( time.elapsed() > 5000 )
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

    return app.exec();
}
