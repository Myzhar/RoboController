#include "cmainwindow.h"
#include <QApplication>
#include <QSettings>

class MyApplication : public QApplication
{
public:
    MyApplication(int& argc, char** argv) :
        QApplication(argc, argv) {}

    virtual ~MyApplication() { }

    bool notify(QObject* receiver, QEvent* event)
    {
        bool done = true;
        try
        {
            done = QApplication::notify(receiver, event);
        }
        catch(roboctrl::RcException &e)
        {
            qCritical() << QObject::tr("RoboController Exception: %1").arg(e.getExcMessage());
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
    QApplication a(argc, argv);

    QSettings::setPath(QSettings::IniFormat, QSettings::SystemScope, ".");

    CMainWindow w;
    w.show();
    
    return a.exec();
}
