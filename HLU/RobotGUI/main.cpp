#include <QApplication>
#include "cmainwindow.h"

#include "qcommon.h"
#include "exception.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    try
    {
        COMMON = new QCommon();
    }
    catch( ... )
    {
        throw;
    }

    CMainWindow w;

#ifdef ANDROID
    w.showMaximized();
#else
    w.show();
#endif

    return a.exec();
}
