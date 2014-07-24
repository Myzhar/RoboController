#include <loghandler.h>

#include <fstream>
ofstream logfile;

void SimpleLoggingHandler(QtMsgType type, const char *msg)
{
    switch (type)
    {
    case QtDebugMsg:
        logfile /*<< QTime::currentTime().toString().toAscii().data()*/ << " Debug: " << msg << "\n";
        break;
    case QtCriticalMsg:
        logfile /*<< QTime::currentTime().toString().toAscii().data()*/ << " Critical: " << msg << "\n";
        break;
    case QtWarningMsg:
        logfile /*<< QTime::currentTime().toString().toAscii().data()*/ << " Warning: " << msg << "\n";
        break;
    case QtFatalMsg:
        logfile /*<< QTime::currentTime().toString().toAscii().data()*/ <<  " Fatal: " << msg << "\n";
        abort();
    }

    logfile.flush();
}
// <--- Logging
