#ifndef LOGHANDLER_H
#define LOGHANDLER_H

#include <QDebug>
#include <QTime>

#include <fstream>

using namespace std;
extern ofstream logfile;

#define PREFIX  "[ Time: " << QTime::currentTime().toString("hh:mm:ss.zzz")  \
    << "] - Function: " << Q_FUNC_INFO << " - "

void SimpleLoggingHandler(QtMsgType type, const char *msg);


#endif // LOGHANDLER_H
