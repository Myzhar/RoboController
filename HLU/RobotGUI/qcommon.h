#ifndef QCOMMON_H
#define QCOMMON_H

#include <QObject>
#include <qscreentools.h>

class QCommon : public QObject
{
    Q_OBJECT

public:
    roboctrl::QScreenTools mScreen;

private:
    qreal mDefFontSize;

public:
    explicit QCommon(QObject *parent = 0);

signals:

public slots:

};

extern QCommon* COMMON;

#endif // QCOMMON_H
