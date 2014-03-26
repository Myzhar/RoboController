#include "qscreentools.h"
#include <QApplication>
#include <QScreen>

#define MM2INCH 0.0393700787401575
#define INCH2MM 25.4

namespace roboctrl
{

QScreenTools::QScreenTools()
{
    mDpi = QApplication::primaryScreen()->physicalDotsPerInch();
    //mDpi = QApplication::primaryScreen()->logicalDotsPerInch();
}

int QScreenTools::cvtMm2Px( qreal mm )
{
    qreal inch = mm*MM2INCH;

    return cvtInch2Px( inch );
}

int QScreenTools::cvtInch2Px( qreal in )
{
    qreal px = in * mDpi;

    return (int)(px+0.5);
}

}
