#ifndef QSCREENTOOLS_H
#define QSCREENTOOLS_H

#include <QObject>

#define REF_DPI 196.0f

class QScreenTools: public QObject
{
    Q_OBJECT

private:
    qreal mDpi;

public:
    QScreenTools();

    /*!
     * Convert a measure in mm to pixel
     * according to Screen dpi
     *
     * @param mm measure to be converted
     *
     * @return the corresponding pixels
     */
    int cvtMm2Px( qreal mm );

    /*!
     * Convert a measure in inches to pixel
     * according to Screen dpi
     *
     * @param inches measure to be converted
     *
     * @return the corresponding pixels
     */
    int cvtInch2Px( qreal in );
};

#endif // QSCREENTOOLS_H
