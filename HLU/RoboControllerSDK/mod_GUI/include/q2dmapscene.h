#ifndef Q2DMAPSCENE_H
#define Q2DMAPSCENE_H

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>

#include <opencvtools.h>

#define MAX_EXT 10000

namespace roboctrl
{

class Q2dMapScene : public QGraphicsScene
{
public:
    Q2dMapScene();

public slots:
    void setMap(Q2dMap& map );

protected:
    void drawGrid(); ///< Draws grid and axis
    void drawAreas(); ///< Draws warning and danger areas

private:

    QVector< QGraphicsEllipseItem* > mMapPoints; ///< Array of the points of the map

    qreal mWarningDist; ///< Radius of the Warning distance in mm
    qreal mDangerDist; ///< Radius of the Danger Distance in mm

    qreal mPtSize; ///< Size of the points in pixel
};

}

#endif // Q2DMAPSCENE_H
