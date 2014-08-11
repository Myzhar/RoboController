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

    void setMap(Q2dMap& map );

private:
    void drawGrid();
    void drawAreas();

    QVector< QGraphicsEllipseItem* > mMapPoints;

    qreal mWarningDist;
    qreal mDangerDist;
};

}

#endif // Q2DMAPSCENE_H
