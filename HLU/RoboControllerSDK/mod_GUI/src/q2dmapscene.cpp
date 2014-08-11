#include <q2dmapscene.h>
#include <QGraphicsView>
#include <QDebug>

namespace roboctrl
{

Q2dMapScene::Q2dMapScene()
{
    setBackgroundBrush( QBrush(QColor(200,200,200)));

    mWarningDist = 1500;
    mDangerDist = 1000;

    drawGrid();
    drawAreas();
}

void Q2dMapScene::drawGrid()
{
    QPen penGrid(QColor(150,150,150));
    penGrid.setWidth(3);
    for( int i=0; i<=MAX_EXT; i+=100 )
    {
        addLine( -MAX_EXT/2, i, MAX_EXT/2, i, QPen(QColor(150,150,150 )) );
        addLine( i-MAX_EXT/2, 0, i-MAX_EXT/2, MAX_EXT, QPen(QColor(150,150,150 )) );
    }

    QPen penXaxis(QColor(150,50,50));
    penXaxis.setWidth(5);
    QPen penYaxis(QColor(50,50,150));
    penYaxis.setWidth(5);

    addLine( -MAX_EXT/2,0,MAX_EXT/2,0, penXaxis );
    addLine( 0,0,0,MAX_EXT, penYaxis );
}

void Q2dMapScene::drawAreas()
{
    QPen dangerPen( QColor(255,50,50,30) );
    dangerPen.setWidth(1);

    QBrush dangerBr( QColor(255,50,50,30) );

    QGraphicsEllipseItem* danger = new QGraphicsEllipseItem();
    danger->setPen( dangerPen);
    danger->setBrush( dangerBr );
    danger->setRect( -mDangerDist, -mDangerDist, 2*mDangerDist, 2*mDangerDist );
    addItem( danger );

    QPen warningPen( QColor(255,255,50,30) );
    warningPen.setWidth(1);

    QBrush warningBr( QColor(255,255,50,30) );

    QGraphicsEllipseItem* warning = new QGraphicsEllipseItem();
    warning->setPen( warningPen);
    warning->setBrush( warningBr );
    warning->setRect( -mWarningDist, -mWarningDist, 2*mWarningDist, 2*mWarningDist );
    addItem( warning );
}

void Q2dMapScene::setMap(Q2dMap& map )
{
    int countZero=0;

    if(mMapPoints.size()!=map.size())
    {
        clear();
        drawGrid();
        drawAreas();

        mMapPoints.clear();

        for( int i=0; i<map.size(); i++ )
        {
            QGraphicsEllipseItem* newPt = addEllipse( map[i].x, map[i].z, 11, 11,
                                                      QPen(QColor(50,155,50)),
                                                      QBrush(QColor(155,155,50)) ) ;
            mMapPoints.push_back( newPt );
        }
    }
    else for( int i=0; i<mMapPoints.size(); i++ )
    {
        mMapPoints[i]->setPos( map[i].x, map[i].z );

        if(map[i].z==0)
            countZero++;

        if(countZero>=map.size() )
            mMapPoints.clear();
    }
}

}
