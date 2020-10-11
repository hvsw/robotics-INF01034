#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;
}

Planning::~Planning()
{}

void Planning::setGrid(Grid *g)
{
    grid = g;
}

void Planning::setMaxUpdateRange(int r)
{
    maxUpdateRange = 1.2*r*grid->getMapScale();
}

void Planning::setNewRobotPose(Pose p)
{
    newRobotPosition.x = (int)(p.x*grid->getMapScale());
    newRobotPosition.y = (int)(p.y*grid->getMapScale());

    newGridLimits.minX = std::min(newGridLimits.minX,newRobotPosition.x-maxUpdateRange);
    newGridLimits.maxX = std::max(newGridLimits.maxX,newRobotPosition.x+maxUpdateRange);
    newGridLimits.minY = std::min(newGridLimits.minY,newRobotPosition.y-maxUpdateRange);
    newGridLimits.maxY = std::max(newGridLimits.maxY,newRobotPosition.y+maxUpdateRange);
}

void Planning::run()
{
    pthread_mutex_lock(grid->mutex);

    // resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);
}

/////////////////////////////////////////////
///                                       ///
/// Métodos para classificacao de celulas ///
///                                       ///
/////////////////////////////////////////////

void Planning::resetCellsTypes()
{
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){

            Cell* c = grid->getCell(i,j);

            c->occType = UNEXPLORED;
            c->planType = REGULAR;
        }
    }
}

void Planning::updateCellsTypes()
{

    // If you want to access the current cells surrounding the robot, use this range
    //
    //  (robotPosition.x-maxUpdateRange, robotPosition.y+maxUpdateRange)  -------  (robotPosition.x+maxUpdateRange, robotPosition.y+maxUpdateRange)
    //                                 |                                    \                                     |
    //                                 |                                     \                                    |
    //                                 |                                      \                                   |
    //  (robotPosition.x-maxUpdateRange, robotPosition.y-maxUpdateRange)  -------  (robotPosition.y+maxUpdateRange, robotPosition.y-maxUpdateRange)


    // If you want to access all observed cells (since the start), use this range
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)

    float occupiedValue = 9;
    float freeValue = 5;
    float occToFree = 12;

    for(int x = robotPosition.x - maxUpdateRange; x <= robotPosition.x + maxUpdateRange; x++)
    {
        for(int y = robotPosition.y - maxUpdateRange; y <= robotPosition.y + maxUpdateRange; y++)
        {
            Cell* c;

            c = grid->getCell(x,y);

            float log = c->logodds;

            // the occupancy type of a cell can be defined as:
            // c->occType = UNEXPLORED
            // c->occType = OCCUPIED
            // c->occType = FREE
            if(log >= occupiedValue && c->occType == UNEXPLORED)
            {
                c->occType = OCCUPIED;

            }
            else if (log <= freeValue && c->occType == UNEXPLORED)
            {
                c->occType = FREE;

            }
            else if (log >= occupiedValue && c->occType == FREE)
            {
                c->occType = OCCUPIED;

            }
            else if (log <= occToFree && c->occType == OCCUPIED)
            {
                c->occType = FREE;

            }


            // the planning type of a cell can be defined as:
            // c->planType = REGULAR
            // c->planType = FRONTIER
            // c->planType = DANGER
            Cell *planningCell;

            if (c->occType == OCCUPIED) {

                for(int i = x-3; i <= x+3; i++)
                {
                    for(int j = y-3; j <= y+3; j++)
                    {
                        planningCell = grid->getCell(i,j);

                        if ((x == i && y == j) || planningCell->occType != FREE) {
                            continue;
                        }

                        planningCell->planType = DANGER;

                    }
                }

            } else if (c->occType == FREE) {

                for(int i = x-1; i <= x+1; i++)
                {
                    for(int j = y-1; j <= y+1; j++)
                    {
                        planningCell = grid->getCell(i,j);

                        if ((x == i && y == j) || planningCell->occType != UNEXPLORED) {
                            continue;
                        }

                        planningCell->planType = FRONTIER;

                    }
                }

            }
        }
    }
}


