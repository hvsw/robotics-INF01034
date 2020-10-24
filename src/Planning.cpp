#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX
#include <GL/glut.h>

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    robotPosition = newRobotPosition;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;

    gridLimits = newGridLimits;
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

    resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);

    initializePotentials();

    for(int i=0; i<100; i++){
        iteratePotentials();
    }

    updateGradient();
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
            c->planType = REGULAR;
        }
    }
}

void Planning::updateCellsTypes()
{
    Cell* c;

    // If you want to access all observed cells (since the start), use this range
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)

    // TODO: classify cells

    // the occupancy type of a cell can be defined as:
    // c->occType = UNEXPLORED
    // c->occType = OCCUPIED
    // c->occType = FREE

    // the planning type of a cell can be defined as:
    // c->planType = REGULAR
    // c->planType = FRONTIER
    // c->planType = DANGER
    // c->planType = NEAR_WALLS
    // c->planType = FRONTIER_NEAR_WALL

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
                87654321X12345678
                for(int i = x-8; i <= x+8; i++)
                {
                    for(int j = y-8; j <= y+8; j++)
                    {
                        planningCell = grid->getCell(i,j);

                        bool isCurrentCell = (x == i && y == j);
                        if (isCurrentCell || planningCell->occType != FREE) {
                            continue;
                        }

                        if (abs(8-i-x) < 3 && abs(8-j-y) < 3) {
                            planningCell->planType = DANGER;
                        } else {
                            planningCell->planType = NEAR_WALLS;
                        }

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


enum Potencial {
    Harmonico, Preferencias, ObjetivosDinamicos
};

void Planning::initializePotentials() {
    int METODO_HARMONICO = 0;
    int METODO_preferencias = 1;
    int OBJETIVOS_DINAMICOS = 2;
    // the potential of a cell is stored in:
    // c->pot[i]
    // the preference of a cell is stored in:
    // c->pref

    // TODO: initialize the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)

    Cell *c;
    for(int i = curPose.x - radius; i <= curPose.x + radius; i++) {
        for(int j = curPose.y - radius; j <= curPose.y + radius; j++) {
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED || c->planType == DANGER)
                c->pot = 1.0;
            else if(c->type == UNEXPLORED)
                c->pot = 0.0;



            c->pref = curPref;
        }
    }


}

void Planning::iteratePotentials()
{
    Cell* c;
    Cell *left,*right,*up,*down;

    // the update of a FREE cell in position (i,j) will use the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: iterate the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)





}

void Planning::updateGradient()
{
    Cell* c;

    // the components of the descent gradient of a cell are stored in:
    // c->dirX[i] and c->dirY[i], for pot[i]

    Cell *left,*right,*up,*down;

    // the gradient of a FREE cell in position (i,j) is computed using the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: compute the gradient of the FREE cells in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)







}



