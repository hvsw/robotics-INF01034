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

void Planning::classificarPlanType(Cell *c, int cellX, int cellY) {
    int size;
    Cell *planningCell;
    if (c->occType == OCCUPIED) {
        size = 3;
        for (int xInRange = cellX-size; xInRange <= cellX+size; xInRange++) {
            for (int yInRange = cellY-size; yInRange <= cellY+size; yInRange++) {
                bool isCurrentCell = (xInRange == cellX && yInRange == cellY);
                if (isCurrentCell) {
                    continue;
                }
                
                planningCell = grid->getCell(xInRange, yInRange);
                
                if (planningCell->occType == FREE) {
                    planningCell->planType = DANGER;
                }
            }
        }
        
        size = 8;
        for (int xInRange = cellX-size; xInRange <= cellX+size; xInRange++) {
            for (int yInRange = cellY-size; yInRange <= cellY+size; yInRange++) {
                bool isCurrentCell = (xInRange == cellX && yInRange == cellY);
                if (isCurrentCell) {
                    continue;
                }
                
                planningCell = grid->getCell(xInRange, yInRange);
                
                if (planningCell->occType == FREE && planningCell->planType != DANGER) {
                    planningCell->planType = NEAR_WALLS;
                }
            }
        }
    }
    
    size = 1;
    if (c->occType == FREE) {
        for (int xInRange = cellX-size; xInRange <= cellX+size; xInRange++) {
            for (int yInRange = cellY-size; yInRange <= cellY+size; yInRange++) {
                bool isCurrentCell = (xInRange == cellX && yInRange == cellY);
                if (isCurrentCell) {
                    continue;
                }
                
                planningCell = grid->getCell(xInRange, yInRange);
                
                if (planningCell->occType == UNEXPLORED) {
                    planningCell->planType = FRONTIER;
                    
                    for (int i = cellX-8; i <= xInRange+8; i++) {
                        for (int j = cellY-8; j <= cellY+8; j++) {
                            bool isCurrentCell = (i == xInRange && j == yInRange);
                            if (isCurrentCell) {
                                continue;
                            }
                            
                            Cell *otherPlanningCell = grid->getCell(i, j);
                            if (otherPlanningCell->occType == OCCUPIED) {
                                planningCell->planType = FRONTIER_NEAR_WALL;
                            }
                        }
                    }
                }
            }
        }
    }
}

void Planning::classificarOccType(Cell *c) {
    float unexploredToOccThreshold = 9;
    float unexploredToFreeThreshold = 5;
    float freeToOccThreshold = 9;
    float occToFreeThreshold = 12;
    
    float himmReading = c->himm;
    
    //classify cells
    if(himmReading >= unexploredToOccThreshold && c->occType == UNEXPLORED) {
        c->occType = OCCUPIED;
    } else if (himmReading <= unexploredToFreeThreshold && c->occType == UNEXPLORED) {
        c->occType = FREE;
    } else if (himmReading >= freeToOccThreshold && c->occType == FREE) {
        c->occType = OCCUPIED;
    } else if (himmReading <= occToFreeThreshold && c->occType == OCCUPIED) {
        c->occType = FREE;
    }
}

void Planning::updateCellsTypes()
{
    
    // If you want to access all observed cells (since the start), use this range
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)
    
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
    
    Cell *cellInUpdateRange;
    for (int xInUpdateRange = gridLimits.minX; xInUpdateRange <= gridLimits.maxX; xInUpdateRange++) {
        for (int yInUpdateRange = gridLimits.minY; yInUpdateRange <= gridLimits.maxY; yInUpdateRange++) {
            cellInUpdateRange = grid->getCell(xInUpdateRange, yInUpdateRange);
            classificarOccType(cellInUpdateRange);
        }
    }
    
    for (int xInUpdateRange = gridLimits.minX; xInUpdateRange <= gridLimits.maxX; xInUpdateRange++) {
        for (int yInUpdateRange = gridLimits.minY; yInUpdateRange <= gridLimits.maxY; yInUpdateRange++) {
            cellInUpdateRange = grid->getCell(xInUpdateRange, yInUpdateRange);
            classificarPlanType(cellInUpdateRange, xInUpdateRange, yInUpdateRange);
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



