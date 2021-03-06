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
    
    ///  -1,-1   -1,0  -1,1
    ///   0,-1    0,0   0,1
    ///   1,-1    1,0   1,1
    
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
    float unexploredToFreeThreshold = 2;
    float freeToOccThreshold = 5;
    float occToFreeThreshold = 2;
    
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

void Planning::initializeCellPotential(Cell *cell) {
//    Independente do método, células OCCUPIED e DANGER deverão sempre receber um potencial repulsivo, igual a 1.0.
    if (cell->occType == OCCUPIED || cell->planType == DANGER) {
        cell->pot[0] = cell->pot[1] = cell->pot[2] = 1;
    }
    
    //    Nos métodos A e B, células de fronteira (FRONTIER e FRONTIER_NEAR_WALL)
    //    deverão receber um potencial atrator, igual a 0.0. Além disso, por padrão nestes dois
    //    métodos, todas as células iniciam com potencial baixo, pot[0]=pot[1]=0 (isso já
    //    está definido na classe Grid e não deve ser alterado).
    //    if (cell->planType == FRONTIER || cell->planType == FRONTIER_NEAR_WALL) {
    //        cell->pot[0] = cell->pot[1] = 0;
    //    }
    
//    Em contrapartida, no método C somente as células de fronteira próximas a paredes (FRONTIER_NEAR_WALL) recebem potencial atrator fixo (0.0)
    if (cell->planType == FRONTIER_NEAR_WALL) {
        cell->pot[2] = 0;
    }
    
//    Em contrapartida, no método C somente as células de fronteira próximas a paredes
//    (FRONTIER_NEAR_WALL) recebem potencial atrator fixo (0.0) . As demais células
//    de fronteira (FRONTIER), são fixadas com potencial repulsivo (1.0). Note que diferentemente dos métodos A e B, todas as células iniciam com potencial alto (pot[2]=1).
//    Isto é feito para garantir que somente os “cantos” das fronteiras sejam atratores.
//    if (cell->planType == FRONTIER) {
//        cell->pot[2] = 1;
//    }
    
//    Os potenciais das demais células (que não se enquadram nessas classificações) não devem ser inicializados, pois eles serão computados iterativamente a partir dos valores anteriores.
}

void Planning::initializeCellPreference(Cell *cell) {
//    Por fim, é nesta função que se deve inicializar as preferências das células (c->pref) que serão utilizadas no método B. Você deve escolher um valor de preferência fixo positivo para as células NEAR_WALLS e um valor fixo negativo para as demais células livres. Escolha valores simétricos entre 1 e -1 (isto é, ± 0.1; ± 0.2; ± 0.3, ...). Valores adequados devem ser encontrados empiricamente.
    float baseValue = 0.8;
    if (cell->planType == NEAR_WALLS) {
        // Você deve escolher um valor de preferência fixo positivo para as células NEAR_WALLS
        cell->pref = baseValue;
    } else {
        // um valor fixo negativo para as demais células livres
        cell->pref = -baseValue;
    }
    
}

void Planning::initializePotentials()
{
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)
    
    Cell *cellInUpdateRange;
    for (int xInUpdateRange = gridLimits.minX; xInUpdateRange <= gridLimits.maxX; xInUpdateRange++) {
        for (int yInUpdateRange = gridLimits.minY; yInUpdateRange <= gridLimits.maxY; yInUpdateRange++) {
            cellInUpdateRange = grid->getCell(xInUpdateRange, yInUpdateRange);
            initializeCellPotential(cellInUpdateRange);
            initializeCellPreference(cellInUpdateRange);
        }
    }
}

float Planning::potencialHarmonico(int cellX, int cellY, int potentialType) {
    float potencialHarmonico;
    
    Cell *left = grid->getCell(cellX-1, cellY);
    Cell *right = grid->getCell(cellX+1, cellY);
    Cell *up = grid->getCell(cellX, cellY-1);
    Cell *down = grid->getCell(cellX, cellY+1);
    
    potencialHarmonico = (left->pot[potentialType] + right->pot[potentialType] + up->pot[potentialType] + down->pot[potentialType])/4;
    
    return potencialHarmonico;
}

float Planning::potencialPreferencias(int cellX, int cellY) {
    Cell *cell = grid->getCell(cellX, cellY);
    Cell *left = grid->getCell(cellX-1, cellY);
    Cell *right = grid->getCell(cellX+1, cellY);
    Cell *up = grid->getCell(cellX, cellY-1);
    Cell *down = grid->getCell(cellX, cellY+1);
    
    int potentialType = 2; // preferencias
    float h = potencialHarmonico(cellX, cellY, potentialType);
    float d = abs((up->pot[potentialType] - down->pot[potentialType])/2) + abs((left->pot[potentialType] - right->pot[potentialType])/2);
    
    float potencialPreferencias = h - (cell->pref*d/4);
    
    return potencialPreferencias;
}

void Planning::iteratePotentials()
{
    // TODO: iterate the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)
    Cell *cellInUpdateRange;
    for (int xInUpdateRange = gridLimits.minX; xInUpdateRange <= gridLimits.maxX; xInUpdateRange++) {
        for (int yInUpdateRange = gridLimits.minY; yInUpdateRange <= gridLimits.maxY; yInUpdateRange++) {
            cellInUpdateRange = grid->getCell(xInUpdateRange, yInUpdateRange);
            //  basta atualizar o potencial das células FREE.
            if (cellInUpdateRange->occType == FREE) {
                //  Para isso varra todas as células dentro da área conhecida do mapa (limitada por gridLimits). Células que forem FREE devem ser atualizadas em função das células vizinhas usando diferenças finitas
                
                // Os métodos A e C usam a definição original do potencial harmônico descrita no Algoritmo 1 da Aula 15
                cellInUpdateRange->pot[0] = potencialHarmonico(xInUpdateRange, yInUpdateRange, 0);
                cellInUpdateRange->pot[2] = potencialHarmonico(xInUpdateRange, yInUpdateRange, 2);
                
                // Já o método B usa a equação atualizada definida no Algoritmo 2 da Aula 15, que permite o uso de preferências no cálculo do potencial.
                cellInUpdateRange->pot[1] = potencialPreferencias(xInUpdateRange, yInUpdateRange);
            }
        }
    }
}

float Planning::gradientX(int potentialType, int cellX, int cellY) {
    Cell *up = grid->getCell(cellX, cellY-1);
    Cell *down = grid->getCell(cellX, cellY+1);
    Cell *left = grid->getCell(cellX-1, cellY);
    Cell *right = grid->getCell(cellX+1, cellY);
    float dx = (left->pot[potentialType] - right->pot[potentialType])/2;
    float dy = (up->pot[potentialType] - down->pot[potentialType])/2;
    float norma = sqrt(pow(dx, 2)+pow(dy, 2));
    
    float gradientX = dx;
    
    if (norma != 0) {
        gradientX /= norma;
    }
    
    return gradientX;
}

float Planning::gradientY(int potentialType, int cellX, int cellY) {
    Cell *up = grid->getCell(cellX, cellY-1);
    Cell *down = grid->getCell(cellX, cellY+1);
    Cell *left = grid->getCell(cellX-1, cellY);
    Cell *right = grid->getCell(cellX+1, cellY);
    float dx = (left->pot[potentialType] - right->pot[potentialType])/2;
    float dy = (up->pot[potentialType] - down->pot[potentialType])/2;
    
    float norma = sqrt(pow(dx, 2)+pow(dy, 2));
    
    float gradientY = dy;
    
    // Lembre de normalizar o vetor gradiente pois para a navegação só nós interessa saber a direção do gradiente, não sua intensidade. Ou seja, divida as componentes horizontais e verticais pela norma do gradiente |∇p| (desde que |∇p| 6= 0)
    
    if (norma != 0) {
        gradientY /= norma;
    }
    
    return gradientY;
}

void Planning::updateGradient()
{
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)
    Cell *cellInUpdateRange;
    for (int xInUpdateRange = gridLimits.minX; xInUpdateRange <= gridLimits.maxX; xInUpdateRange++) {
        for (int yInUpdateRange = gridLimits.minY; yInUpdateRange <= gridLimits.maxY; yInUpdateRange++) {
            cellInUpdateRange = grid->getCell(xInUpdateRange, yInUpdateRange);
            // Nesta etapa é preciso computar o vetor gradiente descendente (−∇p) de cada um dos três potenciais das células FREE, através de uma varredura igual a feita no exercício anterior.
            if (cellInUpdateRange->occType == FREE) {
                // Cada célula (Cell* c) do grid tem os vetores gradiente descritos pelas componentes horizontais e verticais: c->dirX[i] e c->dirY[i], onde i=0,1,2 indica a qual potencial se refere
                    
                cellInUpdateRange->dirX[0] = gradientX(0, xInUpdateRange, yInUpdateRange);
                cellInUpdateRange->dirY[0] = gradientY(0, xInUpdateRange, yInUpdateRange);
                
                cellInUpdateRange->dirX[1] = gradientX(1, xInUpdateRange, yInUpdateRange);
                cellInUpdateRange->dirY[1] = gradientY(1, xInUpdateRange, yInUpdateRange);
                
                cellInUpdateRange->dirX[2] = gradientX(2, xInUpdateRange, yInUpdateRange);
                cellInUpdateRange->dirY[2] = gradientY(2, xInUpdateRange, yInUpdateRange);
            }
        }
    }
}
