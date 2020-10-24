#ifndef __GRID_H__
#define __GRID_H__

#include <pthread.h>

enum CellOccType {OCCUPIED, UNEXPLORED, FREE};
enum CellPlanType {REGULAR, DANGER, NEAR_WALLS, FRONTIER, FRONTIER_NEAR_WALL};

#define UNDEF -10000000

#define NUM_POTENTIALS 3

class Cell
{
    public:
        int x,y;        
        int himm;
        double logodds,occupancy;
        double logoddsSonar,occupancySonar;

        double pot[NUM_POTENTIALS];
        double dirX[NUM_POTENTIALS], dirY[NUM_POTENTIALS];
        double pref;

        CellOccType occType;
        CellPlanType planType;
};

class Grid
{
    public:
        Grid();
        Cell* getCell(int x, int y);

        int getMapScale();
        int getMapWidth();
        int getMapHeight();

        void draw(int xi, int yi, int xf, int yf);

        int numViewModes;
        int viewMode;
        int firstPotViewMode;

        bool showValues;
        bool showArrows;

        pthread_mutex_t* mutex;

    private:
        int mapScale_; // Number of cells per meter
        int mapWidth_, mapHeight_; // in cells
        int numCellsInRow_, halfNumCellsInRow_;

        Cell* cells_;

        void drawCell(unsigned int i);
        void drawVector(unsigned int i);
        void drawText(unsigned int n);
};

#endif // __GRID_H__
