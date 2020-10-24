#ifndef __PLANNING_H__
#define __PLANNING_H__

class Planning;

#include <pthread.h>
#include <queue>
#include "Robot.h"
#include "Grid.h"

typedef struct
{
    int x,y;
} point2d;

typedef struct
{
    int minX, maxX, minY, maxY;
} bbox;


class Planning {
	public:
        Planning();
        ~Planning();

		void run();

        void initialize();

        void setNewRobotPose(Pose p);
        void setGrid(Grid* g);
        void setMaxUpdateRange(int r);

        void drawRoadmap();

        Grid* grid;

	private:

        void resetCellsTypes();
        void updateCellsTypes();
        void expandObstacles();
        void detectFrontiers();

        void initializePotentials();
        void iteratePotentials();
        void iterateDistortedPotentials();

        void updateGradient();
        void classificarOccType(Cell *c);
        void classificarPlanType(Cell *c, int cellX, int cellY);
        void initializeCellPreference(Cell *cell);
        void initializeCellPotential(Cell *cell);
        float potencialHarmonico(int cellX, int cellY, int potentialType);
        float potencialPreferencias(int cellX, int cellY);
        float gradientX(int potentialType, int cellX, int cellY);
        float gradientY(int potentialType, int cellX, int cellY);

        point2d robotPosition;
        bbox gridLimits;

        point2d newRobotPosition;
        bbox newGridLimits;

        int maxUpdateRange;

};


#endif /* __PLANNING_H__ */
