#ifndef MCL_H
#define MCL_H

#include <string>
#include <vector>
#include <random>

#include "Grid.h"
#include "Utils.h"

typedef struct{
    double rot1;
    double trans;
    double rot2;
} Action;

typedef struct{
    Pose p;
    double w;
} MCLparticle;

#define NUM_MCL 2

class MCL
{
public:
    MCL(float maxRange, std::string mapName, pthread_mutex_t* m);
    ~MCL();

    void run(const Action &u, const std::vector<float> &z);

    void draw(int set);

    int mapWidth;
    int mapHeight;

    bool transparency;

private:
    void readMap(std::string mapName);
    void initParticles(int set);

    void sampling(int set, const Action &u);
    void weighting(int set, const std::vector<float> &z);
    void resampling(int set);

    float computeExpectedMeasurement(int index, Pose &pose);

    void updateMeanAndCovariance(int set);

    std::default_random_engine* generator;

    CellOccType** mapCells;
    double scale;
    float maxRange;

    int numParticles;
    std::vector<MCLparticle> particles[NUM_MCL];

    float meanParticle[NUM_MCL][3];
    Pose meanParticlePose[NUM_MCL];
    float covAngle[NUM_MCL], covMajorAxis[NUM_MCL], covMinorAxis[NUM_MCL];

    pthread_mutex_t* mutex;

    static void Ellipse(float rx, float ry, float angle, int num_segments=80);


};

#endif // MCL_H
