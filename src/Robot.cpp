#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;

    grid = new Grid();

    plan = new Planning();
    plan->setGrid(grid);
    plan->setMaxUpdateRange(base.getMaxLaserRange());

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=0;
    numViewModes=5;
    motionMode_=MANUAL_SIMPLE;

}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.2);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    currentPose_ = base.getOdometry();

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case POTFIELD_0:
            followPotentialField(0);
            break;
        case POTFIELD_1:
            followPotentialField(1);
            break;
        case POTFIELD_2:
            followPotentialField(2);
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_==WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;



}

void Robot::wanderAvoidingCollisions()
{
    float linVel = 0;
    float angVel = 0;

    float minSonar = base.getMinSonarValueInRange(0,7);

    if (minSonar < 0.6) {
        linVel = 0;
        angVel = 0.3 + static_cast<float>(rand())/(static_cast<float>(1.2)/(0.9));
    } else {
        linVel = 1;
        angVel = -0.2;
    }

    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow() {
    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);
    float minAllLasers = base.getMinLaserValueInRange(0, 180);

    float linVel = 0;
    float angVel = 0;

    // Parameters
    float wallDistance = 1.0;
    float rushDistance = wallDistance*2;
    float safeDistance = wallDistance/2;

    // Buffers for integration control
    int NUMBER_OF_MEASURES = 3;
    float *leftDistances = (float*) malloc(NUMBER_OF_MEASURES * sizeof(float));
    float *rightDistances = (float*) malloc(NUMBER_OF_MEASURES * sizeof(float));

    // Rotate integration control buffer
    for (int i = 0; i < NUMBER_OF_MEASURES-1; i++)
        leftDistances[i+1] = leftDistances[i];
    leftDistances[0] = minLeftLaser - wallDistance;

    for(int i = 0; i < NUMBER_OF_MEASURES-1; i++)
        rightDistances[i+1] = rightDistances[i];
    rightDistances[0] = minRightLaser - wallDistance;

    // Linear velocity control
    int isFrontTooClose = minFrontLaser < safeDistance;

    if (isFrontTooClose)
        linVel = 0;
    else
        linVel = 0.1;

    // Increment speed till find wall faster
    if (rushDistance*2 < minAllLasers)
        linVel = 1;

    // PID control
    // Constants
    float tp = 0.001;
    float td = 0.5;
    float ti = 0.00012;

    // CTE: CrossTalk Error = SetPoint - ProcessVariable.
    float CTE = 0;
    float dCTE = 0;
    float integral = 0;

    if (leftDistances[0] < rightDistances[0]) { // Left wall closer
        CTE = leftDistances[0];
        dCTE = -(CTE - leftDistances[1]);
        for(int i = 0; i < NUMBER_OF_MEASURES; i++)
            integral += leftDistances[i];

    } else if (rightDistances[0] < leftDistances[0]) { // Right wall closer
        CTE = rightDistances[0];
        dCTE = CTE - rightDistances[1];
        for(int i = 0; i < NUMBER_OF_MEASURES; i++)
            integral += rightDistances[i];

        tp *= -1;
    }

    //angular velocity (w)
    angVel = tp*CTE - td*dCTE - ti*integral;

    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::followPotentialField(int t)
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // how to access the grid cell associated to the robot position
    Cell* c=grid->getCell(robotX,robotY);

    float linVel, angVel;

    // TODO: define the robot velocities using a control strategy
    //       based on the direction of the gradient of c given by c->dirX[t] and c->dirY[t]




    base.setWheelsVelocity_fromLinAngVelocity(linVel,angVel);
}


///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

float Robot::getOccupancyFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

float Robot::getLogOddsFromOccupancy(float occupancy)
{
    return log(occupancy/(1.0-occupancy));
}

void Robot::mappingWithLogOddsUsingLaser()
{
    float lambda_r = 0.1; //  10 cm
    float lambda_phi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange * scale;

    int robotX = currentPose_.x * scale;
    int robotY = currentPose_.y * scale;
    float robotAngle = currentPose_.theta;

    float locc, lfree;
    locc = getLogOddsFromOccupancy(0.75);
    lfree = getLogOddsFromOccupancy(0.45);

    // how to access a grid cell:
    // Cell* cell=grid->getCell(robotX,robotY);

    // access log-odds value of variable in cell->logodds

    // how to convert logodds to occupancy values:
    // cell->occupancy = getOccupancyFromLogOdds(cell->logodds);

    // ============================================================================
    // you only need to check the cells at most maxRangeInt from the robot position
    // that is, in the following square region:
    //
    //  (robotX-maxRangeInt,robotY+maxRangeInt)  -------  (robotX+maxRangeInt,robotY+maxRangeInt)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotX-maxRangeInt,robotY-maxRangeInt)  -------  (robotX+maxRangeInt,robotY-maxRangeInt)

    for( int y = robotY - maxRangeInt; y <= robotY + maxRangeInt; y++){
        for(int x = robotX - maxRangeInt; x <= robotX + maxRangeInt; x++){
            Cell* cell = grid->getCell(x, y);

            float phi = RAD2DEG(atan2(y-robotY, x-robotX)) - robotAngle; //ângulo em relação ao robô
            phi = normalizeAngleDEG(phi);
            int k = base.getNearestLaserBeam(phi); // nº do laser

            // verifica se está dentro da área de visão
            if (abs(phi - base.getAngleOfLaserBeam(k)) < lambda_phi) {
                int r = sqrt(pow((x - robotX), 2) + pow(( y - robotY), 2)); //distância da célula em relação ao robô

                if (r < maxRangeInt) { // Dentro do alcance
                    int laserCellDistance = base.getKthLaserReading(k) * scale;

                    if (r < laserCellDistance) { // medida antes de obstáculo
                        cell->logodds += lfree;
                        cell->occupancy = getOccupancyFromLogOdds(cell->logodds);
                    } else if (r == laserCellDistance) { // medida no obstáculo
                        cell->logodds += locc;
                        cell->occupancy = getOccupancyFromLogOdds(cell->logodds);
                    }
                }
            }
        }
    }
}

void Robot::mappingUsingSonar()
{
    float lambda_r = 0.5;
    float lambda_phi = 30;

    int scale = grid->getMapScale();
    float maxRange = base.getMaxSonarRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    for(int x = robotX - maxRangeInt; x < robotX + maxRangeInt; x++)
        for(int y = robotY - maxRangeInt; y < robotY + maxRangeInt; y++){
            Cell* c = grid->getCell(x,y);

            float r = sqrt(pow(x - robotX,2) + pow(y - robotY,2)); //distância r medida
            float rMeters = r/scale;   //distância em metros

            float phi = RAD2DEG(atan2(y - robotY, x - robotX)) - robotAngle; //ângulo medido
            float phiNormalized = normalizeAngleDEG(phi); // ângulo normalizado

            float k = base.getNearestSonarBeam(phiNormalized); // laser mais próximo da medida
            float kAngle = base.getAngleOfSonarBeam(k); // ângulo do laser mais próximo
            float kReading = base.getKthSonarReading(k); // leitura do laser mais próximo

            float beta = lambda_phi/2;
            float murphyUpdate = ((maxRangeInt - rMeters)/maxRangeInt + (beta - fabs(phiNormalized - kAngle))/beta)/2 ;

            float occUpdate = 0.0;

            if((rMeters > std::min(((lambda_r/2) + kReading), (float) maxRange)) || (fabs(phiNormalized - kAngle) > lambda_phi/2)){
                occUpdate = 0.5; // desconhecido
            }
            else if((kReading < maxRangeInt) && (fabs(kReading - rMeters) < lambda_r/2)){
                occUpdate = lambda_r*(murphyUpdate) + lambda_r; // ocupado
            }
            else if(rMeters <= kReading){
                occUpdate = lambda_r*(1.0 - murphyUpdate); // livre
            }

            float numerador = occUpdate*c->occupancySonar;
            float denomirador = numerador + ((1.0 - occUpdate)*(1.0 - c->occupancySonar));
            float occ = numerador/denomirador;
            if(occ <= 0) {
                occ = 0.01;
            } else if(occ >= 1.0) {
                occ = 0.99;
            }

            c->occupancySonar = occ;
        }



}

void Robot::mappingWithHIMMUsingLaser()
{

    float lambda_r = 0.2;
    float lambda_phi = 1.0;

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX = currentPose_.x*scale;
    int robotY = currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    float pocc = 0.55;
    float pfree = 0.3;
    float locc = log(pocc/(1-pocc));
    float lfree = log(pfree/(1-pfree));

    for(int x = robotX - maxRangeInt; x < robotX + maxRangeInt; x++)
        for(int y = robotY - maxRangeInt; y < robotY + maxRangeInt; y++){
            Cell* c = grid->getCell(x,y);

            float r = sqrt(pow(x - robotX,2) + pow(y - robotY,2)); //distância r medida
            float rMeters = r/scale; //distância em metros

            float phi = RAD2DEG(atan2(y - robotY, x - robotX)) - robotAngle; //ângulo medido
            float phiNormalized = normalizeAngleDEG(phi); // ângulo normalizado

            float k = base.getNearestLaserBeam(phiNormalized); // laser mais próximo da medida
            float kAngle = base.getAngleOfLaserBeam(k); // ângulo do laser mais próximo
            float kReading = base.getKthLaserReading(k); // leitura do laser mais próximo

            if ((rMeters > std::min(((lambda_r/2) + kReading), (float) maxRange)) ||
               (fabs(phiNormalized - kAngle) > lambda_phi/2)) {
                c->himm += 0.0;
            }
            else if((kReading < maxRange) &&
                    (fabs(kReading - rMeters) < lambda_r/2)) {
                if(c->himm < 13)
                    c->himm += 3.0;
                else
                    c->himm = 15.0;
            }
            else if(rMeters <= kReading){
                if(c->himm == 0)
                    c->himm = 0.0;
                else
                    c->himm -= 1.0;
            }

            c->occupancy = getOccupancyFromLogOdds(c->himm);
        }
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}
