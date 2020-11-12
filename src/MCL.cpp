#include "MCL.h"

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <GL/glut.h>

MCL::MCL(float maxRange, std::string mapName, pthread_mutex_t* m):
    maxRange(maxRange), mutex(m)
{
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);

    readMap(mapName);
    scale = 10;
    transparency = false;

    numParticles = 10000;

    for(int set=0; set<NUM_MCL; set++){
        initParticles(set);
        updateMeanAndCovariance(set);
    }
}

void MCL::run(const Action &u, const std::vector<float> &z)
{
    for(int set=0; set<NUM_MCL; set++){
        sampling(set,u);
        weighting(set,z);
        resampling(set);

        updateMeanAndCovariance(set);
    }
}


//////////////////////////////////////////////////
//// Métodos SAMPLING, WEIGHTING e RESAMPLING ////
//////////////////////////////////////////////////

void MCL::sampling(int set, const Action &u)
{
    /// TODO: propagar todas as particulas de acordo com o modelo de movimento baseado em odometria

    /// Odometria definida pela estrutura Action, composta por 3 variaveis double:
    /// rot1, trans e rot2
    std::cout << "rot1 " << RAD2DEG(u.rot1) << " trans " << u.trans << " rot2 " << RAD2DEG(u.rot2) << std::endl;

    /// Seguindo o modelo de Thrun, devemos gerar 3 distribuicoes normais, uma para cada componente da odometria

    /// Para definir uma distribuição normal X de media M e variancia V, pode-se usar:
    // std::normal_distribution<double> samplerX(M,V);
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerX(*generator)
    /// onde *generator é um gerador de numeros aleatorios (definido no construtor da classe)

    /// Para acessar a i-ésima particula, usar:
    // particles[set][i].p.x
    // particles[set][i].p.y
    // particles[set][i].p.theta




}

void MCL::weighting(int set, const std::vector<float> &z)
{
    /// TODO: faça a pesagem de todas as particulas

    if(set==0){
        /// Estratégia complexa

        /// elimine particulas fora do espaco livre
        /// compare as observacoes da particula com as observacoes z do robo
        // Use a funcao computeExpectedMeasurement(k, particles[i].p)
        // para achar a k-th observacao esperada da particula i
        /// normalize os pesos






    }else{
        /// Estratégia simplificada

        /// elimine particulas fora do espaco livre
        /// normalize os pesos




    }
}

void MCL::resampling(int set)
{
    // gere uma nova geração de particulas com o mesmo tamanho do conjunto atual
    std::vector<MCLparticle> nextGeneration;

    /// TODO: Implemente o Low Variance Resampling

    /// Para gerar amostras de uma distribuição uniforme entre valores MIN e MAX, pode-se usar:
    // std::uniform_real_distribution<double> samplerU(MIN,MAX));
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerU(*generator)
    /// onde *generator é um gerador de numeros aleatorios (definido no construtor da classe)






    /// Atualiza conjunto de particulas
    // particles[set] = nextGeneration;
}

/////////////////////////////////////////////////////
//// Método Auxiliar para o Modelo de Observacao ////
/////////////////////////////////////////////////////

float MCL::computeExpectedMeasurement(int index, Pose &pose)
{
    double angle = pose.theta + double(90-index)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange;
    }
    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale;

    double i=pose.x*scale;
    double j=pose.y*scale;
    for(int k=0;k<(int)(dist);k++){

        if(mapCells[(int)i][(int)j] == OCCUPIED){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale-(i+deltaX),2)+pow(pose.y*scale-(j+deltaY),2))/scale;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange;
}

//////////////////////////////////
//// Métodos de Inicializacao ////
//////////////////////////////////

void MCL::initParticles(int set)
{
    particles[set].resize(numParticles);

    std::uniform_real_distribution<double> randomX(0.0,mapWidth/scale);
    std::uniform_real_distribution<double> randomY(0.0,mapHeight/scale);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<numParticles; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles[set][i].p.x = randomX(*generator);
            particles[set][i].p.y = randomY(*generator);
            particles[set][i].p.theta = randomTh(*generator);

            // check if particle is valid (known and not obstacle)
            if(mapCells[(int)(particles[set][i].p.x*scale)][(int)(particles[set][i].p.y*scale)] == FREE)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles[set][i].p.x << ' '
                  << particles[set][i].p.y << ' '
                  << RAD2DEG(particles[set][i].p.theta) << std::endl;
    }
}

void MCL::readMap(std::string mapName)
{
    std::string name("../phir2framework/DiscreteMaps/");
    name += mapName;
    std::ifstream file;
    file.open(name.c_str(), std::ifstream::in);

    if( !file.good() )
    {
        std::cerr << "The file \"" << name << "\"  does not exist!" << std::endl;
        return;
    }

    // Read dimensions.
    file >> mapWidth >> mapHeight;
    std::cout << "map.width " << mapWidth << " map.height " << mapHeight << std::endl;

    mapCells = new CellOccType*[mapWidth];
        for(int i=0;i<mapWidth;i++)
            mapCells[i] = new CellOccType[mapHeight];

    // Read grid from file.
    char read;
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file >> read;
            switch(read)
            {
                case '1':
                    mapCells[x][y] = OCCUPIED;
                    break;
                case '0':
                    mapCells[x][y] = FREE;
                    break;
                case '-':
                    mapCells[x][y] = UNEXPLORED;
                    break;
            }
        }
    }

    file.close();
}

//////////////////////////////////////////////////////
//// Método de Atualização da Media e Covariancia ////
//////////////////////////////////////////////////////

void MCL::updateMeanAndCovariance(int set)
{
    // Compute Mean
    float sx=0, cx=0;
    meanParticle[set][0] = meanParticle[set][1] = 0.0;
    for(unsigned int i=0; i<numParticles; i++){
        meanParticle[set][0] += particles[set][i].p.x;
        meanParticle[set][1] += particles[set][i].p.y;
        sx += sin(particles[set][i].p.theta);
        cx += cos(particles[set][i].p.theta);
    }
    meanParticle[set][0] /= numParticles;
    meanParticle[set][1] /= numParticles;
    meanParticle[set][2] = atan2(sx,cx);

    // Compute Covariance Matrix (considering only x and y)
    float covariance[2][2];
    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] = 0;

    float diffx, diffy;
    for(unsigned int i=0; i<numParticles; i++){
        diffx  = meanParticle[set][0]-particles[set][i].p.x;
        diffy  = meanParticle[set][1]-particles[set][i].p.y;

        covariance[0][0] += diffx*diffx;    covariance[0][1] += diffx*diffy;
        covariance[1][0] += diffy*diffx;    covariance[1][1] += diffy*diffy;
    }

    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] /= numParticles;

    // Compute EigenValues and EigenVectors of covariance matrix
    float T = covariance[0][0] + covariance[1][1]; // Trace
    float D = covariance[0][0]*covariance[1][1] - covariance[0][1]*covariance[1][0]; // Determinant

    if((pow(T,2.0)/4.0 - D)<0.0)
        return;

    std::cout << "Covariance [" << covariance[0][0] << " " << covariance[0][1]
                        << "; " << covariance[1][0] << " " << covariance[1][1] << std::endl;

    float lambda1 = T/2.0 + sqrt(pow(T,2.0)/4.0 - D);
    float lambda2 = T/2.0 - sqrt(pow(T,2.0)/4.0 - D);
    float eigvec1[2], eigvec2[2];

    if(covariance[1][0]!=0.0){
        eigvec1[0] = lambda1 - covariance[1][1];    eigvec2[0] = lambda2 - covariance[1][1];
        eigvec1[1] = covariance[1][0];              eigvec2[1] = covariance[1][0];
    }else if(covariance[0][1]!=0.0){
        eigvec1[0] = covariance[0][1];              eigvec2[0] = covariance[0][1];
        eigvec1[1] = lambda1 - covariance[0][0];    eigvec2[1] = lambda2 - covariance[0][0];
    }else if(covariance[1][0]==0.0 && covariance[0][1]==0.0){
        eigvec1[0] = 1;    eigvec2[0] = 0;
        eigvec1[1] = 0;    eigvec2[1] = 1;
    }

    std::cout << "lambda " << lambda1 << " and " << lambda2 << std::endl;
    std::cout << "eigvectors [" << eigvec1[0] << "; " << eigvec1[1]
              << "] and [" << eigvec1[0] << "; " << eigvec1[1] << "]" << std::endl;

    // Compute direction of covariance ellipse
    //1st - Calculate the angle between the largest eigenvector and the x-axis
    covAngle[set] = RAD2DEG(atan2(eigvec1[1], eigvec1[0]));

    //2nd - Calculate the size of the minor and major axes
    covMajorAxis[set] = sqrt(lambda1);
    covMinorAxis[set] = sqrt(lambda2);
    std::cout << "covAngle " << covAngle[set] << " covMajorAxis " << covMajorAxis[set] << " covMinorAxis " << covMinorAxis[set] << std::endl;
}

////////////////////////////
//// Métodos de desenho ////
////////////////////////////

void MCL::draw(int set)
{
    // Draw map
    for(int x=0;x<mapWidth;x++){
        for(int y=0;y<mapHeight;y++){

            if(mapCells[x][y] == OCCUPIED)
                glColor3f(0.0,0.0,0.0);
            else if (mapCells[x][y] == UNEXPLORED)
                glColor3f(0.5,0.5,0.5);
            else
                glColor3f(1.0,1.0,1.0);

            glBegin( GL_QUADS );
            {
                glVertex2f(x  ,y  );
                glVertex2f(x+1,y  );
                glVertex2f(x+1,y+1);
                glVertex2f(x  ,y+1);
            }
            glEnd();
        }
    }

    double dirScale=5;
    glPointSize(4);
    glLineWidth(2);

    float alpha;
    if(transparency)
        alpha = 100.0/numParticles;
    else
        alpha = 1.0;

    // Draw particles
    for(int p=0;p<particles[set].size();p++){

        double x=particles[set][p].p.x*scale;
        double y=particles[set][p].p.y*scale;
        double th=particles[set][p].p.theta;

        // Draw point
        glColor4f(1.0,0.0,0.0,alpha);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();

        // Draw direction
        glColor4f(0.0, 0.0, 1.0, alpha);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+dirScale*cos(th), y+dirScale*sin(th));
        }
        glEnd();
    }
    glLineWidth(1);

    // Draw mean
    double xRobot = meanParticle[set][0]*scale;
    double yRobot = meanParticle[set][1]*scale;
    double angRobot = RAD2DEG(meanParticle[set][2]);

    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/5.0,1.0/5.0,1.0/5.0);
    glColor3f(0.0,1.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
    glScalef(5,5,5);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);

    // Draw Covariance Ellipse
    glColor3f(0.0,0.4,0.0);
    glLineWidth(2);
    double chisquare_val = 2.4477; // 95% confidence interval
    glTranslatef(xRobot,yRobot,0.0);
    MCL::Ellipse(chisquare_val*covMajorAxis[set]*scale, chisquare_val*covMinorAxis[set]*scale,covAngle[set]);
    glTranslatef(-xRobot,-yRobot,0.0);
    glLineWidth(1);
}

void MCL::Ellipse(float rx, float ry, float angle, int num_segments)
{
    float theta = 2 * M_PI / float(num_segments);
    float c = cos(theta);//precalculate the sine and cosine
    float s = sin(theta);
    float t;

    float x = 1;//we start at angle = 0
    float y = 0;

    glRotatef(angle,0,0,1);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x*rx, y*ry);//output vertex

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
    glRotatef(-angle,0,0,1);
}
