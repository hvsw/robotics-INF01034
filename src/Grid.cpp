#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <float.h> // DBL_MAX

#include "Grid.h"
#include "math.h"

Grid::Grid ()
{
    mapScale_ = 10;
    mapWidth_ = mapHeight_ = 2000;
    numCellsInRow_=mapWidth_;
    halfNumCellsInRow_=mapWidth_/2;

    cells_ = new Cell[mapWidth_*mapHeight_];

    for (unsigned int j = 0; j < numCellsInRow_; ++j)
    {
        for (unsigned int i = 0; i < numCellsInRow_; ++i)
        {
            unsigned int c = j*numCellsInRow_ + i;
            cells_[c].x = -halfNumCellsInRow_ + 1 + i;
            cells_[c].y =  halfNumCellsInRow_ - j;

            cells_[c].occupancy = 0.5;
            cells_[c].logodds = 0;
            cells_[c].occupancySonar = 0.5;
            cells_[c].logoddsSonar = 0;
            cells_[c].himm = 7;

            cells_[c].pot[0]  = 0.0;
            cells_[c].pot[1]  = 0.0;
            cells_[c].pot[2]  = 1.0;
            for(unsigned int k=0; k<NUM_POTENTIALS; k++){
                cells_[c].dirX[k] = 0.0;
                cells_[c].dirY[k] = 0.0;
            }
            cells_[c].pref = 0.0;

            cells_[c].occType = UNEXPLORED;
            cells_[c].planType = REGULAR;
        }
    }

    numViewModes=6;
    viewMode=2;
    firstPotViewMode=3;

    showValues=false;
    showArrows=false;
}

Cell* Grid::getCell (int x, int y)
{
    int i=x+halfNumCellsInRow_-1;
    int j=halfNumCellsInRow_-y;
    return &(cells_[j*numCellsInRow_ + i]);
}

int Grid::getMapScale()
{
    return mapScale_;
}

int Grid::getMapWidth()
{
    return mapWidth_;
}

int Grid::getMapHeight()
{
    return mapHeight_;
}

void Grid::draw(int xi, int yi, int xf, int yf)
{
    glLoadIdentity();

    for(int i=xi; i<=xf; ++i){
        for(int j=yi; j<=yf; ++j){
            drawCell(i+j*numCellsInRow_);
        }
    }

    if(showArrows){
        glPointSize(2);
        for(int i=xi; i<=xf; ++i){
            for(int j=yi; j<=yf; ++j){
                drawVector(i+j*numCellsInRow_);
            }
        }
    }

    if(showValues){
        for(int i=xi; i<=xf; i++){
            for(int j=yi; j<=yf; j++){
                drawText(i+j*numCellsInRow_);
            }
        }
    }
}

void Grid::drawCell(unsigned int n)
{
    float aux;

    if(viewMode==0){
        // DRAW OCCUPANCY GRID MAP
        aux=(1.0-cells_[n].occupancy);
        glColor3f(aux,aux,aux);
    }else if(viewMode==1){
        // DRAW HIMM GRID MAP
        aux=(16.0-cells_[n].himm)/16.0;
        glColor3f(aux,aux,aux);
    }else if(viewMode==2){
        // DRAW CLASSIFIED CELLS (FROM PLANNING)
        if(cells_[n].occType == FREE){
            if(cells_[n].planType == DANGER){
                glColor3f(0.6,0.0,0.0);
            }else if(cells_[n].planType == NEAR_WALLS){
                glColor3f(1.0,0.65,0.0);
            }else{
                glColor3f(1.0,1.0,0.7);
            }
        }else if(cells_[n].occType == UNEXPLORED){
            if(cells_[n].planType == FRONTIER){
                glColor3f(0.0,0.8,0.3);
            }else if(cells_[n].planType == FRONTIER_NEAR_WALL){
                glColor3f(0.0,0.6,0.1);
            }else if(cells_[n].planType == DANGER){
                glColor3f(0.2,0.2,0.2);
            }else if(cells_[n].planType == NEAR_WALLS){
                glColor3f(0.4,0.4,0.4);
            }else{
                glColor3f(0.6,0.6,0.6);
            }
        }else if(cells_[n].occType == OCCUPIED){
            glColor3f(0.3,0.0,0.0);
        }
    }else if(viewMode>=3 && viewMode<6){ //firstPotViewMode=3 NUM_POTENTIAL=3
        // DRAW POTENTIAL FIELDS
        aux=cells_[n].pot[viewMode-3];
        glColor3f(aux,aux,aux);
    }

    glBegin( GL_QUADS );
    {
        glVertex2f(cells_[n].x+1, cells_[n].y+1);
        glVertex2f(cells_[n].x+1, cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y+1);
    }
    glEnd();
}

void Grid::drawVector(unsigned int n)
{
    if(cells_[n].occType == FREE && viewMode>=firstPotViewMode && viewMode<firstPotViewMode+NUM_POTENTIALS){
        if(cells_[n].dirX[viewMode-firstPotViewMode] == 0.0 && cells_[n].dirY[viewMode-firstPotViewMode] == 0.0)
            glColor3f(1.0,0.7,0.0);
        else
            glColor3f(1.0,0.0,0.0);
        glLineWidth(1);
        glBegin( GL_LINES );
        {
            glVertex2f(cells_[n].x+0.5, cells_[n].y+0.5);
            glVertex2f(cells_[n].x+0.5+cells_[n].dirX[viewMode-firstPotViewMode], cells_[n].y+0.5+cells_[n].dirY[viewMode-3]);
        }
        glEnd();
        glBegin( GL_POINTS );
        {
            glVertex2f(cells_[n].x+0.5+cells_[n].dirX[viewMode-firstPotViewMode], cells_[n].y+0.5+cells_[n].dirY[viewMode-firstPotViewMode]);
        }
        glEnd();
    }
}

void Grid::drawText(unsigned int n)
{
    glRasterPos2f(cells_[n].x+0.25, cells_[n].y+0.25);
    std::stringstream s;
    glColor3f(0.5f, 0.0f, 0.0f);
//    s << std::setprecision(1) << std::fixed << cells_[n].pot;
    s << cells_[n].himm;


    std::string text=s.str();
    for (unsigned int i=0; i<text.size(); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
    }
}
