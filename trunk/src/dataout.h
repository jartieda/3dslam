#ifndef H_DATAOUT
#define H_DATAOUT

#include <cv.h>
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include "map.h"
#include "estimator.h"
#include "tracker.h"
#include "cv.h"

namespace SLAM{

/**
@brief Almacena salidas del algoritmo
@author Jorge Artieda
*/
class CDataOut {


public:

CDataOut(std::string res);
~CDataOut();
void Draw(IplImage *img);
void R_Out();
void Disp_out(IplImage *framecopy=NULL);
void Particle(IplImage *framecopy);
void Cam();
void Feat();
void Frame();
int iter;
//void setMap(CMap *p);
//void setDataCam(CDataCam *p);
void setModelCam(CModelCam *p);
void setTracker(CTracker *p);
void setEstimator(CEstimator *p);
float randomVector(float max,float min);

std::string resdir;

CModelCam *pModelCam;
CTracker *pTracker;
CEstimator *pEstimator;

protected:

std::ofstream FeatFile;
std::ofstream CamFile;
std::ofstream DispFile;
std::ofstream RFile;
std::ofstream FrameFile;


CvFont font;
double hScale;
double vScale;
int    lineWidth;
char strID[10];
};
}
#endif
