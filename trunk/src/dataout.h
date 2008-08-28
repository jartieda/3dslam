#ifndef H_DATAOUT
#define H_DATAOUT

#include <cv.h>
#include <iostream>
#include <fstream>

#include "map.h"
#include "kalman.h"
#include "tracker.h"
#include "modelcam.h"
#include "kalman.h"
#include "particlefilter.h"
namespace SLAM{
/**
@brief Almacena salidas del algoritmo
@author Jorge Artieda
*/
class CDataOut {
public:

CDataOut();
~CDataOut();
void Draw(IplImage *img);
void R_Out();
void Disp_out(IplImage *framecopy=NULL);
void Particle(IplImage *framecopy);
void Cam();
void Feat();
void Frame();
int iter;
void setMap(CMap *p);
void setDataCam(CDataCam *p);
void setModelCam(CModelCam *p);
void setTracker(CTracker *p);
void setKalman(CKalman *p);
void setParticle(CParticleFilter *p);
float randomVector(float max,float min);

protected:

ofstream FeatFile;
ofstream CamFile;
ofstream DispFile;
ofstream RFile;
ofstream FrameFile;

CMap *pMap;
CDataCam *pDataCam;
CModelCam *pModelCam;
CTracker *pTracker;
CKalman *pKalman;
CParticleFilter *pParticleFilter;
CvFont font;
double hScale;
double vScale;
int    lineWidth;
char strID[10];
};
}
#endif
