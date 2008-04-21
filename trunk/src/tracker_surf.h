#ifndef TRACKER_SURF_H
#define TRACKER_SURF_H

#include <cv.h>
#include <iostream>
#include "datacam.h" 
#include "map.h"

#include "surf.h"
#include "tracker.h"
/**
@author Jorge Artieda
@brief Seeded Up Robust Features
**

*/

class CTracker_surf: public CTracker {
public:

CTracker_surf();
~CTracker_surf();

//void setDataCam(CDataCam* p);
//void setMap(CMap* p);
void Match(IplImage *f);
int Init(IplImage *img,int **keys,CvMat **points);
void Descriptor(IplImage *img, CvPoint *point,int s, int *key);
 int getFeatDim();
private:

IplImage *img;
IplImage *fImg;
//cvNamedWindow("vent",1);

//Estas variables deberían formar parte de una futura clase abstracta
//CDataCam *pDataCam;
//CMap * pMap;


CSurf surf;
    int levels;
};
#endif
