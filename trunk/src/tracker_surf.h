#ifndef TRACKER_SURF_H
#define TRACKER_SURF_H

#include <cv.h>
#include <iostream>
#include "datacam.h"
#include "map.h"

#include "surf.h"
#include "tracker.h"

namespace SLAM{
/**
@author Jorge Artieda
@brief Seeded Up Robust Features
**

*/

class CTracker_surf: public CTracker {
public:

CTracker_surf();
~CTracker_surf();

void Match(IplImage *f);
int Init(IplImage *img,int **keys,CvMat **points);
/** Obtiene el descriptor asociado a un punto con una escala detrminada **/
void Descriptor(IplImage *img, CvPoint *point,int s, int *key);
/** Devuelve el tamaño en bytes del vector de características **/
int getFeatDim();
private:

IplImage *img;
IplImage *fImg;

CSurf surf;
int levels;
};
}
#endif
