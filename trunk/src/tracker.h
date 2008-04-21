#ifndef TRACKER_H
#define TRACKER_H

#include <cv.h>
#include <iostream>
#include "datacam.h" 
#include "map.h"
#include "modelcam.h"

/**
@author Jorge Artieda
@brief Clase abstracta para implementar seguidores
*/

class CTracker {
public:

CTracker();
virtual ~CTracker();

void setDataCam(CDataCam* p);
void setModelCam(CModelCam* p);

void setMap(CMap* p);
virtual void Match(IplImage *f)=0;
virtual int Init(IplImage *img,int **keys,CvMat **points)=0;
virtual void Descriptor(IplImage *img, CvPoint *point,int s, int *key)=0;
virtual int getFeatDim()=0;
CvSeq* feat;
CvMemStorage* storage;
protected:
CDataCam *pDataCam;
CModelCam *pModelCam;
CMap * pMap;

};
#endif
