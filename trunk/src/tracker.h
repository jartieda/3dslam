#ifndef TRACKER_H
#define TRACKER_H

#include <cv.h>
#include <iostream>
#include "datacam.h"
#include "map.h"
#include "modelcam.h"
#include "mapmnger.h"

namespace SLAM{
/**
@author Jorge Artieda
@brief Clase abstracta para implementar seguidores
*/

class CTracker {
public:

CTracker();
virtual ~CTracker();

void setModelCam(CModelCam* p);
//void setDataCam(CDataCam* p);
//void setMap(CMap* p);
void setMapMnger(CMapMnger *p);

virtual void Match(IplImage *f)=0;
virtual int Init(IplImage *img,int **keys,CvMat **points)=0;
virtual void Descriptor(IplImage *img, CvPoint *point,int s, int *key)=0;
virtual int getFeatDim()=0;
CvSeq* feat;
CvMemStorage* storage;
protected:
CModelCam *pModelCam;
//CMap * pMap;
//CDataCam *pDataCam;
CMapMnger *pMapMnger;

};
}
#endif
