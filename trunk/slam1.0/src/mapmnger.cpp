#include "mapmnger.h"

namespace SLAM{
CMapMnger::CMapMnger()
{
    //ctor
}

CMapMnger::~CMapMnger()
{
    //dtor
}
void CMapMnger::newMap()
{
    CDataCam *t1 = new CDataCam();
    vDataCam.push_back(t1);
    pDataCam = t1;

    CMap *t2 = new CMap();
    vCMap.push_back(t2);
    pMap= t2;

    pEstimator= new CKalman();

    CvMat *t3; //fixme aqui tiene que ser capaz de conseguir la ultima del kalman
    vCovMat.push_back(t3);
    pCovMat = t3;

}
void CMapMnger::getFullMap()
{

}
void CMapMnger::saveMap(string filename)
{

}
}
