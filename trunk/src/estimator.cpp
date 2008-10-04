#include "estimator.h"
namespace SLAM{
void CEstimator::setModelCam(CModelCam *p)
{
  pModelCam=p;
}

void CEstimator::newMap()
{
    CDataCam *t1 = new CDataCam();
    vDataCam.push_back(t1);
    pDataCam = t1;

    CMap *t2 = new CMap();
    vCMap.push_back(t2);
    pMap= t2;

    CvMat *t3; //fixme aqui tiene que ser capaz de conseguir la ultima del kalman
    vCovMat.push_back(t3);
    t3=getCovMat();

    ResetEstimator();
}

void CEstimator::getFullMap()
{

}
void CEstimator::saveMap(string filename)
{

}

}
