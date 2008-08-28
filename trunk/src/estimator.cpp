#include "estimator.h"
namespace SLAM{
void CEstimator::setModelCam(CModelCam *p)
{
  pModelCam=p;
}

void CEstimator::setDataCam(CDataCam *p)
{
  pDataCam=p;
}
void CEstimator::setMap(CMap *p)
{
  pMap=p;
}
}
