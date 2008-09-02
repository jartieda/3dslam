#include "tracker.h"
namespace SLAM{
using namespace std;

CTracker::CTracker()
{

}

CTracker::~CTracker()
{

}

void CTracker::setDataCam(CDataCam *p)
{
	pDataCam=p;
}
void CTracker::setModelCam(CModelCam *p)
{
	pModelCam=p;
}

void CTracker::setMap(CMap *p)
{
	pMap=p;
}

}
