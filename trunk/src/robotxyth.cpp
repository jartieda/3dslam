#include "robotxyth.h"
namespace SLAM{
CRobotXYTh::CRobotXYTh()
{
    //ctor
}

CRobotXYTh::~CRobotXYTh()
{
    //dtor
}

void CRobotXYTh::ReadData(CvMat *Rob, CvMat* CamRob,CvMat* T)
{
    double x,y,northing;
    double th;
    fscanf(f,"%lf %lf %lf\n",&x,&y,&northing);
    fscanf(f,"%*lf %*lf %*lf\n");
    fscanf(f,"%*lf %*lf %*lf\n");
    fscanf(f,"%*lf %*lf %*lf\n");

}
}
