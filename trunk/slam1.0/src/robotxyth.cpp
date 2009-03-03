#include "robotxyth.h"
namespace SLAM{
CRobotXYTh::CRobotXYTh()
{
    //ctor
        cvZero(RotSrobScam);
        cvmSet(RotSrobScam,0,0,1);
        cvmSet(RotSrobScam,2,1,1);
        cvmSet(RotSrobScam,1,2,-1);
        cvSetIdentity(RotCamRob);
}

CRobotXYTh::~CRobotXYTh()
{
    //dtor
}

void CRobotXYTh::ReadData()
{
    double x,y,northing;
    double th;
    char filein[400];
    sprintf(filein,filename.c_str(),iter);
    f=fopen(filein,"r");

    if (f!=0){
    fscanf(f,"%lf %lf %lf\n",&x,&y,&northing);
    fscanf(f,"%*lf %*lf %*lf\n");
    fscanf(f,"%*lf %*lf %*lf\n");
    fscanf(f,"%*lf %*lf %*lf\n");
    fclose(f);
    double th = 90 -northing;
    cvmSet(RotRobSrob,0,0,cos(th));
    cvmSet(RotRobSrob,1,0,sin(th));
    cvmSet(RotRobSrob,0,0,0);
    cvmSet(RotRobSrob,0,1,-sin(th));
    cvmSet(RotRobSrob,1,1,cos(th));
    cvmSet(RotRobSrob,2,1,0);
    cvmSet(RotRobSrob,0,2,0);
    cvmSet(RotRobSrob,1,2,0);
    cvmSet(RotRobSrob,2,2,1);

    }else{
        cout<<"ERROR robotxyth: file not open"<<endl;
        exit(-1);
    }
}
}
