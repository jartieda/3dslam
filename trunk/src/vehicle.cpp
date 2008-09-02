#include "vehicle.h"
namespace SLAM{
CVehicle::CVehicle()
{
    ang_cov=0.1;
    pos_cov=3;
//    sprintf(filename,"G:\\airview\\2007.09.10 ROZAS\\11-04-19gps3.txt");

    StateNum=12;
    TransitionMatrix=cvCreateMat(12,12,CV_32FC1);
    ProcessNoiseCov=cvCreateMat(12,12,CV_32FC1);;
    MeasurementMatrix=cvCreateMat(6,12,CV_32FC1);
    MeasurementNoiseCov=cvCreateMat(6,6,CV_32FC1);
    MeasurementNum=6;
//    InputMatrix;
//    InputCov;
    InputNum=0;
    MeasurementVector=cvCreateMat(6,1,CV_32FC1);

    cvSetIdentity( TransitionMatrix);
    for (int i=0;i<6;i++){
       cvmSet(TransitionMatrix,2*i,2*i,2);
       cvmSet(TransitionMatrix,2*i,2*i+1,-1);
       cvmSet(TransitionMatrix,2*i+1,2*i,1);
       cvmSet(TransitionMatrix,2*i+1,2*i+1,0);
    }

    cvSetIdentity( ProcessNoiseCov, cvRealScalar(1) );

//covarianzas de las posicion
   for (int i=0;i<3;i++){
      cvmSet(ProcessNoiseCov,2*i,2*i,pos_cov);
      cvmSet(ProcessNoiseCov,2*i+1,2*i+1,pos_cov);
   }
//covarianzas del angulo
   for (int i=3;i<6;i++){
      cvmSet(ProcessNoiseCov,2*i,2*i,ang_cov);
      cvmSet(ProcessNoiseCov,2*i+1,2*i+1,ang_cov);
   }
//modelo de la medida
   cvSetIdentity( MeasurementMatrix,cvRealScalar(0) );
   for (int i=0;i<6;i++){
	cvmSet(MeasurementMatrix,i,2*i,1);
   }

//covarianza de la medida
   cvSetIdentity( MeasurementNoiseCov, cvRealScalar(0.01) );
   //f=fopen("/media/WOXTER/datos/univ-alberta-vision-sift/ualberta-csc-flr3-vision/test2.pos","r");
   //f=fopen("/media/WOXTER/airview/2007.08.21 ROZAS/track35xyzInterCeroAng","r");
 //  f=fopen(filename,"r");

     RotCamRob=cvCreateMat(3,3,CV_32FC1);
     RotRobSrob=cvCreateMat(3,3,CV_32FC1);
     RotSrobScam=cvCreateMat(3,3,CV_32FC1);
     RotCamScam=cvCreateMat(3,3,CV_32FC1);
     VecRotCamScam=cvCreateMat(3,1,CV_32FC1);
     TransSRob=cvCreateMat(3,1,CV_32FC1);
     TransScam=cvCreateMat(3,1,CV_32FC1);
}


CVehicle::~CVehicle()
{
}

CvMat* CVehicle:: getMeasurementVector()
{

    CvMat *t1;
    t1=cvCreateMat(3,3,CV_32FC1);

    ReadData();

    cvMatMul( RotRobSrob,RotSrobScam,t1 );
    cvMatMul(RotCamRob,t1,RotCamScam);
    cvRodrigues2( RotCamScam, VecRotCamScam);

    cvMatMul(RotSrobScam ,TransSRob,TransScam);

    cvmSet(MeasurementVector,0,0,cvmGet(TransScam,0,0));
    cvmSet(MeasurementVector,1,0,cvmGet(TransScam,1,0));
    cvmSet(MeasurementVector,2,0,cvmGet(TransScam,2,0));
    cvmSet(MeasurementVector,3,0,cvmGet(VecRotCamScam,0,0));
    cvmSet(MeasurementVector,4,0,cvmGet(VecRotCamScam,1,0));
    cvmSet(MeasurementVector,5,0,cvmGet(VecRotCamScam,2,0));

    cvReleaseMat(&t1);



	return MeasurementVector;
}

/*
 * Rellena el nombre del fichero cierra el fichero anterior y abre el nuevo
 */
void CVehicle::set_filename(string name)
{
    filename = name;

}
void CVehicle::set_iter(int i){
    iter=i;
}

}

