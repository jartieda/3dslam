#include "freecam.h"
namespace SLAM{
/**
 * Cosntructor <br>
 * Crea las matrices<br>
 * El modelo de la c&aacute;mara se pone comoun solido de movimiento libre <br>
 * Rellena la matriz de la covarianza del proceso <br>
 * Covarianza de la posición es 3 y coovarianza del ángulo 0.1 <br>
 * No se rellena la matriz de medida porque no hay medida
 **/
CFreeCam::CFreeCam()
{
    StateNum=12;
    TransitionMatrix=cvCreateMat(12,12,CV_32FC1);
    ProcessNoiseCov=cvCreateMat(12,12,CV_32FC1);
    MeasurementNum=0;
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
      cvmSet(ProcessNoiseCov,2*i,2*i,10);
      cvmSet(ProcessNoiseCov,2*i+1,2*i+1,0.05);
   }
//covarianzas del angulo
   for (int i=3;i<6;i++){
      cvmSet(ProcessNoiseCov,2*i,2*i,20);
      cvmSet(ProcessNoiseCov,2*i+1,2*i+1,0.05);
   }
}

/** Destructor **/
CFreeCam::~CFreeCam()
{
    cvReleaseMat(&TransitionMatrix);
    cvReleaseMat(&ProcessNoiseCov);
    cvReleaseMat(&MeasurementVector);
}

/** No hace nada porque no hay medida **/
CvMat* CFreeCam:: getMeasurementVector()
{
	return MeasurementVector;
}
}
