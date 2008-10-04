#ifndef KALMAN_H
#define KALMAN_H

#include "highgui.h"
#include "estimator.h"

namespace SLAM{

/**
@author Jorge Artieda
@brief implementaci&oacute;n del filtro de kalman
*/
class CKalman: public CEstimator {

public:
CKalman();
virtual ~CKalman();

void Predict();
void Correct();
void Test();///<test de mahalanobis
void initState();
CvMat *getCovMat();


CvKalman *pKalman;///<estructura que contiene las variables internas del kalman
CvKalman *pKalmanMem;///<estructura que contiene un kalman gigante para reservar memoria

CvMat *rotation;///<rotacion para devolver los valores de la camara
CvMat *trans;///<translacion  para devolver los valores de la camara

CvMat *measurement;
//int dim_measurement;
//int dim_state;
static const int fdims=6;
void UpdateJacob();///< actualiza el jacobiano
void UpdateMatrixSize(); ///< incrementa el tamaño del filtro
void transMat(CvMat* o_mat, CvMat* d_mat);
void SetKalman(CvKalman*pk,int state, int meas, int input);
void Print(int iter);
float xi[31];///<probabilidad acumulada de la distribucion chi cuadrado
void setModel(CModel *p);///< enlaza la clase calman con el modelo de vehÃ­culo
void Predict_FAST();
void Correct_FAST();
void NewPointCov(int old_state,CvMat *h,int xpix,int ypix);
void ResetEstimator();
};
}
#endif
