#ifndef CMODEL_H
#define CMODEL_H

#include "cv.h"
#include "estimator.h"
#include <list>

namespace SLAM{
using namespace std;
class CEstimator;
/**
@author Jorge Artieda
@brief Clase abstracta para implementar modelos de movimiento de c&aacute;maras.
*/

class CModel{
public:
       CModel();
virtual ~CModel();
        CvMat* getTransitionMatrix();
        int getStateNum();
        CvMat* getProcessNoiseCov();
        CvMat* getMeasurementMatrix();
        CvMat* getMeasurementNoiseCov();
        int getMeasurementNum();
        CvMat* getInputMatrix();
        CvMat* getInputCov();
        int getInputNum();
        virtual CvMat* getMeasurementVector()=0;
        void setEstimator(CEstimator *p);

protected:
        int StateNum;
        CvMat* TransitionMatrix;
        CvMat* ProcessNoiseCov;
        CvMat* MeasurementMatrix;
        CvMat* MeasurementNoiseCov;
        int MeasurementNum;
        CvMat* InputMatrix;
        CvMat* InputCov;
        int InputNum;
        CvMat* MeasurementVector;
        CEstimator *pEstimator;
};
}
#endif
