#ifndef CMODEL_H
#define CMODEL_H
#include "cv.h"
#include "modelcam.h"
#include "map.h"

#include <list>

using namespace std;
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
        void setModelCam(CModelCam *p);///< enlaza la clase kalman con la clase model cam
        void setDataCam(CDataCam *p);///< enlaza la clase kalman con la clase data cam
        void setMap(CMap *p); ///< enlaza kalman con mapa
        virtual CvMat* getMeasurementVector()=0;

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
        CModelCam *pModelCam;///< puntero a modelo de la camara para proyectar y jacobiano
        CDataCam *pDataCam;///< puntero a datos de la camara
        CMap *pMap; ///<puntero a datos del mapa

};

#endif
