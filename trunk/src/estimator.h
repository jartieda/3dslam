#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "cv.h"
#include "modelcam.h"
#include "model.h"
#include <vector>

namespace SLAM{

/**
@author Jorge Artieda
@brief implementaci&oacute;n del filtro de estimación abstracto
*/
class CModel;
class CModelCam;

class CEstimator {

public:
CEstimator(){
}
virtual ~CEstimator()
{

}
/**Virtual methods for estimator*/
virtual void Predict()=0;
virtual void Correct()=0;
virtual void Test()=0;
virtual void initState()=0;
virtual void UpdateMatrixSize()=0;

/**methods for map managing */
void newMap(); ///<Creates a new blank map
void getFullMap(); ///<creates
void saveMap(std::string filename);
virtual void ResetEstimator()=0;///<Resets the estimate state to initial values
virtual void ManageMap(){};
virtual CvMat* getCovMat()=0;

CModelCam *pModelCam;///< puntero a modelo de la camara para proyectar y jacobiano
void setModelCam(CModelCam *p);///< enlaza la clase kalman con la clase model cam
CModel *pModel; ///<puntero a model de vehiculo
virtual void setModel(CModel *p)=0;///< enlaza la clase calman con el modelo de vehÃ­culo
CDataCam *pDataCam;///< puntero a datos de la camara
CMap *pMap; ///<puntero a datos del mapa
CvMat*pCovMat; ///<puntero a matriz de covarianzas actual.

protected:
std::vector <CDataCam*> vDataCam; ///<Camera data structures vector
std::vector <CMap*> vCMap; ///<Feature Map vector
std::vector <CvMat*> vCovMat; ///<Covariance Matrix vector

};

}
#endif
