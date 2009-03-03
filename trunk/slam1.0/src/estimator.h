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

};

}
#endif
