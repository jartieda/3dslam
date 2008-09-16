#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "modelcam.h"
#include "map.h"
#include "model.h"
#include "mapmnger.h"

#include "cv.h"
namespace SLAM{
/**
@author Jorge Artieda
@brief implementaci&oacute;n del filtro de estimación abstracto
*/

class CEstimator {

public:

virtual void Predict()=0;
virtual void Correct()=0;
virtual void Test()=0;
virtual void initState()=0;
virtual void UpdateMatrixSize()=0;

CModelCam *pModelCam;///< puntero a modelo de la camara para proyectar y jacobiano
void setModelCam(CModelCam *p);///< enlaza la clase kalman con la clase model cam

//CDataCam *pDataCam;///< puntero a datos de la camara
//void setDataCam(CDataCam *p);///< enlaza la clase kalman con la clase data cam

CModel *pModel; ///<puntero a model de vehiculo
virtual void setModel(CModel *p)=0;///< enlaza la clase calman con el modelo de vehÃ­culo

//CMap *pMap; ///<puntero a datos del mapa
//void setMap(CMap *p); ///< enlaza kalman con mapa
CMapMnger *pMapMnger;
void setMapMnger(CMapMnger *p);

};
}
#endif
