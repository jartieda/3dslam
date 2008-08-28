#ifndef MODELCAM_H
#define MODELCAM_H

#include <cv.h>
#include <iostream>
#include "datacam.h"
#include "map.h"
namespace SLAM{
/**
@author Jorge Artieda
@brief Ecuaciones del modelo de c&aacute;mara pin-hole
*/

class CModelCam {
public:

CModelCam();
virtual ~CModelCam();

void ProjectPoints();

/** @brief funcion de proyeccion de puntos de opencv
 *
 */
void cvProjectPoints3( const CvMat* obj_points,
                        const CvMat* r_vec,
                        const CvMat* t_vec,
                        const CvMat* A,
                        const CvMat* dist_coeffs,
                        CvMat* img_points, CvMat* dpdr,
                        CvMat* dpdt, CvMat* dpdf,
                        CvMat* dpdc, CvMat* dpdk ,CvMat* dpdw);

void cvProject_1_pto(CvMat* obj, CvMat* img,
                        CvMat* dpdr, CvMat* dpdt, CvMat* dpdw);
void cvInverseParam(CvMat** h,CvPoint pto);
void setDataCam(CDataCam* p);
void setMap(CMap* p);
void getJInit(CvMat *Jpos, CvMat *Jpix, CvPoint pto);
private:
CDataCam *pDataCam;
CMap * pMap;
};
}
#endif
