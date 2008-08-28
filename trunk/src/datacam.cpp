#include "datacam.h"
namespace SLAM{
using namespace std;
/**
 * Constructor <br>
 * Reserva memoria para las matrices: <br>
 * <ul>
    <li>rotation</li>
    <li>rotMat</li>
    <li>translation</li>
    <li>trans_world</li>
    <li>calibration</li>
    <li>distortion</li>
   </ul>
 * Calibration se inicializa como la matriz identidad
 **/
CDataCam::CDataCam()
{

rotation=cvCreateMat(3,1,CV_32FC1);
rotMat=cvCreateMat(3,3,CV_32FC1);

translation=cvCreateMat(3,1,CV_32FC1);

calibration=cvCreateMat(3,3,CV_32FC1);
distortion=cvCreateMat(4,1,CV_32FC1);
cvSetIdentity( calibration, cvRealScalar(1) );

}
/** El destructor no hace nada **/
CDataCam::~CDataCam()
{
cvReleaseMat(&rotation);
cvReleaseMat(&rotMat);
cvReleaseMat(&translation);
cvReleaseMat(&calibration);
cvReleaseMat(&distortion);

}
void CDataCam::SetFx(double v)
{
   cvmSet(calibration,0,0,v);
}
void CDataCam::SetFy(double v)
{
   cvmSet(calibration,1,1,v);
}
void CDataCam::SetCx(double v)
{
   cvmSet(calibration,0,2,v);
}
void CDataCam::SetCy(double v)
{
   cvmSet(calibration,1,2,v);
}
/** Copia el vector v en el vecotor de rotaci&oacute;n y después se actualiza la matriz rotMat usando la función cvRodrigues2 **/
void CDataCam::SetRotation(CvMat *v)
{
   cvCopy(v,rotation);
   cvRodrigues2( rotation, rotMat);
}
/**
 * asigna el vector de translacion
 * tambien calcula el vector de translacion en coordenadas del mundo
 *
 **/
void CDataCam::SetTranslation(CvMat *v)
{
   cvCopy(v,translation);

}
}