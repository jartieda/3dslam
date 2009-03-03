#ifndef DATACAM_H
#define DATACAM_H

#include <cv.h>
#include <iostream>

namespace SLAM{
/**
@brief Par&aacute;metros intr&iacute;nsecos y extr&iacute;nsecos de la c&aacute;mara
@author Jorge Artieda

*/

class CDataCam {
public:

CDataCam();
~CDataCam();

void SetFx(double v);
void SetFy(double v);
void SetCx(double v);
void SetCy(double v);
void SetRotation(CvMat *v);
void SetTranslation(CvMat *v);

CvMat *rotation;///< vector de rotacion
CvMat *rotMat;///< matriz de rotacion
CvMat *translation;///<vector de translacion relativo

CvMat *calibration;///<matriz de calibracion
CvMat *distortion;///<matriz de distrosion
//CvMat *abs_pos;
int frame_width;
int frame_height;

};
}
#endif
