#ifndef CFreeCam_H
#define CFreeCam_H

#include "cv.h"
#include <list>

#include "model.h"

namespace SLAM{
using namespace std;

/**
@author Jorge Artieda
@brief modelo de movimiento de la c&aacute;mara sin medida del movimiento
*/

class CFreeCam:public CModel{
public:
    CFreeCam();
    virtual ~CFreeCam();
    CvMat* getMeasurementVector();

protected:

};
}
#endif
