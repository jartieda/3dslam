#ifndef CVehicle_H
#define CVehicle_H
#include "cv.h"
#include <list>
#include "model.h"
using namespace std;
namespace SLAM{
/**
@author Jorge Artieda
@brief modelo de movimiento de c&aacute;mara con medida de la posici&oacute;n
*/

class CVehicle:public CModel{

public:
    CVehicle();
    virtual ~CVehicle();
    CvMat* getMeasurementVector();
    virtual void ReadData()=0;
    /**
     * Rellena el nombre del fichero cierra el fichero anterior y abre el nuevo
     **/
    void set_filename(string name);
    int iter;
    void set_iter(int i);
protected:
    FILE *f;
    string filename;
    double pos_cov;
    double ang_cov;
    CvMat* RotCamRob;
    CvMat* RotRobSrob;
    CvMat* RotSrobScam;
    CvMat* RotCamScam;
    CvMat* VecRotCamScam;
    CvMat* TransScam;
    CvMat* TransSRob;

};

}

#endif
