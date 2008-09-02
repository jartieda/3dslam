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
virtual    ~CVehicle();
CvMat* getMeasurementVector();
virtual void ReadData(CvMat* Rob, CvMat* CamRob,CvMat* T)=0;
		/**
		 * Rellena el nombre del fichero cierra el fichero anterior y abre el nuevo
		 */
		void set_filename(char * name);

protected:
FILE *f;
char filename[100];
double pos_cov;
double ang_cov;
CvMat* RotCamRob;
CvMat* RotRobSrob;
CvMat* RotSrobScam;
CvMat* RotCamScam;
CvMat* VecRotCamScam;

};

}

#endif
