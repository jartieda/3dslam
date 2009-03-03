#ifndef CELEMPUNTO_H
#define CELEMPUNTO_H
#include "cv.h"
#include <list>
namespace SLAM{
using namespace std;
/**
 * tipo enumerado para describir los diferentes estados de un punto en su inicializaci&oacute;n <br>
 *
**/
enum st_enum {
	st_empty,///< punto creado 0
	st_1st_point,///< punto visto por primera vez 1 (ESTO YA NO SE USA)
	st_ready_to_init, ///< listo para ser triangulado. 2
	st_inited, ///<ya inicializado con una posicion 3D. 3
	st_no_view ///<fuera de vista. (pero iniciado). 4
};

/**
@author Jorge Artieda
@elemento del mapa
*/

class CElempunto{
public:
   CElempunto();
   ~CElempunto();
   CvPoint pto;///<posicion del punto en la imagen visto por la camara real
   CvPoint old_pto;///<ultima posción visto el punto
   int ID;///<identificador del punto. número sequencial.

   double wx,wy,wz;///<Posicion del punto en el espacio
   double theta,phi,rho;///<inverse depth parametrization...

   double wx_s,wy_s,wz_s;///<Estimaci&oacute;n del error de est punto
   double projx, projy;///<Puntos proyectados por modelo de la camara

   CvMat *dpdr; ///<derivada del punto respecto de la rotacion
   CvMat *dpdt; ///<derivada del punto respecto de la translacion
   CvMat *dpdw; ///<derivada del punto respecto a la posicion del punto tridimensional
   st_enum state ; ///<estado del punto
   unsigned char *key; ///<indentificador sift o surf
   int count; ///<número de veces que ha sido visto un punto
};
}
#endif
