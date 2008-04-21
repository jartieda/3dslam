#include "map.h"


/** Constructor <br>
 * ID=0, visible=0, inited=0
 **/
CMap::CMap()
{
   ID=0;
   visible =0;
   inited=0;
}

/** Destructor <br> No hace nada **/
CMap::~CMap()
{

}

/*!
    \fn CMap::add_key()
	Añade un punto con una clave de una longitud determinada <br>
	@param pts punto que se añade 
	@key clave que representa al punto en el mapa
 	@length tamano de la clabe en bytes
 */
void CMap::add_key(CvPoint pts, unsigned char *key,int length)
{
      CElempunto *pelempunto;
      pelempunto=new CElempunto();
      pelempunto->pto.x=pts.x;
      pelempunto->pto.y=pts.y;
      pelempunto->old_pto.x=pts.x;
      pelempunto->old_pto.y=pts.y;
      pelempunto->ID=get_ID();//bbdd.back()->ID+1;
      pelempunto->wx=0.;//pts[i].x;
      pelempunto->wy=0.;//pts[i].y;
      pelempunto->wz=0.0;
      pelempunto->state=st_empty;
	  pelempunto->key = new unsigned char[length];
	  for (int dd=0;dd<length;dd++)
		pelempunto->key[dd]=key[dd];

      bbdd.push_back(pelempunto);
      visible ++;      
   
}

/*!
    \fn CMap::add3D()
	Anade un punto al mapa (solo se usa en test);
 */

void CMap::add3D(double x, double y, double z)
{
   CElempunto *pelempunto;
      pelempunto=new CElempunto();
      pelempunto->pto.x=0;
      pelempunto->pto.y=0;
      pelempunto->old_pto.x=0;
      pelempunto->old_pto.y=0;
      pelempunto->ID=get_ID();//bbdd.back()->ID+1;
      pelempunto->wx=x;//pts[i].x;
      pelempunto->wy=y;//pts[i].y;
      pelempunto->wz=z;
      pelempunto->theta=0;
      pelempunto->phi=0;
      pelempunto->rho=999999999;
      pelempunto->state=st_inited; //al crearse como una posicion 3d ya esta iniciado.
      bbdd.push_back(pelempunto);
      visible++;
      inited++;
}
