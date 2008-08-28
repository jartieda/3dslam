#ifndef CMAP_H
#define CMAP_H
#include "elempunto.h"
#include <list>

namespace SLAM{
using namespace std;
/**
@author Jorge Artieda
@brief Contiene el mapa
*/
class CMap{
public:

    list<CElempunto*> bbdd;
    CMap();
    ~CMap();
    void add3D(double x, double y, double z);
    void add_key(CvPoint pts, unsigned char *key,int length);

    int get_ID(){
       return ++ID;
    }
    int visible;
    int inited;///<numero de puntos iniciads puntos iniciados
    int ID;

};
}

#endif
