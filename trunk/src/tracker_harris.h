#ifndef TRACKER_HARRIS_H
#define TRACKER_HARRIS_H

#include <cv.h>
#include <iostream>
#include "datacam.h" 
#include "map.h"
#include "tracker.h"

/**
@author Jorge Artieda
@brief Seguidor de puntos usando el metodo de harris
*/

class CTracker_harris:public CTracker {
public:

CTracker_harris();
virtual ~CTracker_harris();

/* Implementación de funciones virtuales */

void Match(IplImage *f);
void Init(IplImage *img,CvMat **keys,CvMat **points);
/** Obtiene el descriptor asociado a un punto con una escala detrminada **/
void Descriptor(IplImage *img, CvPoint *point,int s, int *key);
/** Devuelve el tamaño en bytes del vector de características **/
int getFeatDim();
/** Inicializa los primeros puntos puede ser un proceso diferente de otros**/
int Init(IplImage *img,int **keys,CvMat **points);

private:
IplImage *zonabusca;
IplImage *corr;
double minVal;
double maxVal;
CvPoint minLoc;
CvPoint maxLoc;

};
#endif
