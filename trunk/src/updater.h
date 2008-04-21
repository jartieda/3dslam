#include "map.h"
#include "kalman.h"
#include "tracker.h"
#include "modelcam.h"
#include <vector>
#include "highgui.h"
/**
@author Jorge Artieda
@brief Clase para el mantenimiento del mapa
*/
struct param
{
  double px[6];
  double py[6]; 
      
};
struct point
{
       double ground[2];
       double img[2];   
       int ID;    
};
class CUpdater{
public:
CUpdater(CMap *pMap_,CDataCam* pDataCam_);
CUpdater();
void setMap(CMap *p);
void setDataCam(CDataCam *p);
void setModelCam(CModelCam *p);
void setTracker(CTracker *p);
void TestRANSAC();
void randsample (vector<point> *data, int n,vector<point> *maybeinliners);
double fits(point p, param model);
double fit(param *model, vector<point> points);
void ranasac(vector<point> orig_data,int k, int n,double t,int d);
param maybemodel;
param bettermodel;
param bestfit ;

~CUpdater();
CMap *pMap;
CDataCam *pDataCam;
CModelCam *pModelCam;
CTracker *pTracker;
int update();
int busca_esquinas(IplImage *f,CvPoint* pts,int count);
int busca_posibles_para_anadir(IplImage *f,IplImage *f2,int faltan);

int Add(IplImage *f,IplImage *f2);
int AddByHand(IplImage *f);
int remove();

int rematch();
int border;
int num_feat_min;
int num_feat_max;
double point_sep;
double calidad_min_punto;
double depth;
};
