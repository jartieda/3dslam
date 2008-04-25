#include "trackerfile.h"
#define FORIGEN "F:\\SLAM\\Datos\\datos_manuales\\datos%3d.txt"
CTrackerFile::CTrackerFile()
{
    //ctor
}

CTrackerFile::~CTrackerFile()
{
    //dtor
}

 void CTrackerFile::Match(IplImage *im){

      char fichero[200];
     sprintf(fichero,FORIGEN,frame);
     f=fopen(fichero, "r");
     int num;
     int x,y;
     for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
         	It != pMap->bbdd.end(); It++ )
	{
        if((*It)->state==st_inited)
        {
            std::cout << "borropuntos: "<<(*It)->pto.x<<" "<<(*It)->pto.y<<std::endl;
            (*It)->state = st_no_view;
            pMap->visible--;
         }
	}
    while (fscanf(f,"%d %d %d\n", &num, &x, &y)!=EOF)
     {
         list<CElempunto*>::iterator It=pMap->bbdd.begin();
        while  (((*It)->ID!=num)&&( It != pMap->bbdd.end()))
        {
            It++ ;
        }
        if( It != pMap->bbdd.end()){
           (*It)->old_pto.x=(*It)->pto.x;
           (*It)->old_pto.y=(*It)->pto.y;
           (*It)->pto.x=(int)x;
           (*It)->pto.y=(int)y;
		   if((*It)->state==st_no_view)
		   {
                (*It)->state=st_inited;
                pMap->visible++;
		   }
        }else{

        }
    }

        //end if dentro proj

 }
 int CTrackerFile::Init(IplImage *img,int **keys,CvMat **points){
     if (first==true){
         char fichero[200];
         sprintf(fichero,FORIGEN,frame);
         f=fopen(fichero, "r");
         int numptos=0;
         int num;
         int x,y;
         *keys = new int[64*40];// cvCreateMat(feat->total,64,CV_32FC1) ;
        *points = cvCreateMat(40,4,CV_32FC1) ;

         while (fscanf(f,"%d %d %d\n", &num, &x, &y)!=EOF)
         {
            cvmSet(*points,numptos,0,x) ;
            cvmSet(*points,numptos,1,y) ;
            cvmSet(*points,numptos,2,1) ;
            cvmSet(*points,numptos,3,0) ;
            numptos++;
         }
         first=false;
         return numptos;
     }else{
         return 0;
     }
 }
 void CTrackerFile::Descriptor(IplImage *img, CvPoint *point,int s, int *key){

 }
 int CTrackerFile::getFeatDim(){
      return 64;///realmente no almaceno ningun punto.
 }