#include "trackerfile.h"
//#define FORIGEN "F:\\SLAM\\Datos\\datos_manuales\\datos%0.3d.txt"
//#define FORIGEN "F:\\SLAM\\Datos\\datos_manuales\\redatos%0.3d.txt"
//#define FORIGEN "F:\\SLAM\\Datos\\dat2\\dat\\redatos%0.4d.txt"
#define FORIGEN "/media/WOXTER/SLAM/Datos/dat2/dat/redatos%0.4d.txt"

namespace SLAM{
CTrackerFile::CTrackerFile():first(true),frame(0)
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
     frame+=3;
     cout<<fichero<<endl;
     fclose(f);
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
	cout<<"aqui casco"<<endl;
	int dumb;
    while (fscanf(f,"%d %d %d %d\n", &num, &y, &x,&dumb)!=EOF)
     {
         cout<<"in while"<<endl;
         list<CElempunto*>::iterator It=pMap->bbdd.begin();
         int count=0;
        while  (((*It)->ID!=num)&&( count < pMap->bbdd.size()))
        {
            cout<<"."<<count;
            It++ ;
            count++;
            if (count>= pMap->bbdd.size()) break;
        }
        cout<<"fuera while"<<endl;

        if( count<pMap->bbdd.size()){
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
            ///add point
            unsigned char *key;
            key= new unsigned char [getFeatDim()];
            pMap->add_key(cvPoint(x,y),key,getFeatDim());
	    cout <<"ANADIDO PUNTO NUEVO!!!!!!!"<<endl;

        }
    }
    cout<<"end match"<<endl;
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
	int dumb;
         while (fscanf(f,"%d %d %d %d\n", &num, &y, &x, &dumb)!=EOF)
         {
            cout<<"x "<<x<<" y "<<y<<" id "<<num<<endl;
            cvmSet(*points,numptos,0,x) ;
            cvmSet(*points,numptos,1,y) ;
            cvmSet(*points,numptos,2,1000) ;
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
}
