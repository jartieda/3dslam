#include "tracker_harris.h"
namespace SLAM{
    using namespace SLAM;
using namespace std;

CTracker_harris::CTracker_harris()
{
   int ancho, alto;
   zonabusca=cvCreateImage( cvSize( 100, 100), 8, 1 );
   ancho= cvGetSize(zonabusca).width-10 +1; ///FIXME EL 10 es el numero de pixels del patron
   alto = cvGetSize(zonabusca).height-10+1;
   corr = cvCreateImage( cvSize( ancho, alto), 32, 1 );
}

CTracker_harris::~CTracker_harris()
{

}
/**
 * sigue los puntos del mapa usando el metodo de la correlaci&oacute;n <br>
 * Se buscan los puntos alrededor de una zona de 100x100 de la última posición vista del punto.
 * El punto se busca usando este método en la imagen de gris y en una imagen de bordes. El punto se actualiza si la diferncia entre ambos
 * m&eacute;todos es menor que 40 <br>
 * Si el punto esta inicializado se marca como no view <br>
 * FIXME si no esta inicializado no se hace nada
 * FIXME hay que buscar el punto alrededor de la zona proyectada por el kalman
**/
void CTracker_harris::Match(IplImage *f)
{
   IplImage *grey;
   grey=cvCreateImage( cvGetSize(f), 8, 1 );
   CvPoint temp;
   cvCvtColor(f,grey,CV_RGB2GRAY);

   IplImage *patron,*patron_border;
   patron = cvCreateImageHeader( cvSize(10,10),8,1);
   patron_border = cvCreateImage( cvSize(10,10),8,1);

   for ( std::list<CElempunto*>::iterator It=pMap->bbdd.begin();
         It != pMap->bbdd.end(); It++ )
   {
        if((*It)->state!=st_empty){
	patron->imageData=(char*)(*It)->key;

        cvGetRectSubPix( grey, zonabusca,cvPointTo32f((*It)->pto) );
        //cvMatchTemplate( zonabusca, (*It)->patron,corr, CV_TM_CCORR_NORMED );
	cvMatchTemplate( zonabusca, patron,corr, CV_TM_CCORR_NORMED );
        cvMinMaxLoc( corr, &minVal, &maxVal, &minLoc, &maxLoc, 0 );
        //(*It)->maxVal=maxVal;//FIXME Usar como indicador de calidad
        temp.x=maxLoc.x+(*It)->pto.x-(cvGetSize(corr).width/2);
        temp.y=maxLoc.y+(*It)->pto.y-(cvGetSize(corr).height/2);

	cvSobel(zonabusca,zonabusca,1,1,3);
	cvSobel(patron,patron_border,1,1,3);

        cvMatchTemplate( zonabusca, patron_border,corr, CV_TM_CCORR_NORMED );
        cvMinMaxLoc( corr, &minVal, &maxVal, &minLoc, &maxLoc, 0 );

	double dx=temp.x-(maxLoc.x+(*It)->pto.x-cvGetSize(corr).width/2);
	double dy=temp.y-(maxLoc.y+(*It)->pto.y-cvGetSize(corr).height/2);
	cout<<"dx: "<<dx <<" dy: "<<dy<<endl;
	cout<<dx<<" "<<temp.x<<" "<<maxLoc.x+(*It)->pto.x-cvGetSize(corr).width/2<<" "<<(*It)->pto.x<<endl;
	cout<<dy<<" "<<temp.y<<" "<<maxLoc.y+(*It)->pto.y-cvGetSize(corr).height/2<<" "<<(*It)->pto.y<<endl;

//	if((*It)->maxVal<0.5)
	if (sqrt(dx*dx+dy*dy)>40||temp.x<0 ||temp.y<0)
	{
	//	cout<<"no match "<<sqrt(dx*dx+dy*dy)<<" "<<(*It)->maxVal<<" "<<maxVal<<endl;
	        if((*It)->state==st_inited)
         	{
         		std::cout << "borropuntos: "<<(*It)->pto.x<<" "<<(*It)->pto.y<<std::endl;
         		(*It)->state = st_no_view;
         		pMap->visible--;
         	}else if((*It)->state!=st_no_view)
         	{
         		cout<< "borro punto por no iniciado y fuera devista " << endl;
//         		pMap->bbdd.erase(It);
//	      		(*It)->state = st_empty;
//         		pMap->visible--;
		}
	///FIXME En el casod de que el punto se deje de ver antes de inicializar quiero borrarlo.
	}
	else
	{
	//	cout<<"si match "<<sqrt(dx*dx+dy*dy)<<" "<<(*It)->maxVal<<" "<<maxVal<<endl;
		(*It)->old_pto.x=(*It)->pto.x;
		(*It)->old_pto.y=(*It)->pto.y;
		(*It)->pto.x=temp.x;
		(*It)->pto.y=temp.y;
		if((*It)->state==st_no_view)
		{
			(*It)->state=st_inited;
		}
	}
      } //if !empty
   }//end for
   cvReleaseImage(&grey);
   cvReleaseImage(&patron_border);
}

void CTracker_harris::Init(IplImage *f,CvMat **keys,CvMat **points)
{
   int cornercount;
   cornercount=30;
   //cvCvtColor(f,grey,CV_RGB2GRAY);
   CvPoint2D32f corners[100];
   IplImage *grey, *eig_image, *temp, *patron;
   grey=cvCreateImage( cvGetSize(f), 8, 1 );
   patron=cvCreateImage( cvSize(10,10), 8, 1 );
   eig_image = cvCreateImage( cvGetSize(f), 32, 1 );
   temp=cvCreateImage(cvGetSize(f),32,1);
   cvCvtColor(f,grey,CV_RGB2GRAY);

   CvPoint pts[30];

   cvGoodFeaturesToTrack(grey,eig_image,temp,corners,&cornercount,0.1, 40, NULL);

    *keys = cvCreateMat(cornercount,128,CV_32FC1) ;
    *points = cvCreateMat(cornercount,4,CV_32FC1) ;
	cout<<"esquinas encontradas"<<cornercount<<endl;
   for (int i=0;i<cornercount;i++){
      pts[i]=cvPointFrom32f( corners[i]);
	  cvGetRectSubPix( grey, patron,corners[i] );//
	  for (int j=0;j<100;j++)
	  {
		  cvmSet(*keys,i,j,*(patron->imageData+j));
	  }
	  cvmSet(*points,i,0,pts[i].x);
	  cvmSet(*points,i,1,pts[i].y);
	  cvmSet(*points,i,2,10);
	  cvmSet(*points,i,3,0);
   }

   cvReleaseImage(&grey);
   cvReleaseImage(&eig_image);
   cvReleaseImage(&temp);

}

/** Obtiene el descriptor asociado a un punto con una escala detrminada **/
void  CTracker_harris::Descriptor(IplImage *img, CvPoint *point,int s, int *key)
{
IplImage *grey,*patron;
   grey=cvCreateImage( cvGetSize(img), 8, 1 );
   cvCvtColor(img,grey,CV_RGB2GRAY);
   patron=cvCreateImage( cvSize(10,10), 8, 1 );
	CvScalar a;
	cvGetRectSubPix( grey, patron,cvPoint2D32f(point->x,point->y) );//
   for (int i = 0 ; i<10; i++)
	for ( int j = 0; j<10; j++)
	{
		a= cvGet2D( patron, i , j);
	  	key[i*10+j]= a.val[0];
	}

}
/** Devuelve el tamaño en bytes del vector de características **/
int  CTracker_harris::getFeatDim()
{
	return 100; //10 x 10
}

int CTracker_harris::Init(IplImage *img,int **keys,CvMat **points)
{

}
}
