#include "tracker_surf.h"

using namespace std;
namespace SLAM{
CTracker_surf::CTracker_surf()
{
   storage = cvCreateMemStorage(0);
   feat= cvCreateSeq( CV_32FC3, /* sequence of integer elements */
                          sizeof(CvSeq), /* header size - no extra fields */
                          sizeof(CvPoint3D32f), /* element size */
                          storage /* the container storage */ );
   levels=6;
}

CTracker_surf::~CTracker_surf()
{

}

 int CTracker_surf::getFeatDim()
 {
     return 64;
 }
void CTracker_surf::Match(IplImage *img)
{
cout<<"entro en el match"<<endl;
IplImage *grey =cvCreateImage(cvGetSize(img),8,1);
cvCvtColor(img,grey,CV_RGB2GRAY);

IplImage *grey2 =cvCreateImage(cvGetSize(img),8,1);
cvCvtColor(img,grey2,CV_RGB2GRAY);

surf.img = grey;
cvClearSeq( feat );
surf.find_features( grey,feat, levels);

float ang;
int *desc;
int *samples1;
samples1=new int[64*feat->total];
CvMat *new_points = cvCreateMat(feat->total,4,CV_32FC1) ;
for( int i = 0; i < feat->total; i++ )
    {
         CvPoint3D32f point = *(CvPoint3D32f*)cvGetSeqElem( feat, i );
         ang=surf.orientation((int)point.x,(int)point.y,(int)point.z);
         desc=surf.descriptor((int)point.x,(int)point.y,(int)point.z,ang);
         cvCircle(grey2 , cvPoint((int)point.x,(int)point.y), (int)(point.z*3.0), cvScalar(255));
         cvLine(grey2,cvPoint((int)point.x,(int)point.y),
			cvPoint((int)(point.x+point.z*3.0*sin(ang)),
				(int)(point.y+point.z*3.0*cos(ang))),cvScalar(255));

         cvmSet(new_points,i,0,point.x) ;
         cvmSet(new_points,i,1,point.y) ;
         cvmSet(new_points,i,2,point.z) ;
         cvmSet(new_points,i,3,ang) ;
	 for (int di=0; di<64; di++){
	      samples1[i*64+di]=(unsigned char )(desc[di]+128);
	 }
         delete desc;
    }
  int key[64];
	for ( list<CElempunto*>::iterator It=pEstimator->pMap->bbdd.begin();
         	It != pEstimator->pMap->bbdd.end(); It++ )
	{///FIXME Esto tiene que estar relacionado con un valor mas claro del surf
	     if((*It)->projx>(9*levels+1) && (*It)->projx<img->width-(9*levels+1) &&
                (*It)->projy>(9*levels+1) && (*It)->projy<img->height-(9*levels+1))
	     {
                 for (int d=0; d<64;d++)
                 {
                     key[d]=(*It)->key[d];
                 }

	         int n = surf.nearest_neighbor_2(key,samples1,feat->total,
					   cvPoint((int)(*It)->projx,(int)(*It)->projy),
					   new_points,50);

	         if(n==-1)//en caso de no encorar fallo
	         {
                 //cout<<"no match "<<endl;
	             if((*It)->state==st_inited)
         	     {
             		std::cout << "borropuntos: "<<(*It)->pto.x<<" "<<(*It)->pto.y<<std::endl;
             		(*It)->state = st_no_view;
             		pEstimator->pMap->visible--;
         	     }else if((*It)->state!=st_no_view)
         	     {
         		cout<< "borro punto por no iniciado y fuera devista " << endl;
         	//	pEstimator->pMap->bbdd.erase(It);
         	//	pEstimator->pMap->visible--;
	 	     }
	 	     (*It)->pto.x=0;
  		     (*It)->pto.y=0;
	         }
	         else//encaso de exito
	         {
                   (*It)->old_pto.x=(*It)->pto.x;
                   (*It)->old_pto.y=(*It)->pto.y;
                   (*It)->pto.x=(int)cvmGet(new_points,n,0);
	           (*It)->pto.y=(int)cvmGet(new_points,n,1);
		   if((*It)->state==st_no_view)
		   {
			(*It)->state=st_inited;
			pEstimator->pMap->visible++;
		   }
	         }
            }else
            {
              (*It)->state=st_no_view;
              pEstimator->pMap->visible--;
            } //end if proj dentro borde
	}//end for cada elemento del mapa
	cvShowImage("win",grey2);
	cvWaitKey(100);

cvReleaseImage(&grey);
cvReleaseImage(&grey2);
delete samples1;
cvReleaseMat(&new_points);
}

int CTracker_surf::Init(IplImage *img,int **keys,CvMat **points)
{

IplImage *grey =cvCreateImage(cvGetSize(img),8,1);

cvCvtColor(img,grey,CV_RGB2GRAY);
IplImage *grey2 =cvCreateImage(cvGetSize(img),8,1);

cvCvtColor(img,grey2,CV_RGB2GRAY);
surf.img=grey;
cvClearSeq( feat );

surf.find_features( grey,feat, levels);

*keys = new int[64*feat->total];// cvCreateMat(feat->total,64,CV_32FC1) ;
*points = cvCreateMat(feat->total,4,CV_32FC1) ;
cout<<"surf: "<< "feat_total: "<<feat->total<<endl;
cout<<"end2"<<endl;
float ang;
int *desc;

for( int i = 0; i < feat->total; i++ )
    {
        CvPoint3D32f point = *(CvPoint3D32f*)cvGetSeqElem( feat, i );
        if((point.x>(9*levels+1)) && (point.y >(9*levels+1)) &&
           (point.x<img->width-(9*levels+1)) && (point.y < img->height-(9*levels+1)))
        {
        ang=surf.orientation((int)point.x,(int)point.y,(int)point.z);
        desc=surf.descriptor((int)point.x,(int)point.y,(int)point.z,ang);
        cvCircle(grey2 , cvPoint((int)point.x,(int)point.y), (int)(point.z*3.0), cvScalar(255));
        cvLine(grey2,cvPoint((int)point.x,(int)point.y),cvPoint((int)(point.x+point.z*3.0*sin(ang)),(int)(point.y+point.z*3.0*cos(ang))),cvScalar(255));

       	cvmSet(*points,i,0,point.x) ;
       	cvmSet(*points,i,1,point.y) ;
       	cvmSet(*points,i,2,point.z) ;
        cvmSet(*points,i,3,ang) ;
        //cout<<"desc: ";
        for (int di=0; di<64; di++){
           //cout<<" "<<desc[di];
	   //cvmSet(*keys,i,di, desc[di]+128) ;
	   (*keys)[i*64+di]=desc[di]+128;
        }
       	//cout<<endl;
        delete desc;
        }
    }
    cvShowImage("win",grey2);
    cvWaitKey(100);
    cvReleaseImage(&grey);
    cvReleaseImage(&grey2);
return feat->total;//FIXME ESTO ES ASÍ PORQUE LA MATRIZ QUE CREO TIENE ESTAS DIMENSIONES PERO NO ESTA RELLENA

}

void CTracker_surf::Descriptor(IplImage *img, CvPoint *point,int s, int *key)
{
     IplImage *grey =cvCreateImage(cvGetSize(img),8,1);
     cvCvtColor(img,grey,CV_RGB2GRAY);
     surf.img=grey;
     float ang;
     int *desc;
     ang=surf.orientation((int)point->x,(int)point->y,s);
     desc=surf.descriptor((int)point->x,(int)point->y,s,ang);
     for (int di=0; di<64; di++){
	   key[di]=desc[di]+128;
     }
        delete desc;
     cvReleaseImage(&grey);
}
}
