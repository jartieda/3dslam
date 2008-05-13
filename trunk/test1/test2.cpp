#include <stdio.h>
#include <iostream>
#include <fstream>

#include "../src/datacam.h"
#include "../src/modelcam.h"
#include "../src/map.h"
#include "../src/kalman.h"
#include "../src/updater.h"
#include "../src/tracker_surf.h"
#include "../src/freecam.h"
#include "../src/dataout.h"
#include "highgui.h"
#include "../src/particlefilter.h"
#include "../src/trackerfile.h"

#define DATA "F:\\SLAM\\Datos\\Vuelo28032008ArgandadelRey\\vuelo6\\original%0.4d.tif"
//#define DATA "c:\\datos\\slam\\kkk%0.4d.tif"
//#define DATA "F:\\SLAM\\Datos\\heli\\kkk%0.4d.tif"
//#define DATA "/media/WOXTER/SLAM/Datos/heli/kkk%0.4d.tif"
//#define DATA "/media/WOXTER/SLAM/Datos/December62007-ArgandaDelRey/imagenes1/image%0.4d.jpg"
//#define DATA "/media/WOXTER/SLAM/Datos/Vuelo28032008ArgandadelRey/vuelo6/original%0.4d.tif"
//	sprintf(filein,"G:\\SLAM\\Datos\\December62007-ArgandaDelRey\\imagenes1\\image%0.4d.jpg",iter);
//	sprintf(filein,"G:\\SLAM\\Datos\\renders\\escal%0.4d.jpg",iter);//G:\SLAM\Datos\renders
//  sprintf(filein,"c:\\datos\\slam\\kkk%0.4d.tif",iter);//G:\SLAM\Datos\renders
//	sprintf(filein,"/media/WOXTER/SLAM/Datos/heli/kkk%0.4d.tif",iter);
//	sprintf(filein,"/media/WOXTER/SLAM/Datos/December62007-ArgandaDelRey/imagenes1/image%0.4d.jpg",iter);
//	sprintf(filein,"/media/WOXTER/SLAM/Datos/univ-alberta-vision-sift/ualberta-csc-flr3-vision/image%0.4d.png",iter);

#define KALMAN

using namespace std;

CDataCam mDataCam;
CModelCam mModelCam;
CMap mMap;
CUpdater mUpdater;
CTrackerFile mTracker;
//CTracker_surf mTracker;
CFreeCam mVehicle;
#ifndef KALMAN
CParticleFilter mEstimator;
#else
CKalman mEstimator;
#endif
CDataOut mDataOut;


int ptox[1000][120];
int ptoy[1000][120];

///video variables
	CvCapture* capture = 0;

char c;//donde se guarda las teclas pulsadas
//variables deonde se deja el frame y copia para dibujar en ella
IplImage* frame = 0;
IplImage* frameold = 0;
IplImage* framecopy=0;

/// variable para contar la iteracion;
int iter=1;

float randomVector(float max, float min);


void conect()
{

  mModelCam.setDataCam(&mDataCam);
  mModelCam.setMap(&mMap);

  mEstimator.setModelCam(&mModelCam);
  mEstimator.setDataCam(&mDataCam);
  mEstimator.setMap(&mMap);
  mEstimator.setModel((CModel*)(&mVehicle));

  mUpdater.setDataCam(&mDataCam);
  mUpdater.setMap(&mMap);
  mUpdater.setTracker((CTracker*)&mTracker);
  mUpdater.setModelCam(&mModelCam);

  mDataOut.setDataCam(&mDataCam);
  mDataOut.setMap(&mMap);
  mDataOut.setTracker((CTracker*)&mTracker);
  mDataOut.setModelCam(&mModelCam);
  #ifndef KALMAN
  mDataOut.setParticle(&mEstimator);
  #else
  mDataOut.setKalman(&mEstimator);//FIXME!!!!
  #endif

  mTracker.setDataCam(&mDataCam);

}
void param_init()
{
  mDataCam.SetFx(629.49);
  mDataCam.SetFy(630.74);
  mDataCam.SetCx(337.8743788);
  mDataCam.SetCy(241.0742116);
  mDataCam.frame_width=640;
  mDataCam.frame_height=480;

  /*mDataCam.SetFx(1000);
    mDataCam.SetFy(1000);
    mDataCam.SetCx(320);
    mDataCam.SetCy(240);
    mDataCam.frame_width=640;
    mDataCam.frame_height=480;*/

  double dist[4];
  dist[0]=-0.1928;
  dist[1]=0.1373;
  dist[2]=0.;
  dist[3]=0.;

  CvMat *distMat;
  distMat = cvCreateMat(4,1,CV_32FC1);

  for ( int i=0;i<4;i++) cvmSet(distMat,i,0,dist[i]);

  cvCopy(distMat,mDataCam.distortion);

  CvMat *rotation;
  CvMat *trans ,*trans2;
  rotation=cvCreateMat(3,1,CV_32FC1);
  trans=cvCreateMat(3,1,CV_32FC1);
  trans2=cvCreateMat(3,1,CV_32FC1);

  CvMat *temp=mVehicle.getMeasurementVector();

  cvmSet(rotation,0,0,cvmGet(temp,3,0));
  cvmSet(rotation,1,0,cvmGet(temp,4,0));
  cvmSet(rotation,2,0,cvmGet(temp,5,0));
  mDataCam.SetRotation(rotation);

  cvmSet(trans,0,0,cvmGet(temp,0,0));
  cvmSet(trans,1,0,cvmGet(temp,1,0));
 // cvmSet(trans,2,0,cvmGet(temp,2,0));//-(-911));
  cvmSet(trans,2,0,-20);

  mDataCam.SetTranslation(trans);

  mTracker.setDataCam(&mDataCam);
  mTracker.setMap(&mMap);

}

void init_video(int argc, char **argv)
{
  //Arbrir video para captura
  if (argc < 2)
    {
      cout<<"uso commando <fichero de entrada> [fichero de datos]"<<endl;
   //   exit(0);
    }

  cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, 250000 );
  //capture = cvCreateFileCapture(  "/media/WOXTER/airview/valdelaguna/vuelo-1/crop120sec.mpg");
  /*   capture = cvCreateFileCapture( argv[1]);
       if (capture==0)
       {
       cout<< "no se puede abrir el fichero de video "<<endl;
       exit(0);
       }
  */
  //abrir una ventana para visualizar
  cvNamedWindow( "CamShiftDemo", 1 );
  //obtencion del primer fram
  //frame = cvQueryFrame( capture );
  //iter++;
  char filein[400];
  sprintf(filein,DATA,iter);
  printf("%s\n",filein);

  frameold=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
  frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
  framecopy=cvCreateImage(cvGetSize(frame),8,3);//copia para poder tocar
  cvCopy( frame,framecopy);

}


void init_ptos()
{
   char filein[400];
   sprintf(filein,DATA,iter);
   printf("%s\n",filein);
   frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
   cvCopy( frame,framecopy);

   mUpdater.Add(frame,frameold);

   CvMat *rotation;
   CvMat *trans;
   rotation=cvCreateMat(3,1,CV_32FC1);
   trans=cvCreateMat(3,1,CV_32FC1);
   //inicializacion de la posición de la camara con funciones de opencv
   CvMat *image_points;
   CvMat *object_points;
   image_points=cvCreateMat(2,mMap.visible,CV_32FC1);
   object_points=cvCreateMat(3,mMap.visible,CV_32FC1);
   int i =0;
   cout <<"marcas visibles en init "<<mMap.visible<<endl;

   CvMat *h;
   h=cvCreateMat(3,1,CV_32FC1);

   for   (list<CElempunto*>::iterator It=mMap.bbdd.begin();It != mMap.bbdd.end();It++)
   {
     mModelCam.cvInverseParam(&h,(*It)->pto);
     (*It)->wx=cvmGet(mDataCam.translation,0,0);
     (*It)->wy=cvmGet(mDataCam.translation,1,0);
     (*It)->wz=cvmGet(mDataCam.translation,2,0);
     (*It)->theta=atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0)));
     (*It)->phi=atan2(cvmGet(h,0,0),cvmGet(h,2,0));
     (*It)->rho=1./0.2;
     (*It)->state=st_inited;
     mMap.inited++;
     cvmSet(object_points,0,i,(*It)->wx);
     cvmSet(object_points,1,i,(*It)->wy);
     cvmSet(object_points,2,i,(*It)->wz);
     cvmSet(image_points,0,i,(*It)->pto.x);
     cvmSet(image_points,1,i,(*It)->pto.y);
     i++;
     cout<<i<<endl;
   }

   cout<<"kalman ini"<<endl;
   mEstimator.initState();
   cout<<"kalman inited"<<endl;

}
void data_out()
{

  //dibjo puntos encontrados
  cvCopy( frame,framecopy);
  mDataOut.Draw(framecopy);
  #ifndef KALMAN
   mDataOut.Particle(framecopy);
  #else
  mDataOut.Disp_out(framecopy);
  #endif

  mDataOut.Feat();
  mDataOut.Frame();
  mDataOut.Cam();

   char nombre[100];
   //muestro imagen y compruebo salida
   sprintf(nombre,"frame%.2d.png",iter);
   cvSaveImage(nombre ,framecopy);
   cvShowImage( "CamShiftDemo", framecopy );

   /* splot 'post' u 2:4:6 w l,'post' u 14:15:16, 'post' u 17:18:19, 'post' u 20:21:22, 'post' u 23:24:25, 'post' u 26:27:28, 'post' u 30:31:32, 'post' u 33:34:35, 'post' u 36:37:38, 'post' u 39:40:41*/
}
int main (int argc, char **argv)
{

//cvSetErrMode( CV_ErrModeSilent  );


conect();
	cout<<"param_init"<<endl;
param_init();
	cout<<"init_video"<<endl;
init_video( argc, argv);
	cout<<"init_ptos"<<endl;

init_ptos();
///test 1
//cvSetErrMode(2);
int n=0;
int i;
//iter=8;

while(1)
{
iter+=3;
cout <<"ITERACION: "<< iter<<endl;
//	frame = cvQueryFrame( capture );
	char filein[400];
sprintf(filein,DATA,iter);
        cvCopy(frame,frameold);
    frame=cvLoadImage(filein, CV_LOAD_IMAGE_COLOR );

//  compruebo que no se ha acabado el video
  if(frame==NULL) break;


//   match frame
    mModelCam.ProjectPoints();
    mTracker.Match(frame);//entre los puntos de la imagen anterior y los de esta

    mUpdater.Add(frame,frameold);
  //mUpdater.AddByHand(frame);
   //   ADD
//esto anadiría puntos ya anadidos....

   i=0;

   n++;

   mUpdater.remove();

   mEstimator.UpdateMatrixSize();

   mEstimator.Predict();
   mModelCam.ProjectPoints();

//   mEstimator.Test();
   cout<<"ransac"<<endl;
//   mUpdater.TestRANSAC();

   mEstimator.UpdateMatrixSize();

   mEstimator.Correct();
   mModelCam.ProjectPoints();

#ifdef KALMAN
   mEstimator.Print(iter);
#endif
   cout<<"update"<<endl;
   mUpdater.update();
   cout<<"dataout"<<endl;

   data_out();
   cout<<"waitkey"<<endl;
   c = cvWaitKey(0);//esto probablemente se pueda quitar
   if( c == 27 )//si presiono escape salgo del programa limpiamente
     break;
}
/// FIN
//////////////////////////////////

return 0;
}

