#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <jni.h>
#include <time.h>

#include "../src/datacam.h"
#include "../src/modelcam.h"
#include "../src/map.h"
#include "../src/kalman.h"
#include "../src/updater.h"
#include "../src/tracker_surf.h"
#include "../src/tracker_harris.h"
#include "../src/freecam.h"
#include "../src/dataout.h"
#include "highgui.h"
#include "../src/particlefilter.h"
#include "../src/trackerfile.h"
#include "../src/xmlParser.h"



#define RHO_INIT_DIST 1

//#define DATA "F:\\SLAM\\Datos\\Vuelo28032008ArgandadelRey\\vuelo6\\original%0.4d.tif"
//#define DATA "c:\\datos\\slam\\kkk%0.4d.tif"
//#define DATA "F:\\SLAM\\Datos\\heli\\kkk%0.4d.tif"
#define DATA "/media/WOXTER/SLAM/Datos/heli/kkk%0.4d.tif"
//#define DATA "/media/WOXTER/SLAM/Datos/December62007-ArgandaDelRey/imagenes1/image%0.4d.jpg"
//#define DATA "/media/WOXTER/SLAM/Datos/Vuelo28032008ArgandadelRey/vuelo6/original%0.4d.tif"
#define LINUX
//	sprintf(filein,"G:\\SLAM\\Datos\\December62007-ArgandaDelRey\\imagenes1\\image%0.4d.jpg",iter);
//	sprintf(filein,"G:\\SLAM\\Datos\\renders\\escal%0.4d.jpg",iter);//G:\SLAM\Datos\renders
//      sprintf(filein,"c:\\datos\\slam\\kkk%0.4d.tif",iter);//G:\SLAM\Datos\renders
//	sprintf(filein,"/media/WOXTER/SLAM/Datos/heli/kkk%0.4d.tif",iter);
//	sprintf(filein,"/media/WOXTER/SLAM/Datos/December62007-ArgandaDelRey/imagenes1/image%0.4d.jpg",iter);
//	sprintf(filein,"/media/WOXTER/SLAM/Datos/univ-alberta-vision-sift/ualberta-csc-flr3-vision/image%0.4d.png",iter);

#define KALMAN

using namespace std;

CDataCam mDataCam;
CModelCam mModelCam;
CMap mMap;
CUpdater mUpdater;
//CTrackerFile mTracker;
//CTracker_surf mTracker;
CTracker *pTracker;
CFreeCam mVehicle;
#ifndef KALMAN
CParticleFilter mEstimator;
#else
CKalman mEstimator;
#endif
CDataOut mDataOut;

XMLNode xMainNode;

int ptox[1000][120];
int ptoy[1000][120];

///video variables
	CvCapture* capture = 0;
bool video = true ; ///< define si se usa un origen de video o de seq. de imagenes
char c;//donde se guarda las teclas pulsadas
//variables deonde se deja el frame y copia para dibujar en ella
IplImage* frame = 0;
IplImage* frameold = 0;
IplImage* framecopy=0;

FILE *gplot=0;

/// variable para contar la iteracion;
int iter=1;

float randomVector(float max, float min);
/** Variables interfaz grafica java **/
jobject startobj; ///< objeto start
jmethodID startcons, openMethod, setImage;///< variable clase start
JNIEnv* env;

using namespace std;
JNIEnv* create_vm() {
	JavaVM* jvm;
	JavaVMInitArgs args;
	JavaVMOption options[3];

	/* There is a new JNI_VERSION_1_4, but it doesn't add anything for the purposes of our example. */
	args.version = JNI_VERSION_1_6;
	args.nOptions = 2;
//	options[0].optionString = "-Djava.class.path=C:\\programas\\JavaApplication4\\dist\\JavaApplication4.jar;java.class.path=c:\\programas\\JavaApplication4\\build\\classes\\javaapplication4;\"c:\\Archivos de programa\\NetBeans 6.0.1\\platform7\\modules\\org-netbeans-api-visual.jar\";\"c:\\Archivos de programa\\NetBeans 6.0.1\\platform7\\lib\\org-openide-util.jar\"" ;
	options[0].optionString = "-Djava.class.path=/media/WOXTER/SLAM/Programas/JavaApplication4/build/classes/:/media/WOXTER/SLAM/Programas/JavaApplication4/build/classes/javaapplication4/:/opt/netbeans-6.1/platform8/modules/org-netbeans-api-visual.jar:/opt/netbeans-6.1/platform8/lib/org-openide-util.jar:/media/WOXTER/SLAM/Programas/JavaApplication4/dist/JavaApplication4.jar" ;

//options[1].optionString = "-Djava.library.path=C:\\Sun\\SDK\\jdk\\jre\\bin\\client";  /* set native library path */
    options[1].optionString = "-verbose:lll";                   /* print JNI-related messages */

    args.options = options;
    args.ignoreUnrecognized = JNI_FALSE;

    cout<<"jni_createvm="<<JNI_CreateJavaVM(&jvm, (void **)&env, &args)<<endl;
    return env;
}

void invoke_class() {
	jclass cls;
	jclass startcls;
	jmethodID mainMethod;
	jmethodID runMethod;
	jobjectArray applicationArgs;
	jstring applicationArg0;
cout<<"0"<<endl;
	applicationArgs = env->NewObjectArray( 1, env->FindClass( "java/lang/String"), NULL);
	applicationArg0 = env->NewStringUTF( "From-C-program");
	env->SetObjectArrayElement(applicationArgs, 0, applicationArg0);
	
cout<<"1 "<<env<<endl;
	cls = env->FindClass( "javaapplication4/Main");
	if (cls ==0){
	 if (env->ExceptionOccurred()) {
	       env->ExceptionDescribe();
	    }
	 exit(-1);
	}
cout<<"2 class "<<cls<<endl;
	startcls = env->FindClass( "javaapplication4/start");
	if (startcls ==0){
	 if (env->ExceptionOccurred()) {
	       env->ExceptionDescribe();
	    }
	 exit(-1);
	}
cout<<"2 startclass "<<startcls<<endl;

    jmethodID cons = env->GetMethodID(cls, "<init>", "()V");
cout<<"a"<<endl;
    jobject obj = env->NewObject(cls, cons);
cout<<"b"<<endl;
        mainMethod = env->GetStaticMethodID( cls, "main", "([Ljava/lang/String;)V");
     	runMethod = env->GetMethodID( cls, "run", "()V");
cout<<"c "<<mainMethod<<endl;
	if (runMethod==0) {
	    if (env->ExceptionOccurred()) {
	       env->ExceptionDescribe();
	    }
	    exit(-1);
	}

    startcons = env->GetMethodID(startcls, "<init>", "()V");
    startobj = env->NewObject(startcls, startcons);
    openMethod = env->GetMethodID( startcls, "open", "()V");
    setImage = env->GetMethodID(startcls,"setImage","(Ljava/lang/String;)V");

    cout<<"c "<<openMethod<<endl;
	if (openMethod==0) {
	    if (env->ExceptionOccurred()) {
	       env->ExceptionDescribe();
	    }
	    exit(-1);
	}
	if (setImage==0) {
	    if (env->ExceptionOccurred()) {
	       env->ExceptionDescribe();
	    }
	    exit(-1);
	}
cout<<"precall"<<endl;
	env->CallVoidMethod(startobj,openMethod);


	//env->CallStaticVoidMethod( cls, mainMethod, applicationArgs);
cout<<"callvoidmethod"<<endl;
	//env->CallVoidMethod( cls, runMethod);
cout<<"3 class "<<cls<<endl;

}

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
  
  mUpdater.setTracker((CTracker*)pTracker);
  mUpdater.setModelCam(&mModelCam);

  mDataOut.setDataCam(&mDataCam);
  mDataOut.setMap(&mMap);
  mDataOut.setTracker((CTracker*)pTracker);
  mDataOut.setModelCam(&mModelCam);
  #ifndef KALMAN
  mDataOut.setParticle(&mEstimator);
  #else
  mDataOut.setKalman(&mEstimator);//FIXME!!!!
  #endif

  pTracker->setDataCam(&mDataCam);

}
void param_init()
{
///Cámaras sony?
/*mDataCam.SetFx(629.49);
  mDataCam.SetFy(630.74);
  mDataCam.SetCx(337.8743788);
  mDataCam.SetCy(241.0742116);
  mDataCam.frame_width=640;
  mDataCam.frame_height=480;*/
///Camara firewire escuela
  mDataCam.SetFx(837.9529);
  mDataCam.SetFy(838.6889);
  mDataCam.SetCx(315.5322);
  mDataCam.SetCy(277.6592);
  mDataCam.frame_width=640;
  mDataCam.frame_height=480;
///Camara Genérica
  /*mDataCam.SetFx(1000);
    mDataCam.SetFy(1000);
    mDataCam.SetCx(320);
    mDataCam.SetCy(240);
    mDataCam.frame_width=640;
    mDataCam.frame_height=480;*/

  double dist[4];
  dist[0]=0;
  dist[1]=0;
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

  if (mVehicle.getMeasurementNum()==0){
    cvmSet(rotation,0,0,0);
    cvmSet(rotation,1,0,0);
    cvmSet(rotation,2,0,0);
    cvmSet(trans,0,0,0);
    cvmSet(trans,1,0,0);
    cvmSet(trans,2,0,0);
  }else{	
    CvMat *temp=mVehicle.getMeasurementVector();
    cvmSet(rotation,0,0,cvmGet(temp,3,0));
    cvmSet(rotation,1,0,cvmGet(temp,4,0));
    cvmSet(rotation,2,0,cvmGet(temp,5,0)); 
    cvmSet(trans,0,0,cvmGet(temp,0,0));
    cvmSet(trans,1,0,cvmGet(temp,1,0));
    cvmSet(trans,2,0,cvmGet(temp,2,0));
  }
  mDataCam.SetRotation(rotation);
  mDataCam.SetTranslation(trans);

  pTracker->setDataCam(&mDataCam);
  pTracker->setMap(&mMap);

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
  capture = cvCreateFileCapture( argv[1]);
  if (capture==0)
  {
   cout<< "no se puede abrir el fichero de video "<<endl;
   video = false ;
  }else
  {
    video = true; 
  }

  //abrir una ventana para visualizar
  cvNamedWindow( "CamShiftDemo", 1 );
  if (video == true ) {
     //obtencion del primer fram
     frame = cvQueryFrame( capture );
  }else {  
     char filein[400];
     sprintf(filein,DATA,iter);
     printf("%s\n",filein);     frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
  }
  frameold = cvCloneImage(frame);
  framecopy=cvCreateImage(cvGetSize(frame),8,3);//copia para poder tocar
  cvCopy( frame,framecopy);

}


void init_ptos()
{
  if (video == true ) {
     //obtencion del primer fram
     frame = cvQueryFrame( capture );
  }else {  
     char filein[400];
     sprintf(filein,DATA,iter);
     printf("%s\n",filein);     frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
  }

   cvCopy( frame,framecopy);

   mUpdater.Add(frame,frameold);

   //inicializacion de la posición de la camara con funciones de opencv
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
     (*It)->rho=1./RHO_INIT_DIST;
     (*It)->state=st_inited;
     mMap.inited++;
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
#ifdef LINUX 
    char gplotstr[300];
    if (gplot==0) gplot= popen("/usr/bin/gnuplot","w");
    sprintf(gplotstr,"splot 'disp%d.txt' u 1:2:3, 'cam.txt' u 1:2:3 w l \n",mDataOut.iter-1);
    fprintf(gplot,gplotstr);
    fflush(gplot);
#endif
   /* splot 'post' u 2:4:6 w l,'post' u 14:15:16, 'post' u 17:18:19, 'post' u 20:21:22, 'post' u 23:24:25, 'post' u 26:27:28, 'post' u 30:31:32, 'post' u 33:34:35, 'post' u 36:37:38, 'post' u 39:40:41*/
}
int main (int argc, char **argv)
{

//cvSetErrMode( CV_ErrModeSilent  );
xMainNode=XMLNode::openFileHelper("config.xml","slam");
  const char * t;
  t=xMainNode.getChildNode("tracker").getText();
if (!strcmp(t,"tracker_harris"))
{
   pTracker= new CTracker_harris();
}else if  (!strcmp(t,"tracker_surf"))
{
   pTracker= new CTracker_surf();
}else {
	cout<< "error no hay tracker"<<endl;
}
  cout<<"TRACKER IS : "<<t<<endl;

  t=xMainNode.getChildNode("gui").getText();
if (!strcmp(t,"true"))
{
    cout<< "hey"<<endl;
    env = create_vm();
    cout<< "adios env="<<env<<endl;
    invoke_class( );
    cout<<"invoca clases"<<endl;
}

  cout<<"TRACKER IS : "<<t<<endl;


conect();
cout<<"param_init"<<endl;
param_init();

cout<<"init_video"<<endl;
init_video( argc, argv);

cout<<"init_ptos"<<endl;
init_ptos();

int n=0;
int i;
char filein[400];
   
while(1)
{
    iter+=3;
    cout <<"ITERACION: "<< iter<<endl;
    //	frame = cvQueryFrame( capture );
    cvCopy(frame,frameold);
    if (video == true ) {
       //obtencion del primer fram
       frame = cvQueryFrame( capture );
    }else {  
       char filein[400];
       sprintf(filein,DATA,iter);
       printf("%s\n",filein);
	printf("precalvoid\n");
       env->CallVoidMethod(startobj,setImage,env->NewStringUTF(filein));	    if (env->ExceptionOccurred()) {
	       env->ExceptionDescribe();
	    }
	    exit(-1);

       frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
    } 

    //  compruebo que no se ha acabado el video
    if(frame==NULL) break;


    //   match frame
    mModelCam.ProjectPoints();
    pTracker->Match(frame);//entre los puntos de la imagen anterior y los de esta

    mUpdater.Add(frame,frameold);

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
   cout<<"waitkey correct"<<endl;
   c = cvWaitKey(0);//esto probablemente se pueda quitar
   if( c == 27 )//si presiono escape salgo del programa limpiamente
     break;
}
/// FIN
//////////////////////////////////

return 0;
}

