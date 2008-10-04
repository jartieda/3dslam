/****************************************************************************/
/*! \mainpage 3D SLAM Project
 * \section intro_sec Introduction
 *
 * This project is an implementation of visual simultaneous location and mapping

 * Copyright (c) 2008, Jorge Artieda Trigueros
 * All rights reserved.
 *
 * @version     V1.0
 * @author      Jorge Atieda Trigueros
 *
 * \section tutorial Turorial
 * <a href="../../slam.html">To be created...</a>
 ****************************************************************************/

//#define JAVA_GUI

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

#ifdef JAVA_GUI
    #include <jni.h>
#endif

#include <time.h>

#include "highgui.h"

#include "../src/datacam.h"
#include "../src/modelcam.h"
#include "../src/map.h"
#include "../src/kalman.h"
#include "../src/updater.h"
#include "../src/tracker_surf.h"
#include "../src/tracker_harris.h"
#include "../src/freecam.h"
#include "../src/dataout.h"
#include "../src/particlefilter.h"
#include "../src/trackerfile.h"
#include "../src/xmlParser.h"
#include "../src/robotxyth.h"

#define RHO_INIT_DIST 1

//#define DATA "F:\\SLAM\\Datos\\Vuelo28032008ArgandadelRey\\vuelo6\\original%0.4d.tif"
//#define DATA "c:\\datos\\slam\\kkk%0.4d.tif"
//#define DATA "F:\\SLAM\\Datos\\heli\\kkk%0.4d.tif"
//#define DATA "/media/WOXTER/SLAM/Datos/heli/kkk%0.4d.tif"
//#define DATA "/media/WOXTER/SLAM/Datos/December62007-ArgandaDelRey/imagenes1/image%0.4d.jpg"
//#define DATA "/media/WOXTER/SLAM/Datos/Vuelo28032008ArgandadelRey/vuelo6/original%0.4d.tif"
#define LINUX
// sprintf(filein,"G:\\SLAM\\Datos\\December62007-ArgandaDelRey\\imagenes1\\image%0.4d.jpg",iter);
// sprintf(filein,"G:\\SLAM\\Datos\\renders\\escal%0.4d.jpg",iter);//G:\SLAM\Datos\renders
//      sprintf(filein,"c:\\datos\\slam\\kkk%0.4d.tif",iter);//G:\SLAM\Datos\renders
// sprintf(filein,"/media/WOXTER/SLAM/Datos/heli/kkk%0.4d.tif",iter);
// sprintf(filein,"/media/WOXTER/SLAM/Datos/December62007-ArgandaDelRey/imagenes1/image%0.4d.jpg",iter);
// sprintf(filein,"/media/WOXTER/SLAM/Datos/univ-alberta-vision-sift/ualberta-csc-flr3-vision/image%0.4d.png",iter);

#define KALMAN

using namespace std;
using namespace SLAM;

//CDataCam *pDataCam;
//CMap *pMap;

CModelCam mModelCam;
CUpdater mUpdater;
//CTrackerFile mTracker;
//CTracker_surf mTracker;
CTracker *pTracker;
CModel *pVehicle;

#ifndef KALMAN
CParticleFilter mEstimator;
#else
CKalman mEstimator;
#endif

CDataOut *pDataOut;



XMLNode xMainNode;

/*int ptox[1000][120];
int ptoy[1000][120];*/

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
int iter=120;

float randomVector(float max, float min);

using namespace std;

const char *DATA;

#ifdef JAVA_GUI
/** Variables interfaz grafica java **/

jobject startobj; ///< objeto start
jmethodID startcons, openMethod, setImage;///< variable clase start
JNIEnv* env;

JNIEnv* create_vm()
{
///*    JavaVM* jvm;
//    JavaVMInitArgs args;
//    JavaVMOption options[3];
//**/
//    /* There is a new JNI_VERSION_1_4, but it doesn't add anything for the purposes of our example. */
//  /*  args.version = JNI_VERSION_1_6;
//    args.nOptions = 2;
//     options[0].optionString = "-Djava.class.path=D:\\SLAM\\Programas\\JavaApplication4\\build\\classes;d:\\slam\\programas\\JavaApplication4\\build\\classes\\javaapplication4;\"c:\\Archivos de programa\\NetBeans 6.0.1\\platform7\\modules\\org-netbeans-api-visual.jar\";\"c:\\Archivos de programa\\NetBeans 6.0.1\\platform7\\lib\\org-openide-util.jar\"" ;
//*/
////    options[0].optionString = "-Djava.class.path=/media/WOXTER/SLAM/Programas/JavaApplication4/build/classes/:/media/WOXTER/SLAM/Programas/JavaApplication4/build/classes/javaapplication4/:/opt/netbeans-6.1/platform8/modules/org-netbeans-api-visual.jar:/opt/netbeans-6.1/platform8/lib/org-openide-util.jar:/media/WOXTER/SLAM/Programas/JavaApplication4/dist/JavaApplication4.jar" ;
//
//    //options[1].optionString = "-Djava.library.path=C:\\Sun\\SDK\\jdk\\jre\\bin\\client";  /* set native library path */
//    options[1].optionString = "-verbose:lll";                   /* print JNI-related messages */
//
//    args.options = options;
//    args.ignoreUnrecognized = JNI_TRUE;


JavaVM *jvm;       /* denotes a Java VM */
 //   JNIEnv *env;       /* pointer to native method interface */
    JavaVMInitArgs vm_args; /* JDK/JRE 6 VM initialization arguments */
    JavaVMOption* options = new JavaVMOption[2];
    options[0].optionString = "-verboses:jni";
    options[1].optionString = "-Djava.class.path=\"C:\\archivos de programa\\java\\jdk1.6.0_05\"";
    vm_args.version = JNI_VERSION_1_4;
    vm_args.nOptions = 2;
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false;
    /* load and initialize a Java VM, return a JNI interface
     * pointer in env */
    cout<<"jni_createvm="<<JNI_CreateJavaVM(&jvm, (void**)&env, &vm_args)<<endl;
cout<<"hola"<<endl;

    //cout<<"jni_createvm="<<JNI_CreateJavaVM(&jvm, (void **)&env, &args)<<endl;
    return env;
}


void invoke_class()
{
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
    if (cls ==0)
    {
        if (env->ExceptionOccurred())
        {
            env->ExceptionDescribe();
        }
        exit(-1);
    }
    cout<<"2 class "<<cls<<endl;
    startcls = env->FindClass( "javaapplication4/start");
    if (startcls ==0)
    {
        if (env->ExceptionOccurred())
        {
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
    if (runMethod==0)
    {
        if (env->ExceptionOccurred())
        {
            env->ExceptionDescribe();
        }
        exit(-1);
    }

    startcons = env->GetMethodID(startcls, "<init>", "()V");
    startobj = env->NewObject(startcls, startcons);
    openMethod = env->GetMethodID( startcls, "open", "()V");
    setImage = env->GetMethodID(startcls,"setImage","(Ljava/lang/String;)V");

    cout<<"c "<<openMethod<<endl;
    if (openMethod==0)
    {
        if (env->ExceptionOccurred())
        {
            env->ExceptionDescribe();
        }
        exit(-1);
    }
    if (setImage==0)
    {
        if (env->ExceptionOccurred())
        {
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
#endif

void conect()
{
    mEstimator.setModelCam(&mModelCam);
    //mEstimator.setDataCam(mEstimator.pDataCam);
    //mEstimator.setMap(pMap);
    mEstimator.setModel((CModel*)(pVehicle));

    //mUpdater.setDataCam(mEstimator.pDataCam);
    //mUpdater.setMap(pMap);
    mUpdater.setTracker((CTracker*)pTracker);
    mUpdater.setModelCam(&mModelCam);
    mUpdater.pEstimator=&mEstimator;

//    pDataOut->setDataCam(mEstimator.pDataCam);
//    pDataOut->setMap(pMap);
    pDataOut->setTracker((CTracker*)pTracker);
    pDataOut->setModelCam(&mModelCam);
    pDataOut->pEstimator=&mEstimator;
#ifndef KALMAN
    pDataOut->setParticle(&mEstimator);
#else
//    pDataOut->setKalman(&mEstimator);//FIXME!!!!
#endif
      mModelCam.pEstimator=&mEstimator;
      pTracker->pEstimator=&mEstimator;
//    pTracker->setDataCam(mEstimator.pDataCam);

}
void param_init()
{
    ///Cámaras sony?
    /*mEstimator.pDataCam->SetFx(629.49);
      mEstimator.pDataCam->SetFy(630.74);
      mEstimator.pDataCam->SetCx(337.8743788);
      mEstimator.pDataCam->SetCy(241.0742116);
      mEstimator.pDataCam->frame_width=640;
      mEstimator.pDataCam->frame_height=480;*/
    ///Camara firewire escuela
    mEstimator.pDataCam->SetFx(837.9529);
    mEstimator.pDataCam->SetFy(838.6889);
    mEstimator.pDataCam->SetCx(315.5322);
    mEstimator.pDataCam->SetCy(277.6592);
    mEstimator.pDataCam->frame_width=640;
    mEstimator.pDataCam->frame_height=480;
    ///Camara Genérica
    /*mEstimator.pDataCam->SetFx(1000);
      mEstimator.pDataCam->SetFy(1000);
      mEstimator.pDataCam->SetCx(320);
      mEstimator.pDataCam->SetCy(240);
      mEstimator.pDataCam->frame_width=640;
      mEstimator.pDataCam->frame_height=480;*/

    double dist[4];
    dist[0]=0;
    dist[1]=0;
    dist[2]=0.;
    dist[3]=0.;

    CvMat *distMat;
    distMat = cvCreateMat(4,1,CV_32FC1);

    for ( int i=0;i<4;i++)
        cvmSet(distMat,i,0,dist[i]);

    cvCopy(distMat,mEstimator.pDataCam->distortion);

    CvMat *rotation;
    CvMat *trans ,*trans2;
    rotation=cvCreateMat(3,1,CV_32FC1);
    trans=cvCreateMat(3,1,CV_32FC1);
    trans2=cvCreateMat(3,1,CV_32FC1);

    if (pVehicle->getMeasurementNum()==0)
    {
        cvmSet(rotation,0,0,0);
        cvmSet(rotation,1,0,0);
        cvmSet(rotation,2,0,0);
        cvmSet(trans,0,0,0);
        cvmSet(trans,1,0,0);
        cvmSet(trans,2,0,0);
    }
    else
    {
        CvMat *temp=pVehicle->getMeasurementVector();
        cvmSet(rotation,0,0,cvmGet(temp,3,0));
        cvmSet(rotation,1,0,cvmGet(temp,4,0));
        cvmSet(rotation,2,0,cvmGet(temp,5,0));
        cvmSet(trans,0,0,cvmGet(temp,0,0));
        cvmSet(trans,1,0,cvmGet(temp,1,0));
        cvmSet(trans,2,0,cvmGet(temp,2,0));
    }
    mEstimator.pDataCam->SetRotation(rotation);
    mEstimator.pDataCam->SetTranslation(trans);

//    pTracker->setDataCam(mEstimator.pDataCam);
//    pTracker->setMap(pMap);
//    pTracker->setMapMnger(pMapMnger);
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
        capture = cvCreateFileCapture( DATA);
        if (capture ==0){
        cout<< "no se puede abrir el fichero de video "<<DATA<<endl;
        video = false ;}
        else
        {video = true;}
    }
    else
    {
        video = true;
    }

    //abrir una ventana para visualizar
    cvNamedWindow( "CamShiftDemo", 1 );
    if (video == true )
    {
        //obtencion del primer fram
        frame = cvQueryFrame( capture );
    }
    else
    {
        char filein[400];
        sprintf(filein,DATA,iter);
        printf("%s\n",filein);
     frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
    }
    frameold = cvCloneImage(frame);
    framecopy=cvCreateImage(cvGetSize(frame),8,3);//copia para poder tocar
    cvCopy( frame,framecopy);

}
void init_ptos()
{
    if (video == true )
    {
        //obtencion del primer fram
        frame = cvQueryFrame( capture );
    }
    else
    {
        char filein[400];
        sprintf(filein,DATA,iter);
        printf("%s\n",filein);
     frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
    }

    cvCopy( frame,framecopy);

    mUpdater.Add(frame,frameold);

    //inicializacion de la posición de la camara con funciones de opencv
    cout <<"marcas visibles en init "<<mEstimator.pMap->visible<<endl;

    CvMat *h;
    h=cvCreateMat(3,1,CV_32FC1);

    for   (list<CElempunto*>::iterator It=mEstimator.pMap->bbdd.begin();It != mEstimator.pMap->bbdd.end();It++)
    {
        mModelCam.cvInverseParam(&h,(*It)->pto);
        (*It)->wx=cvmGet(mEstimator.pDataCam->translation,0,0);
        (*It)->wy=cvmGet(mEstimator.pDataCam->translation,1,0);
        (*It)->wz=cvmGet(mEstimator.pDataCam->translation,2,0);
        (*It)->theta=atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0)));
        (*It)->phi=atan2(cvmGet(h,0,0),cvmGet(h,2,0));
        (*It)->rho=1./RHO_INIT_DIST;
        (*It)->state=st_inited;
        mEstimator.pMap->inited++;
    }

    cout<<"kalman ini"<<endl;
    mEstimator.initState();
    cout<<"kalman inited"<<endl;

}
void data_out()
{

    //dibjo puntos encontrados
    cvCopy( frame,framecopy);
    pDataOut->Draw(framecopy);
#ifndef KALMAN

    pDataOut->Particle(framecopy);
#else

    pDataOut->Disp_out(framecopy);
#endif

    pDataOut->Feat();
    pDataOut->Frame();
    pDataOut->Cam();

    char nombre[100];
    //muestro imagen y compruebo salida
    sprintf(nombre,(pDataOut->resdir+"frame%.2d.png").c_str(),iter);
    cvSaveImage(nombre ,framecopy);
    cvShowImage( "CamShiftDemo", framecopy );
#ifdef LINUX
    char gplotstr[300];
    if (gplot==0)
        gplot= popen("/usr/bin/gnuplot","w");
    sprintf(gplotstr,"splot 'disp%d.txt' u 1:2:3, 'cam.txt' u 1:2:3 w l \n",pDataOut->iter-1);
    fprintf(gplot,gplotstr);
    fflush(gplot);
#endif
    /* splot 'post' u 2:4:6 w l,'post' u 14:15:16, 'post' u 17:18:19, 'post' u 20:21:22, 'post' u 23:24:25, 'post' u 26:27:28, 'post' u 30:31:32, 'post' u 33:34:35, 'post' u 36:37:38, 'post' u 39:40:41*/
}
int main (int argc, char **argv)
{
    cout<<"start.."<<endl;
    //cvSetErrMode( CV_ErrModeSilent  );
    xMainNode=XMLNode::openFileHelper("config.xml","slam");
    const char * t;
    /** Search for tracker in the config.xml
     * Available trackers:
     * - tracker_harris
     * - tracker_surf
     **/
    t=xMainNode.getChildNode("tracker").getText();
    if (!strcmp(t,"tracker_harris"))
    {
        pTracker= new CTracker_harris();
    }
    else if  (!strcmp(t,"tracker_surf"))
    {
        pTracker= new CTracker_surf();
    }
    else
    {
        cout<< "error no tracker"<<endl;
        exit(-1);
    }
    cout<<"TRACKER IS : "<<t<<endl;

    /** Search for data dir in confif.xml **/
    DATA=xMainNode.getChildNode("data").getText();
    cout <<"DATA IS : " <<DATA<<endl;

#ifdef JAVA_GUI
{
    t=xMainNode.getChildNode("gui").getText();
    if (!strcmp(t,"true"))
    {
        env = create_vm();
        invoke_class( );
    }
}
#endif

    t=xMainNode.getChildNode("frame_increment").getText();
    int frame_increment=1;
    sscanf(t,"%d",&frame_increment);
    cout<<"FRAME INCREMENT IS : "<<frame_increment<<endl;

    t=xMainNode.getChildNode("waitkey").getText();
    int waitkey=0;
    sscanf(t,"%d",&waitkey);
    cout<<"WAITKEY IS : "<<waitkey<<endl;

    bool mahalanobis = false;
    if (xMainNode.getChildNode("tests").getChildNode("mahalanobis").isEmpty()==false)
        mahalanobis = true;

    cout<<"MAHALANOBIS: "<< mahalanobis<<endl;

    bool ransac = false;
    if( xMainNode.getChildNode("tests").getChildNode("ransac").isEmpty()==false)
        ransac = true;
    cout<<"RANSAC: "<< ransac <<endl;

    t=xMainNode.getChildNode("resdir").getText();
    string res;
    if (t!=NULL)
        res= t;
    else
        res="";
    pDataOut = new CDataOut(res);
    cout<<"RESDIR: "<<pDataOut->resdir<<endl;

    t=xMainNode.getChildNode("vehicle").getAttribute("id");
    string vehicle_mode = t;
    if (vehicle_mode == "FreeCam"){
        pVehicle = new CFreeCam;
    }else if (vehicle_mode == "RobotXYTh"){
        pVehicle = new CRobotXYTh;
        ((CRobotXYTh*)pVehicle)->set_filename((char*) xMainNode.getChildNode("vehicle").getChildNode("data").getText());
        ((CRobotXYTh*)pVehicle)->set_iter(iter);
    }
    cout<<"VEHICLE: "<<t<<endl;


    //Map Manager initialization

    mEstimator.newMap();


    conect();
    cout<<"param_init"<<endl;
    param_init();

    cout<<"init_video"<<endl;
    init_video( argc, argv);

    cout<<"init_ptos"<<endl;
    init_ptos();

    int n=0;
    int i;

    while(1)
    {
        iter+=frame_increment;
         ((CRobotXYTh*)pVehicle)->set_iter(iter);

        cout <<"ITERACION: "<< iter<<endl;
        // frame = cvQueryFrame( capture );
        cvCopy(frame,frameold);
        if (video == true )
        {
            //obtencion del primer fram
            frame = cvQueryFrame( capture );
        }
        else
        {
            char filein[400];
            sprintf(filein,DATA,iter);
            printf("%s\n",filein);
            printf("precalvoid\n");
#ifdef JAVAGUI
            env->CallVoidMethod(startobj,setImage,env->NewStringUTF(filein));            if (env->ExceptionOccurred())
            {
                env->ExceptionDescribe();
            }
            exit(-1);
#endif
            frame=cvLoadImage(filein,CV_LOAD_IMAGE_COLOR );
        }

        //  compruebo que no se ha acabado el video
        if(frame==NULL)
            break;

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

        if (mahalanobis) mEstimator.Test();

        if (ransac) mUpdater.TestRANSAC();

        mEstimator.UpdateMatrixSize();

        mEstimator.Correct();

        mModelCam.ProjectPoints();

#ifdef KALMAN
        mEstimator.Print(iter);
#endif

        cout<<"update"<<endl;
        mUpdater.update();
        cout<<"dataout"<<endl;

        mEstimator.ManageMap();

        data_out();


        cout<<"waitkey correct"<<endl;
        c = cvWaitKey(waitkey);//esto probablemente se pueda quitar
        if( c == 27 )//si presiono escape salgo del programa limpiamente
            break;
    }
    /// FIN
    //////////////////////////////////

    return 0;
}

