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
#include "highgui.h"

using namespace std;

CDataCam mDataCam;
CModelCam mModelCam;
CMap mMap;
CKalman mKalman;
CUpdater mUpdater;
CTracker_surf mTracker;
CFreeCam mFreeCam;

ofstream outfile;
    ofstream plotr;
int ptox[1000][120];
int ptoy[1000][120];

CvFont font;
double hScale=0.5;
double vScale=0.5;
int    lineWidth=2;
char strID[10];
char c;//donde se guarda las teclas pulsadas   
  //variables deonde se deja el frame y copia para dibujar en ella
IplImage* frame = 0;
IplImage* framecopy=0;

int iter;
void trasf_cov();
void conect()
{
mModelCam.setDataCam(&mDataCam);
mModelCam.setMap(&mMap);

mKalman.setModelCam(&mModelCam);
mKalman.setDataCam(&mDataCam);
mKalman.setMap(&mMap);
mKalman.setModel((CModel*)&mFreeCam);

mUpdater.setDataCam(&mDataCam);
mUpdater.setMap(&mMap);
mUpdater.setModelCam(&mModelCam);

}
void param_init()
{
/*mDataCam.SetFx(629.49);
mDataCam.SetFy(630.74);
mDataCam.SetCx(337.8743788);
mDataCam.SetCy(241.0742116);
mDataCam.frame_width=640;
mDataCam.frame_height=480;
*/
	
mDataCam.SetFx(1000);
mDataCam.SetFy(1000);
mDataCam.SetCx(320);
mDataCam.SetCy(240);
mDataCam.frame_width=640;
mDataCam.frame_height=480;

double dist[4];
dist[0]=-0.1928;
dist[1]= 0.1373;
dist[2]= 0.;
dist[3]= 0.;

/*dist[0]=0.;
dist[1]=0.;
dist[2]=0.;
dist[3]=0.;
*/
CvMat *distMat;
distMat = cvCreateMat(4,1,CV_32FC1);
for ( int i=0;i<4;i++) cvmSet(distMat,i,0,dist[i]);
cvCopy(distMat,mDataCam.distortion);

CvMat *rotation;
CvMat *trans;
rotation=cvCreateMat(3,1,CV_32FC1);
trans=cvCreateMat(3,1,CV_32FC1);
cvmSet(trans,2,0,100);
mDataCam.SetRotation(rotation);
mDataCam.SetTranslation(trans);

mTracker.setDataCam(&mDataCam);
mTracker.setMap(&mMap);

}
void init_ptos()
{

//   frame = cvQueryFrame( capture );//tomo 2 frames para evitar negros
//   mUpdater.Add(frame);
CvMat *rotation;
CvMat *trans;//,*trans2;
rotation=cvCreateMat(3,1,CV_32FC1);
trans=cvCreateMat(3,1,CV_32FC1);
//trans2=cvCreateMat(3,1,CV_32FC1);
/*//inicializacion de la posición de la camara con funciones de opencv
CvMat *image_points;
CvMat *object_points;
image_points=cvCreateMat(2,mMap.visible,CV_32FC1);
object_points=cvCreateMat(3,mMap.visible,CV_32FC1);
*/

for (int iobj = 0; iobj< 40 ;iobj++)
   for ( int jobj= 0; jobj<3;jobj++)
   {
     cout<<"a";
     mMap.add3D(40*jobj,10*iobj,0);
   }
/// PRIMERA FASE SIMULACIÓN DE LA CAMARA
int npto,npos;
npto=0;
npos=0;
for (float inc=0; inc <200; inc+=1){
   cvmSet(rotation,0,0,0.001);
   cvmSet(rotation,1,0,0);
   cvmSet(rotation,2,0,0);
   mDataCam.SetRotation(rotation);
   
   cvmSet(trans,0,0,40);
   cvmSet(trans,1,0,inc);
   cvmSet(trans,2,0,-200);
  
   mDataCam.SetTranslation(trans);
   
   mModelCam.ProjectPoints();
   npto=0;
   for   (list<CElempunto*>::iterator It=mMap.bbdd.begin();It != mMap.bbdd.end();It++)
   {
	//inserto la proyeccion como la vision de tal manera que simulo una camara
	ptox[npos][npto]=(int)(*It)->projx;
	ptoy[npos][npto]=(int)(*It)->projy;
	//cout<<"pto "<<ptox[npos][npto]<<" "<<ptoy[npos][npto]<<" "<<npos*256*256<<" "<<npto<<endl;
	npto++;	
   }
   npos++;
}

int primeros=0;
mMap.inited=0;
mMap.visible=0;
mMap.bbdd.clear();
mMap.ID=-1;
for (int iobj = 0; iobj< 4 ;iobj++)
   for ( int jobj= 0; jobj<3;jobj++)
   {
       if (primeros<10)
       {
            mMap.add3D(40*jobj,10*iobj,0);
        	primeros++;
       }
   }

   cvmSet(rotation,0,0,0.001);
   cvmSet(rotation,1,0,0);
   cvmSet(rotation,2,0,0);
   mDataCam.SetRotation(rotation);

   cvmSet(trans,0,0,40);
   cvmSet(trans,1,0,0);
   cvmSet(trans,2,0,-200);
   
   mDataCam.SetTranslation(trans);
   mKalman.initState();

}
void data_out()
{
//saco un fichero de puntos 3D .. FIXME Anadir 2d
/*
   outfile<< cvmGet(mDataCam.translation,0,0)<<" ";
   outfile<< cvmGet(mDataCam.translation,1,0)<<" ";
   outfile<< cvmGet(mDataCam.translation,2,0)<<" ";
   outfile<<endl;
*/
CvMat *m;
m=cvCreateMat(3,1,CV_32FC1);
CvMat *J;
J=cvCreateMat(3,6,CV_32FC1);

   for   (list<CElempunto*>::iterator It=mMap.bbdd.begin();It != mMap.bbdd.end();It++)
   {
       cvmSet(m,0,0,sin((*It)->phi));
       cvmSet(m,1,0,-sin((*It)->theta));
       cvmSet(m,2,0,cos((*It)->phi));
       cvNormalize( m, m);

   outfile<<(*It)->wx +cvmGet(m,0,0)/(*It)->rho<<" ";
   outfile<<(*It)->wy +cvmGet(m,1,0)/(*It)->rho<<" ";
   outfile<<(*It)->wz +cvmGet(m,2,0)/(*It)->rho<<" ";
   
	//outfile<<(*It)->wx<<" ";
	//outfile<<(*It)->wy<<" ";
//	outfile<<(*It)->wz<<" ";
    outfile<<(*It)->wx_s+(*It)->wy_s+(*It)->wz_s<<" ";
	outfile<<endl;
   }
   outfile<< cvmGet(mDataCam.translation,0,0)<<" ";
   outfile<< cvmGet(mDataCam.translation,1,0)<<" ";
   outfile<< cvmGet(mDataCam.translation,2,0)<<" ";
   outfile<<"65000 ";
   
   outfile<<endl;


   cout<< "rotation ";
   cout<< cvmGet(mDataCam.rotation,0,0)<<" ";
   cout<< cvmGet(mDataCam.rotation,1,0)<<" ";
   cout<< cvmGet(mDataCam.rotation,2,0)<<" ";

   cout<<endl;

//dibjo puntos encontrados
//   cvCopy( frame,framecopy);
   //int rvis=0;
   for ( list<CElempunto*>::iterator It=mMap.bbdd.begin();
            It != mMap.bbdd.end(); It++ )
   {
        if ((*It)->state!=st_empty){
            sprintf(strID,"%d",(*It)->ID);
        	if((*It)->state==3)
        	{
                cvPutText (framecopy,strID,(*It)->pto, &font, cvScalar(0,255,0)); 
        	}else
        	{
        	    cvPutText (framecopy,strID,(*It)->pto, &font, cvScalar(255,0,0)); 
        	}
            //rvis++;
    		cvPutText (framecopy,strID,cvPoint((int)(*It)->projx,(int)(*It)->projy), &font, cvScalar(0,255,255)); 
      }
      cout<<"punto "<<(*It)->ID<<"x "<< (*It)->pto.x<<"y "<<(*It)->pto.y<<"dif "<< (*It)->pto.x-(*It)->projx<<" "<<(*It)->pto.y-(*It)->projy<<" "<<(*It)->state<<endl;
   }


int ii=0;
plotr<<"library(rgl) "<<endl;
plotr<<"rgl.clear(\"all\")"<<endl;
plotr<<"rgl.bg(sphere = TRUE, color = c(\"black\", \"green\"), lit = FALSE, size=2, alpha=0.2, back = \"lines\")"<<endl;
plotr<<"rgl.light()"<<endl;
plotr<<"rgl.bbox()"<<endl;
CvMat* cov;
CvMat* transf_cov;
cov=cvCreateMat(6,6,CV_32FC1);
transf_cov=cvCreateMat(3,3,CV_32FC1);

for ( list<CElempunto*>::iterator It=mMap.bbdd.begin();
            It != mMap.bbdd.end(); It++ )
{
     if((*It)->state>2)
  	 {
           for (int i=0; i<6 ; i++)
               for (int j=0; j<6;j++)
                   {
                       cvmSet(cov,i,j,cvmGet(mKalman.pKalman->error_cov_post,i+12+ii*6,j+12+ii*6));     
                   }

        //Jacobiano
       cvmSet(J,0,0,1);cvmSet(J,0,1,0);cvmSet(J,0,2,0);
       cvmSet(J,0,3,0);
       cvmSet(J,0,4,cos((*It)->phi)/(*It)->rho);
       cvmSet(J,0,5,-sin((*It)->phi)/((*It)->rho*(*It)->rho));
       
       cvmSet(J,1,0,0);cvmSet(J,1,1,1);cvmSet(J,1,2,0);
       cvmSet(J,1,3,-cos((*It)->theta)/(*It)->rho);
       cvmSet(J,1,4,0);
       cvmSet(J,1,5,sin((*It)->theta)/((*It)->rho*(*It)->rho));
       
       cvmSet(J,2,0,0);cvmSet(J,2,1,0);cvmSet(J,2,2,1);
       cvmSet(J,2,3,0);
       cvmSet(J,2,4,-sin((*It)->phi)/(*It)->rho);
       cvmSet(J,2,5,-cos((*It)->phi)/((*It)->rho*(*It)->rho));
   //sx2
       double temp=0;
         for(int l=0; l<3;l++)
            for (int n=0;n<3;n++)
            {
                temp=0;
                for (int i =0 ;i <6; i++)
                    for (int k=0; k<6; k++)
                        {
                            temp+=cvmGet(cov,i,k)*cvmGet(J,l,i)*cvmGet(J,n,k);
                        }        
                
                cvmSet(transf_cov,l,n,temp);
            }

          plotr<<"p"<<ii<< " <- matrix(c(" ;
      	   
           for (int i=0; i<2 ; i++)
               for (int j=0; j<3;j++)
                   {
//                       plotr<<cvmGet(mKalman.pKalman->error_cov_post,i+12+ii*3,j+12+ii*3)<<",";     
                       plotr<<cvmGet(transf_cov,i,j)<<",";     
                   }
           plotr<<cvmGet(transf_cov,2,0)<<",";     
           plotr<<cvmGet(transf_cov,2,1)<<",";     
           plotr<<cvmGet(transf_cov,2,2);     
           plotr<<"),3,3)"<<endl;

       cvmSet(m,0,0,sin((*It)->phi));
       cvmSet(m,1,0,-sin((*It)->theta));
       cvmSet(m,2,0,cos((*It)->phi));
       cvNormalize( m, m);

           plotr<<"pos <- c("<<(*It)->wx+cvmGet(m,0,0)/(*It)->rho<<", ";
           plotr<<(*It)->wy+cvmGet(m,1,0)/(*It)->rho<<", ";
           plotr<<(*It)->wz+cvmGet(m,2,0)/(*It)->rho<<")"<<endl;
           plotr<<"plot3d( ellipse3d(p"<<ii<<",centre=";
           plotr<<"pos), col=\"blue\", alpha=0.5, add = TRUE) "<<endl;
     } 
     ii++;
}
cvReleaseMat(&cov);
cvReleaseMat(&transf_cov);

plotr<<"p"<<ii<< " <- matrix(c(" ;
   for (int i=0; i<2 ; i++)
       for (int j=0; j<3;j++)
           {
               plotr<<cvmGet(mKalman.pKalman->error_cov_post,2*i,2*j)<<",";     
           }
   plotr<<cvmGet(mKalman.pKalman->error_cov_post,4,0)<<",";     
   plotr<<cvmGet(mKalman.pKalman->error_cov_post,4,2)<<",";     
   plotr<<cvmGet(mKalman.pKalman->error_cov_post,4,4);     
   plotr<<"),3,3)"<<endl;
   plotr<<"pos <- c("<< cvmGet(mDataCam.translation,0,0)<<", ";
   plotr<< cvmGet(mDataCam.translation,1,0)<<", ";
   plotr<< cvmGet(mDataCam.translation,2,0)<<") "<<endl;
   plotr<<"plot3d( ellipse3d(p"<<ii<<",centre=";
           plotr<<"pos), col=\"red\", alpha=0.5, add = TRUE) "<<endl;
   plotr<<"rgl.viewpoint(45,30)"<<endl;
   plotr<<"rgl.snapshot(\"c:\\\\out"<<iter<<".png\")"<<endl;
//plotr.close();

/* cout<<"visibles teoricos "<<mMap.visible<<endl;
   cout<< cvmGet(mDataCam.trans_world,0,0)<<" ";
   cout<< cvmGet(mDataCam.trans_world,1,0)<<" ";
   cout<< cvmGet(mDataCam.trans_world,2,0)<<endl;*/

// cout<<"visibles reales "<<rvis<<endl;
  
// muestro imagen y compruebo salida
   cvShowImage( "CamShiftDemo", framecopy );
/* splot 'post' u 2:4:6 w l,'post' u 14:15:16, 'post' u 17:18:19, 'post' u 20:21:22, 'post' u 23:24:25, 'post' u 26:27:28, 'post' u 30:31:32, 'post' u 33:34:35, 'post' u 36:37:38, 'post' u 39:40:41*/
   cvReleaseMat(&m);


}
int main (int argc, char **argv)
{
	
	//cvSetErrMode( CV_ErrModeSilent  );

    plotr.open("plot.r");
    outfile.open("res3");
    //abrir una ventana para visualizar
    cvNamedWindow( "CamShiftDemo", 1 );
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
   framecopy=cvCreateImage(cvSize(640,480),8,3);//copia para poder tocar

conect();
param_init();

//init_video( argc, argv);
init_ptos();

///test 1

//cvSetErrMode(2);
int n=0;
int i;
iter=0;
while(1)
{
iter++;
cout <<"ITERACION: "<< iter<<endl;
//  frame = cvQueryFrame( capture );
//  compruebo que no se ha acabado el video
//  if(frame==NULL) break;

//   mUpdater.Add(frame);
//   match frame

//   mTracker.Match(frame);//entre los puntos de la imagen anterior y los de esta
//   ADD
//esto anadiría puntos ya anadidos....

//Match simulado ...
   i=0;
   for ( list<CElempunto*>::iterator It=mMap.bbdd.begin();
            It != mMap.bbdd.end(); It++ )
   {
     if((ptox[n][(*It)->ID]<620)&&
		(ptox[n][(*It)->ID]>20)&&
	    (ptoy[n][(*It)->ID]<460)&&
	    (ptoy[n][(*It)->ID]>20))
       {
            (*It)->pto.x=ptox[n][(*It)->ID];
            (*It)->pto.y=ptoy[n][(*It)->ID];         	
            cout<<"acutalizando "<<(*It)->ID<<endl;
       }else
       {
            (*It)->state=st_no_view;
            	mMap.visible--;
       //     (*It)->pto.x=0;
       //  	  (*It)->pto.y=0;
         	
       //   	cout<<"acutalizando "<<(*It)->ID<<endl;
       }
       i++;
   }

   for(int j=i;j<120;j++)
   {
	if((ptox[n][j]<620)&&
            (ptox[n][j]>20)&&
	    (ptoy[n][j]<460)&&
	    (ptoy[n][j]>20))
	{
//         mMap.add(ptox[i][n],ptoy[i][n]);
           CElempunto *pelempunto;
           pelempunto=new CElempunto();
           pelempunto->pto.x=ptox[n][j];
           pelempunto->pto.y=ptoy[n][j];
           pelempunto->old_pto.x=ptox[n][j];
           pelempunto->old_pto.y=ptoy[n][j];
           pelempunto->ID=j;//mMap.get_ID();//bbdd.back()->ID+1;
	   cout<<"anadido ID:"<<pelempunto->ID<<endl;
           pelempunto->wx=0.;//pts[i].x;
           pelempunto->wy=0.;//pts[i].y;
           pelempunto->wz=0.0;
           pelempunto->phi=0;
           pelempunto->theta=0;
           pelempunto->rho=999999999;
           pelempunto->wx_s=0.;
           pelempunto->wy_s=0.;
           pelempunto->wz_s=0.0;
           
           pelempunto->state=st_empty;

           mMap.bbdd.push_back(pelempunto);
           mMap.visible++;
	   cout<<"insertado "<<j<<endl;
	}
   }

   n++;
   mUpdater.remove();

//cuenta el numero de puntos iniciados yvisibles???
      
   mKalman.UpdateMatrixSize();   
   cout<<"predict"<<endl;
   mKalman.Predict();
   
   mKalman.UpdateMatrixSize();   
   
   // mKalman.Test();
   cout<<"correct"<<endl;
   mKalman.Correct();
   
  // mKalman.Print();
   cout<<"updater update"<<endl;
   mUpdater.update();
   mKalman.UpdateMatrixSize();  
   data_out();
   c = cvWaitKey(0);//esto probablemente se pueda quitar
   if( c == 27 )//si presiono escape salgo del programa limpiamente
     break;
   cvSetZero(framecopy);

}
/// FIN
//////////////////////////////////
   outfile.close();	

return 0;
}
void transf_cov()
{

}
