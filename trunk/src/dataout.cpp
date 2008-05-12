#include "dataout.h"

CDataOut::CDataOut():iter(0),hScale(0.5),vScale(0.5),lineWidth(2)
{
  FeatFile.open("feat.txt");
     RFile.open("plot.r");
     CamFile.open("cam.txt");
      cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);

}
CDataOut::~CDataOut()
{

}
void CDataOut::Draw(IplImage *framecopy)
{
       //int rvis=0;
  for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
	It != pMap->bbdd.end(); It++ )
    {
      if ((*It)->state!=st_empty){
        sprintf(strID,"%d",(*It)->ID);
    	if((*It)->state==3)
    	  {
    	    cvPutText (framecopy,strID,(*It)->pto, &font, cvScalar(0,255,0));
    	  }else if((*It)->state==4)
    	  {
    	    cvPutText (framecopy,strID,(*It)->pto, &font, cvScalar(255,0,0));
    	  }else
    	  {
    	    cvPutText (framecopy,strID,(*It)->pto, &font, cvScalar(0,0,255));
    	  }
    	cvPutText (framecopy,strID,cvPoint((int)(*It)->projx,(int)(*It)->projy), &font, cvScalar(0,255,255));
      }
    }

}
void CDataOut::R_Out()
{
  int ii=0;
  RFile<<"library(rgl) "<<endl;
  RFile<<"rgl.clear(\"all\")"<<endl;
  RFile<<"rgl.bg(sphere = TRUE, color = c(\"black\", \"green\"), lit = FALSE, size=2, alpha=0.2, back = \"lines\")"<<endl;
  RFile<<"rgl.light()"<<endl;
  RFile<<"rgl.bbox()"<<endl;
  CvMat *temp2,*temp3;
  temp2=cvCreateMat(3,6,CV_32FC1);
  temp3=cvCreateMat(3,3,CV_32FC1);
  	    CvMat* m = cvCreateMat(3,1,CV_32FC1);
  CvMat *temp,*jtemp;
  temp=cvCreateMatHeader(6,6,CV_32FC1);
  jtemp=cvCreateMat(3,6,CV_32FC1);
  for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
	It != pMap->bbdd.end(); It++ )
    {
      if((*It)->state>2)
	{
	  float s_th=sin((*It)->theta);
	  float c_th=cos((*It)->theta);
	  float s_ph=sin((*It)->phi);
	  float c_ph=cos((*It)->phi);

	  cvmSet(m,0,0,sin((*It)->phi));
	  cvmSet(m,1,0,-sin((*It)->theta));
	  cvmSet(m,2,0,cos((*It)->phi));
	  cvNormalize( m, m);

	  float xreal=(*It)->wx +cvmGet(m,0,0)/(*It)->rho;
	  float yreal=(*It)->wy +cvmGet(m,1,0)/(*It)->rho;
	  float zreal=(*It)->wz +cvmGet(m,2,0)/(*It)->rho;


	  cvZero(jtemp);
	  cvmSet(jtemp,0,0,1);
	  cvmSet(jtemp,1,1,1);
	  cvmSet(jtemp,2,2,1);

	  cvmSet(jtemp,0,3,(-s_th*s_ph)/(*It)->rho);
	  cvmSet(jtemp,1,3,(-c_th     )/(*It)->rho);
	  cvmSet(jtemp,2,3,(-s_th*c_ph)/(*It)->rho);

	  cvmSet(jtemp,0,4,(c_th*c_ph)/(*It)->rho);
	  cvmSet(jtemp,1,4,(0    )/(*It)->rho);
	  cvmSet(jtemp,2,4,(-c_th*s_ph)/(*It)->rho);

	  cvmSet(jtemp,0,5,(-c_th*s_ph)/((*It)->rho*(*It)->rho));
	  cvmSet(jtemp,1,5,(s_th     )/((*It)->rho*(*It)->rho));
	  cvmSet(jtemp,2,5,(-c_th*c_ph)/((*It)->rho*(*It)->rho));

	  if (12+ii*6< pKalman->pKalman->error_cov_post->width && 12+ii*6< pKalman->pKalman->error_cov_post->height)
	    {

	      cvGetSubRect( pKalman->pKalman->error_cov_post,temp,cvRect(12+ii*6,12+ii*6,6,6) );
	      cvMatMul(jtemp,temp,temp2);
	      cvGEMM( temp2,jtemp,1,NULL,0,temp3,CV_GEMM_B_T );

	      RFile<<"p"<<ii<< " <- matrix(c(" ;
	      for (int i=0; i<2 ; i++)
		for (int j=0; j<3;j++)
		  {
		    RFile<<cvmGet(temp3,i,j)<<",";
		  }
	      RFile<<cvmGet(temp3,2,0)<<",";
	      RFile<<cvmGet(temp3,2,1)<<",";
	      RFile<<cvmGet(temp3,2,2);
	      RFile<<"),3,3)"<<endl;
	      RFile<<"pos <- c("<<xreal<<", ";
	      RFile<<yreal<<", ";
	      RFile<<zreal<<")"<<endl;
	      RFile<<"try(plot3d( ellipse3d(p"<<ii<<",centre=";
	      RFile<<"pos), col=\"blue\", alpha=0.5, add = TRUE) )"<<endl;
	    }
	}
      ii++;
    }
  RFile<<"p"<<ii<< " <- matrix(c(" ;
  for (int i=0; i<2 ; i++)
    for (int j=0; j<3;j++)
      {
	RFile<<cvmGet(pKalman->pKalman->error_cov_post,2*i,2*j)<<",";
      }
   RFile<<cvmGet(pKalman->pKalman->error_cov_post,4,0)<<",";
   RFile<<cvmGet(pKalman->pKalman->error_cov_post,4,2)<<",";
   RFile<<cvmGet(pKalman->pKalman->error_cov_post,4,4);
   RFile<<"),3,3)"<<endl;
   RFile<<"pos <- c("<< cvmGet(pDataCam->translation,0,0)<<", ";
   RFile<< cvmGet(pDataCam->translation,1,0)<<", ";
   RFile<< cvmGet(pDataCam->translation,2,0)<<") "<<endl;
   RFile<<"plot3d( ellipse3d(p"<<ii<<",centre=";
   RFile<<"pos), col=\"red\", alpha=0.5, add = TRUE) "<<endl;
   RFile<<"rgl.viewpoint(45,30)"<<endl;
   RFile<<"rgl.snapshot(\"c:\\\\out"<<iter<<".png\")"<<endl;
   //RFile.close();

   cvReleaseMat(&temp);
   cvReleaseMat(&temp2);
   cvReleaseMat(&temp3);
   cvReleaseMat(&jtemp);

}
void CDataOut::Particle(IplImage *framecopy)
{
    CvScalar c[pMap->bbdd.size()];
    for (int j =0; j<pMap->bbdd.size();j++)
    {
        c[j]=cvScalar(rand()*255.0/RAND_MAX,rand()*255.0/RAND_MAX,rand()*255.0/RAND_MAX);
    }
    for (int p=0; p<pParticleFilter->num_particles;p++)
    {
        int i=0;
        for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
            It != pMap->bbdd.end(); It++ )
        {

            if (framecopy != NULL){
                 /*  cvCircle (framecopy,cvPoint(pParticleFilter->pred_measure[i][p],
                                                pParticleFilter->pred_measure[i+1][p]),
                            1,c[i/2],1 );*/
                   cvCircle (framecopy,cvPoint(pParticleFilter->pred_measure[i][p],
                                                pParticleFilter->pred_measure[i+1][p]),
                            1,cvScalar(0,0,pParticleFilter->weights[p]*10001.0),1 );
            }
            i+=2;
        }
    }
}
void CDataOut::Disp_out(IplImage *framecopy)
{
  char ndisp[100];
  sprintf(ndisp,"disp%d.txt",iter++);
  DispFile.open(ndisp);
  int ii=0;
  CvMat *temp,*temp2,*temp3;
  temp=cvCreateMatHeader(6,6,CV_32FC1);
  temp2=cvCreateMat(3,6,CV_32FC1);
  temp3=cvCreateMat(3,3,CV_32FC1);
  CvMat* vect=cvCreateMat (6,1,CV_32FC1);
  CvMat* res6=cvCreateMat (6,1,CV_32FC1);
  CvMat* vect2=cvCreateMat(1,6,CV_32FC1);
  CvMat* proj = cvCreateMat(4,2,CV_32FC1);    ///NOT RELEASED
  CvMat* m = cvCreateMat(3,1,CV_32FC1);

  for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
	    It != pMap->bbdd.end(); It++ )
    {
      if((*It)->state>2)
	  {

	  cvmSet(m,0,0,cos((*It)->theta)*sin((*It)->phi));
	  cvmSet(m,1,0,-sin((*It)->theta));
	  cvmSet(m,2,0,cos((*It)->theta)*cos((*It)->phi));
	  cvNormalize( m, m);

	/*  float xreal=(*It)->wx +cvmGet(m,0,0)/(*It)->rho;
	  float yreal=(*It)->wy +cvmGet(m,1,0)/(*It)->rho;
	  float zreal=(*It)->wz +cvmGet(m,2,0)/(*It)->rho;
*/
	  if (12+ii*6< pKalman->pKalman->error_cov_post->width && 12+ii*6< pKalman->pKalman->error_cov_post->height)
	    {

	      cvGetSubRect( pKalman->pKalman->error_cov_post,temp,cvRect(12+ii*6,12+ii*6,6,6) );

        for (int part=0; part<40; part++){
		  cvmSet(vect,0,0,randomVector(-.005,.005));
  		  cvmSet(vect,1,0,randomVector(-.005,.005));
  		  cvmSet(vect,2,0,randomVector(-.005,.005));
		  cvmSet(vect,3,0,randomVector(-.005,0.005));
		  cvmSet(vect,4,0,randomVector(-.005,0.005));
		  cvmSet(vect,5,0,randomVector(-1,1));

          cvMatMul(temp,vect,res6);

          cvmSet(m,0,0,cos(cvmGet(res6,3,0)+(*It)->theta)*sin(cvmGet(res6,4,0)+(*It)->phi));
  		  cvmSet(m,1,0,-sin(cvmGet(res6,3,0)+(*It)->theta));
  		  cvmSet(m,2,0,cos(cvmGet(res6,3,0)+(*It)->theta)*cos(cvmGet(res6,4,0)+(*It)->phi));
    	  cvNormalize( m, m);
    	  cvmSet (vect2,0,0,((cvmGet(res6,0,0)+(*It)->wx)+cvmGet(m,0,0)/(cvmGet(res6,5,0)+(*It)->rho)));
    	  cvmSet(vect2,0,1,((cvmGet(res6,1,0)+(*It)->wy)+cvmGet(m,1,0)/(cvmGet(res6,5,0)+(*It)->rho)));
   	      cvmSet(vect2,0,2,((cvmGet(res6,2,0)+(*It)->wz)+cvmGet(m,2,0)/(cvmGet(res6,5,0)+(*It)->rho)));

        DispFile<<cvmGet(vect2,0,0)<<" ";
        DispFile<<cvmGet(vect2,0,1)<<" ";
        DispFile<<cvmGet(vect2,0,2)<<" ";
        DispFile<<ii<<endl;
          cvmSet(vect2,0,0,cvmGet(res6,0,0)+(*It)->wx);
          cvmSet(vect2,0,1,cvmGet(res6,1,0)+(*It)->wy);
          cvmSet(vect2,0,2,cvmGet(res6,2,0)+(*It)->wz);
          cvmSet(vect2,0,3,cvmGet(res6,3,0)+(*It)->theta);
          cvmSet(vect2,0,4,cvmGet(res6,4,0)+(*It)->phi);
          cvmSet(vect2,0,5,cvmGet(res6,5,0)+(*It)->rho);

             pModelCam->cvProject_1_pto(vect2,proj,NULL,NULL,NULL);
             if (framecopy != NULL){
    		   cvCircle (framecopy,cvPoint(cvmGet(proj,0,0),cvmGet(proj,0,1)),1,cvScalar(0,0,255),1 );
             }
   	      }

	    }
	  }
      ii++;
    }
    cvReleaseMatHeader(&temp);
    cvReleaseMat(&temp2);
    cvReleaseMat(&temp3);
    cvReleaseMat(&vect);
    cvReleaseMat(&vect2);
    cvReleaseMat(&res6);
    cvReleaseMat(&proj);
    DispFile.close();
}
void CDataOut::Cam()
{
  CamFile<< cvmGet(pDataCam->translation,0,0)<<" ";
  CamFile<< cvmGet(pDataCam->translation,1,0)<<" ";
  CamFile<< cvmGet(pDataCam->translation,2,0)<<" ";
  CamFile<<"65000 ";
  CamFile<<"65000 ";
  CamFile<<endl;
}
void CDataOut::Feat()
{

  CvMat *m;
  m=cvCreateMat(3,1,CV_32FC1);

  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
	///FIXME PONER en una funcion para que solo esté en un sitio
      cvmSet(m,0,0,cos((*It)->theta)*sin((*It)->phi));
      cvmSet(m,1,0,-sin((*It)->theta));
      cvmSet(m,2,0,cos((*It)->theta)*cos((*It)->phi));
      cvNormalize( m, m);
      FeatFile<<(*It)->wx +cvmGet(m,0,0)/(*It)->rho<<" ";
      FeatFile<<(*It)->wy +cvmGet(m,1,0)/(*It)->rho<<" ";
      FeatFile<<(*It)->wz +cvmGet(m,2,0)/(*It)->rho<<" ";
      FeatFile<<(*It)->ID<<" ";
      FeatFile<<(*It)->count;
      FeatFile<<endl;
    }
  cvReleaseMat(&m);

}
void CDataOut::Frame()
{
  char nombre[100];
  sprintf(nombre,"frame%.2d.txt",iter);
  FrameFile.open(nombre);
    for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
	It != pMap->bbdd.end(); It++ )
    {
      if ((*It)->state==st_inited){
      	FrameFile<<(*It)->ID<<" "<< (*It)->pto.x<<" "<<(*It)->pto.y<<" "<<(*It)->state<<endl;
      }
    }
}
void CDataOut::setMap(CMap *p)
{
  pMap=p;
}
void CDataOut::setDataCam(CDataCam *p)
{
  pDataCam=p;
}
void CDataOut::setModelCam(CModelCam *p)
{
  pModelCam=p;
}

void CDataOut::setTracker(CTracker *p)
{
  pTracker=p;
}
void CDataOut::setKalman(CKalman *p)
{
  pKalman=p;
}
void CDataOut::setParticle(CParticleFilter *p){
  pParticleFilter = p;
}

float CDataOut::randomVector(float max,float min)
{
     float x = min;
     float y = max-min;
return  x+ (y*rand()/(RAND_MAX+1.0));

}
