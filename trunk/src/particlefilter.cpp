#include "particlefilter.h"

/**
* @fn constructor
* initializates memory for particles as a double array particles[variable][particle]
**/
CParticleFilter::CParticleFilter():num_max_variables(400),num_max_particles(100),num_particles(100),num_variables(200),
                                   fdims(6),num_max_measurements(300),threshold(20),var_modelo(0)
{
   particles = new double*[num_max_variables];
   for(int i=0; i<num_max_variables;i++){
      particles[i]=new double[num_max_particles];
   }
   A= new double*[num_max_variables];
   for (int i=0; i<num_max_variables; i++){
       A[i]=new double [num_max_variables];
   }
   rand_num = new double [num_max_variables];
   var_modelo=new double [num_max_variables];
   for(int i=0;i<12;i++) var_modelo[i]=0.0;
   for(int i=12;i<num_max_variables;i++) var_modelo[i]=0.0;

   int ii=0;
    for (double i =-2; i<2; i+=0.02){
         ERF[ii++]=erf(i);
         cout<<"erf( "<<ii-1<<") "<<ERF[ii-1]<<endl;
     }
   pred_measure=new double* [num_max_measurements];
   for (int i =0; i<num_max_particles;i++){
       pred_measure[i]=new double[num_max_particles];
   }
   measure=new double [num_max_measurements];
   reject=new bool [num_max_particles];
   state=new double [num_max_variables];

   weights=new double [num_max_particles];

   trans=cvCreateMat(3,1,CV_32FC1);
   rotation=cvCreateMat(3,1,CV_32FC1);

}
CParticleFilter::~CParticleFilter()
{
   for (int i=0; i<num_variables; i++)
   {
       delete particles[i];
       delete A[i];
   }
   delete particles;
   delete A;
}

   /** given an estate propagate particle state using a model **/

void CParticleFilter::Predict()
{
     cout<<"predict start"<<endl;
     double *temp;
     temp=new double[num_variables];

     for (int n_part=0; n_part<num_particles; n_part++)
     {
         /** A*x(:,n_part)+B+W **/
         /** FIXME Esta parte es completamente cerrada a nuestro problema **/
         noise(rand_num,var_modelo,num_variables);

         for(int n_var=0; n_var<num_variables;n_var++)
         {
             temp[n_var]=0;
             for (int n_var2=0; n_var2<num_variables;n_var2++)
             {
                 /** modelo lineal **/
                 temp[n_var]+=A[n_var][n_var2]*particles[n_var2][n_part];
             }
             temp[n_var]+=rand_num[n_var];
//             cout<<"rand_num "<<rand_num[n_var]<<" var "<<var_modelo[n_var]<<endl;
         }
         for (int n_var=0;n_var<num_variables;n_var++){
//             cout<<"particle old "<<particles[n_var][n_part];
             particles[n_var][n_part]=temp[n_var];
//             cout<<" new "<<particles[n_var][n_part]<<endl;
         }
     }
     cout<<"predict stop"<<endl;
     delete[] temp;
}
void CParticleFilter::Correct()
{
     cout<<"correct start"<<endl;
     CvMat *obj;
     CvMat *img;
     obj = cvCreateMat(1,6,CV_32FC1);
     img = cvCreateMat(4,2,CV_32FC1);
     int n_var=pModel->getStateNum();
     int n_meas=0;
     int n_feat;
     double measure_[pMap->bbdd.size()*2];
     cout<<"get measurement"<<endl;
     int ii=0;//pModel->getMeasurementNum();
     for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
     {
        if((*It)->state==st_inited){
        	measure_[ii]=(*It)->pto.x;
        	ii++;
        	measure_[ii]=(*It)->pto.y;
        	ii++;
        	cout<<"ID "<<(*It)->ID<<"pto.x "<<(*It)->pto.x<<" pto.y "<<(*It)->pto.y;
        }
     }
    int kk;
     for (int n_part=0; n_part<num_particles; n_part++)
     {
         /** calculo de el valor de medida correspondiente a esta parícula **/
         /** En nuestro caso es el modelo de la cámara **/
         n_meas=0;
         n_var=0;
         kk=0;
         for(int i =0; i<3;i++){
            cvmSet(trans,i,0,particles[n_var++][n_part]);
            n_var++;
//            cout<<"trans "<<particles[n_var-2][n_part]<<endl;
         }

         for(int i =0; i<3;i++){
            cvmSet(rotation,i,0,particles[n_var++][n_part]);
            n_var++;
//            cout<<"rotation "<<particles[n_var-2][n_part]<<endl;
         }

         pDataCam->SetRotation(rotation);
         pDataCam->SetTranslation(trans);

     for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
     {
              for (int i=0; i<6; i++){
                  cvmSet(obj,0,i,particles[n_var++][n_part]);
//                  cout<<"obj ("<<(n_var-1)<<") "<<particles[n_var-1][n_part]<<endl;
              }

              pModelCam->cvProject_1_pto(obj,img,NULL,NULL,NULL);
              for (int i=0;i<2;i++){
                 pred_measure[n_meas++][n_part]=cvmGet(img,0,i);
//                cout<<"meas ("<<(n_meas-1)<<") "<<pred_measure[n_meas-1][n_part]<<endl;
              }
              for(int i=0;i<2;i++){
//                  cout<<"real meas("<<kk<<") " <<measure_[kk++]<<endl;
              }

     }
          reject[n_part]=false;
     int p=0;
     int m=0;

     double w=0;
     double err=0;
    weights[n_part]=0;
     for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
     {
         w=0;
        if((*It)->state==st_inited){
//              cout<<"pred_measure "<<pred_measure[p][n_part]<<" measure "<<measure_[m]<< endl;
            err=(pred_measure[p][n_part]-measure_[m])*(pred_measure[p][n_part]-measure_[m]);
            weights[n_part]+=(err);

            if (sqrt(err)>threshold){
                 reject[n_part]=true;
                 cout<<"true"<<endl;
            }else{
                 cout<<"false"<<endl;
            }
            m+=2;
          }
          p+=2;
     }
      weights[n_part]=sqrt(weights[n_part]);

      cout<<"wights dist "<<weights[n_part]<<endl;
      double sigma=20;
      weights[n_part]=(1/(sqrt(sigma*2*3.14159)))*exp((-weights[n_part]*weights[n_part])/(sigma*1));
      cout<<"weights "<<weights[n_part]<<endl;

      }//end particles

      /** normalize weights **/
      double suma=0;
      for (int i=0;i<num_particles;i++){
          suma+=weights[i];
          cout<<"weights "<<weights[i]<<endl;
      }
      cout<<"suma weights "<<suma<<endl;
      for (int i=0;i<num_particles;i++){
          weights[i]/=suma;
          cout<<"weights normalized"<<weights[i]<<endl;
      }

     // int n_v;
      for (int v=0; v<num_variables;v++){
          state[v]=0;
       //   n_v=0;
          for (int n_part=0; n_part<num_particles; n_part++){
              //if (reject[n_part]==false){
                 state[v]+=(weights[n_part]*particles[v][n_part]);
               //  n_v++;
              //}else{
      //          cout<<"rejected "<<n_part<<endl;
              //}
          }
          //state[v]/=n_v;
      }

      for (int i=0; i<num_particles;i++)
      {
          if (weights[i]<0.00001&&reject[i]==false)
          {
              cout<<"reject["<<i<<"] for low weight"<<endl;
               reject[i]=true;
          }
      }
      int kstate=0;
   kk=0;
  for(int i =0; i<3;i++){
    cvmSet(trans,i,0,state[kstate++]);
    kstate++;
  }
  for(int i =0; i<3;i++){
    cvmSet(rotation,i,0,state[kstate++]);
    kstate++;
  }

  pDataCam->SetRotation(rotation);
  pDataCam->SetTranslation(trans);
  cout<<"_trans "<<cvmGet(trans,0,0)<<" "<<cvmGet(trans,1,0)<<" "<<cvmGet(trans,1,0)<<endl;
  cout<<"translation "<<cvmGet(pDataCam->translation,0,0)<<" ";
  cout<< cvmGet(pDataCam->translation,1,0)<<" ";
  cout<< cvmGet(pDataCam->translation,2,0)<<" "<<endl;

  ii=pModel->getStateNum();
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited||(*It)->state==st_no_view)
	{
	  (*It)->wx=state[ii];
//	  (*It)->wx_s=cvmGet(pKalman->error_cov_post,ii,ii);
	  ii++;
	  (*It)->wy=state[ii];
//	  (*It)->wy_s=cvmGet(pKalman->error_cov_post,ii,ii);
	  ii++;
	  (*It)->wz=state[ii];
//	  (*It)->wz_s=cvmGet(pKalman->error_cov_post,ii,ii);
	  ii++;
	  (*It)->theta=state[ii];
	  ii++;
	  (*It)->phi=state[ii];
	  ii++;
	  (*It)->rho=state[ii];
	  ii++;
	}
    }
    cout<<"projectando resultados "<<endl;
       kstate=0;

  cout<<"antes trans "<<endl;
  for(int i =0; i<3;i++){
    cvmSet(trans,i,0,state[kstate]);
    kstate++;
    kstate++;
  }

  cout<<"antes rot "<<endl;
  for(int i =0; i<3;i++){
    cvmSet(rotation,i,0,state[kstate]+0.00001);
    kstate++;
    kstate++;
  }

  pDataCam->SetRotation(rotation);
  pDataCam->SetTranslation(trans);

  pModelCam->ProjectPoints();
     cvReleaseMat(&obj);
     cvReleaseMat(&img);
/** Añado partículas nuevas en los sitios de las rechazadas **/
for (int n_part=0; n_part<num_particles; n_part++){
//    for (int v=0; v<num_variables;v++){
          if (reject[n_part]==true){
                 initParticle(n_part);
  //               particles[v][n_part]=state[v];
    //      }
    }
    reject[n_part]=false;
}

     cout<<"correct stop "<<endl;
}
void CParticleFilter::Test()
{

}
void CParticleFilter::initParticle(int part)
{
      int kstate=0;
      for( int i=0; i<3; i++){
           particles[kstate++][part]=cvmGet(pDataCam->translation,i,0);
           particles[kstate++][part]=cvmGet(pDataCam->translation,i,0);
      }

     for(int i =0; i<3;i++){
          particles[kstate++][part]=cvmGet(pDataCam->rotation,i,0);
          particles[kstate++][part]=cvmGet(pDataCam->rotation,i,0);
     }

  // inicializaciÃ³n primeros puntos vistos
    for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      //if((*It)->state==st_inited){
    	particles[kstate++][part]=(*It)->wx+Random(-0.0001,0.0001);
    	particles[kstate++][part]=(*It)->wy+Random(-0.0001,0.0001);
    	particles[kstate++][part]=(*It)->wz+Random(-0.0001,0.0001);
    	particles[kstate++][part]=(*It)->theta+Random(-0.00005,0.00005);
    	particles[kstate++][part]=(*It)->phi+Random(-0.00005,0.00005);
    	particles[kstate++][part]=(*It)->rho+Random(-1./0.19,20);
      //}
    }
}
double CParticleFilter::Random(double min, double max)
{
        double range= max-min;
        return (range*rand()/RAND_MAX)+min;
}
void CParticleFilter::initState()
{

  //fixme esto puede tener otro orden
  UpdateMatrixSize();

  for (int part=0; part<num_particles;part++)
  {
    initParticle(part);
  }
     for(int i=0;i<6;i++) var_modelo[i]=0.0001;
     for(int i=6;i<12;i++) var_modelo[i]=0.00001;
     for(int i=12;i<num_max_variables;i++) var_modelo[i]=0.0;
}
void CParticleFilter::setModel(CModel *p)
{
    cout<<"start set model"<< endl;
  pModel = p;

  for (int i =0;i<num_max_variables;i++)
      for(int j=0;j<num_max_variables;j++){
          if (i==j) A[i][j]=1;
          else A[i][j]=0;
      }
  for (int i =0; i<pModel->getStateNum();i++)
      for (int j=0; j<pModel->getStateNum();j++){
          A[i][j]=cvmGet(pModel->getTransitionMatrix(),i,j);
      }

  for (int i=0; i<pModel->getStateNum();i++)
      var_modelo[i]=cvmGet(pModel->getProcessNoiseCov(),i,i);

  if(pModel->getMeasurementNum()>0){
      /** FIXME TODO esto hay que cambiarlo para poder meter medidas de GPS
    cvSetIdentity( pKalman->measurement_matrix,cvRealScalar(0) );///FIXME
      cvSetIdentity( pKalman->measurement_noise_cov, cvRealScalar(MEAS_COV) );**/
      printf ("ERROR no implementado el soporte para medida en vehículo\n");
      exit (0);
  }
  cout<<"end set model" <<endl;
}
void CParticleFilter::noise(double *v, double *s, int l)
{
     int ii=0;
     double p;
     double raiz_2=sqrt(2);
     for (int i =0; i<l; i++){
         ii=0;
         p=rand()/(RAND_MAX + 1.0);
//         cout<<"noise p "<<p<<endl;
         while (ERF[ii]<(2*p-1)&&(ii<200)) ii++;
//         cout<<"noise ERF["<<ii<<"] "<<ERF[ii]<<endl;
         double m = 0;
         v[i]=m+s[i]*raiz_2*((ii-100)/50.0);
//         cout<<"noise s "<<s[i]<<endl;
     }
}
void CParticleFilter::UpdateMatrixSize()
{
  inited_vis=0;
  inited_no_vis=0;

  for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
	It != pMap->bbdd.end(); It++ )
    {
      if ((*It)->state==st_inited){
        inited_vis++;
      }
      if ((*It)->state==st_no_view){
        inited_no_vis++;
      }
    }
  num_measurements=inited_vis*2+pModel->getMeasurementNum();
  num_variables=(inited_no_vis+inited_vis)*fdims+pModel->getStateNum();

}
