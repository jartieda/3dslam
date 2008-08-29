#include "kalman.h"

namespace SLAM{
//covarianzas de las features solo
#define MEAS_COV 1
//process_noise_cov
#define PROC_COV 0.01
//error_cov_pre inicializacion completa. no importante.
#define ERROR_COV 0.0
//error_cov_post y  superior izquierda (camara)
#define MODEL_ERROR_COV 0.000
/** imprime una matriz **/
void printMat(CvMat *m)
{
  cout<<"------------------------------------------------"<<endl;
  if(m!=0){
    for (int i= 0;i <m->rows;i++)
      {
        for(int j =0; j<m->cols;j++)
          {
            cout<<" "<<cvmGet(m,i,j);
          }
        cout<<endl;
      }
  }
  cout<<"------------------------------------------------"<<endl;
}

/** Genera un Kalman de 3000x2000x10 para reservar memoria suficiente para unos 1000 puntos  <br>
  * reserva memoria para un vector de rotación y otro de translación
  **/
CKalman::CKalman()
{
   //fdim = 6;
   trans=cvCreateMat(3,1,CV_32FC1);

   rotation=cvCreateMat(3,1,CV_32FC1);
   pKalmanMem=cvCreateKalman(3000,2000,10);
   pKalman=new CvKalman;
   xi[1]=0.015790;
   xi[2]=0.210720;
   xi[3]=0.584370;
   xi[4]=1.063620;
   xi[5]=1.610310;
   xi[6]=2.204130;
   xi[7]=2.833110;
   xi[8]=3.489540;
   xi[9]=4.168160;
   xi[10]=4.865180;
   xi[11]=5.577780;
   xi[12]=6.303800;
   xi[13]=7.041500;
   xi[14]=7.789530;
   xi[15]=8.546760;
   xi[16]=9.312240;
   xi[17]=10.085190;
   xi[18]=10.864940;
   xi[19]=11.650910;
   xi[20]=12.442610;
   xi[21]=13.239600;
   xi[22]=14.041490;
   xi[23]=14.847960;
   xi[24]=15.658680;
   xi[25]=16.473410;
   xi[26]=17.291880;
   xi[27]=18.113900;
   xi[28]=18.939240;
   xi[29]=19.767740;
   xi[30]=0.1;//20.599230;
   xi[0]=0.000000;
    pKalman->measurement_matrix=0;
    pKalman->measurement_noise_cov=0;
    pKalman->gain=0;
}

CKalman::~CKalman()
{

}
/**
 * Etapa de predicción del filtro de calman.
 * Es la mísma función que la de opencv
 *  x'(k) = A*x(k)  <br>
 *  x'(k) = x'(k) + B*u(k) <br>
 *  temp1 = A*P(k)
 *  P'(k) = temp1*At + Q
 * TODO revisar esta función para comprobar su eficiencia.
 **/
void CKalman::Predict()
{
     //FIXME cambiar para anadir el control
  CvMat* control=0;
  /* update the state */
  /* x'(k) = A*x(k) */

  cvMatMulAdd( pKalman->transition_matrix, pKalman->state_post, 0, pKalman->state_pre );

  if( control && pKalman->CP > 0 )
    {
      /* x'(k) = x'(k) + B*u(k) */
      cvMatMulAdd( pKalman->control_matrix, control, pKalman->state_pre, pKalman->state_pre );
    }
  /* update error covariance matrices */

  /* temp1 = A*P(k) */
  cvMatMulAdd( pKalman->transition_matrix, pKalman->error_cov_post, 0, pKalman->temp1 );

  /* P'(k) = temp1*At + Q */

  cvGEMM( pKalman->temp1, pKalman->transition_matrix, 1, pKalman->process_noise_cov, 1,
                   pKalman->error_cov_pre, CV_GEMM_B_T );

}
void CKalman::Predict_FAST()
{
  //FIXME cambiar para anadir el control
  CvMat* control=0;
  /* update the state */
  /* x'(k) = A*x(k) */
  CvMat *A_, *post, *pre;
  A_=cvCreateMatHeader(pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  post=cvCreateMatHeader(pModel->getStateNum(),1,CV_32FC1);
  pre=cvCreateMatHeader(pModel->getStateNum(),1,CV_32FC1);

  cvGetSubRect( pKalman->transition_matrix,A_,cvRect(0,0,pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->state_post,post,cvRect(0,0,1,pModel->getStateNum()));
  cvGetSubRect( pKalman->state_pre,pre,cvRect(0,0,1,pModel->getStateNum()));

  cout<<"he llegado a en medio"<<endl;
  //  cvMatMulAdd( pKalman->transition_matrix, pKalman->state_post, 0, pKalman->state_pre );
  cvCopy(pKalman->state_post,pKalman->state_pre);
  cvMatMul( A_, post, pre);
  cout<<"he llegado a en medio2"<<endl;
  cvReleaseMatHeader(&A_);
  cvReleaseMatHeader(&post);
  cvReleaseMatHeader(&pre);

  if( control && pKalman->CP > 0 )
    {
      /* x'(k) = x'(k) + B*u(k) */
      cvMatMulAdd( pKalman->control_matrix, control, pKalman->state_pre, pKalman->state_pre );
    }
  /* update error covariance matrices */

  /* temp1 = A*P(k) */
  CvMat  *P_ ,*temp1_,*Q_,*P_pre,*P_21,*P_12,*P_21_pre,*P_12_pre;
  A_=cvCreateMatHeader(pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  P_=cvCreateMatHeader(pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  temp1_=cvCreateMatHeader(pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  Q_=cvCreateMatHeader(pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  P_pre=cvCreateMatHeader(pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  P_21=cvCreateMatHeader(pModel->getStateNum(),pKalman->DP-pModel->getStateNum(),CV_32FC1);
  P_12=cvCreateMatHeader(pKalman->DP-pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);
  P_21_pre=cvCreateMatHeader(pModel->getStateNum(),pKalman->DP-pModel->getStateNum(),CV_32FC1);
  P_12_pre=cvCreateMatHeader(pKalman->DP-pModel->getStateNum(),pModel->getStateNum(),CV_32FC1);

  cvGetSubRect( pKalman->transition_matrix,A_,cvRect(0,0,pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->error_cov_post,P_,cvRect(0,0,pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->process_noise_cov,Q_,cvRect(0,0,pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->temp1,temp1_,cvRect(0,0,pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->error_cov_pre,P_pre,cvRect(0,0,pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->error_cov_post,P_21,cvRect(pModel->getStateNum(),0,pKalman->DP-pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->error_cov_post,P_12,cvRect(0,pModel->getStateNum(),pModel->getStateNum(),pKalman->DP-pModel->getStateNum()));
  cvGetSubRect( pKalman->error_cov_pre,P_21_pre,cvRect(pModel->getStateNum(),0,pKalman->DP-pModel->getStateNum(),pModel->getStateNum()));
  cvGetSubRect( pKalman->error_cov_pre,P_12_pre,cvRect(0,pModel->getStateNum(),pModel->getStateNum(),pKalman->DP-pModel->getStateNum()));

  cout<<"he llegado a en medio3"<<endl;
  //cvMatMulAdd( pKalman->transition_matrix, pKalman->error_cov_post, 0, pKalman->temp1 );
  cvMatMulAdd( A_, P_, 0,temp1_ );
  cvMatMul(A_,P_21,P_21_pre);
  cvGEMM(P_12,A_,1,NULL,0,P_12_pre,CV_GEMM_B_T);

  /* P'(k) = temp1*At + Q */

  //cvGEMM( pKalman->temp1, pKalman->transition_matrix, 1, pKalman->process_noise_cov, 1,
  //                 pKalman->error_cov_pre, CV_GEMM_B_T );
  cvGEMM( temp1_,A_, 1, Q_, 1, P_pre, CV_GEMM_B_T );

  cvReleaseMatHeader(&A_);
  cvReleaseMatHeader(&P_);
  cvReleaseMatHeader(&Q_);
  cvReleaseMatHeader(&temp1_);
  cvReleaseMatHeader(&P_pre);
  cvReleaseMatHeader(&P_21);
  cvReleaseMatHeader(&P_12);
  cvReleaseMatHeader(&P_21_pre);
  cvReleaseMatHeader(&P_12_pre);

}

/**
 * inicializa el estado del filtro de kalman <br>
 * usa las 12 primeras variables para almacenar los parámetros de translación y rotación de pDataCam <br>
 * despues llama a UdateMatrixSize para ajustar los tamanos de las matrices.<br>
 * despues rellena el resto del vector de estado con las coordenadas xyz de los puntos de la base de datos
 **/
void CKalman::initState()
{
  int kstate=0;
  //fixme esto puede tener otro orden
  for(int i =0; i<3;i++){
    cvmSet(pKalman->state_post,kstate++,0,cvmGet(pDataCam->translation,i,0));
    cvmSet(pKalman->state_post,kstate++,0,cvmGet(pDataCam->translation,i,0));
  }
  for(int i =0; i<3;i++){
    cvmSet(pKalman->state_post,kstate++,0,cvmGet(pDataCam->rotation,i,0));
    cvmSet(pKalman->state_post,kstate++,0,cvmGet(pDataCam->rotation,i,0));
  }
  cout<<"Before updateMatrizSize init_state"<<endl;

  UpdateMatrixSize();

  //grow(pMap->bbdd.size()-10);
  cout<<"after updateMatrizSize " <<endl;

  // inicializaciÃ³n primeros puntos vistos
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
        cvmSet(pKalman->state_post,kstate++,0,(*It)->wx);
        cvmSet(pKalman->state_post,kstate++,0,(*It)->wy);
        cvmSet(pKalman->state_post,kstate++,0,(*It)->wz);
        cvmSet(pKalman->state_post,kstate++,0,(*It)->theta);
        cvmSet(pKalman->state_post,kstate++,0,(*It)->phi);
        cvmSet(pKalman->state_post,kstate++,0,(*It)->rho);
      }
    }
}

/**
 * rellena el valor dle puntero con el modelo que se usara del movimiento de la cámara.<br>
 * des pues llama a SetKalman para actualizar los valores de las matrices con los proporcionados con la clase pModel.<br>
 * @param p clase cmodel
 **/
void CKalman::setModel(CModel *p)
{

  pModel = p;

  cout<<"getStateNum"<<pModel->getStateNum()<<endl;
  cout<<"getMeasurementNum"<<pModel->getMeasurementNum()<<endl;
  //fdims=6;//FIXME WARNING ERROR
  SetKalman(pKalman,pModel->getStateNum(),
	    pModel->getMeasurementNum(),
	    pModel->getInputNum());

  cout<<"mp "<<pKalman->MP<<" dp "<<pKalman->DP<<endl;
  if(pModel->getMeasurementNum()>0){
    measurement=cvCreateMat(pModel->getMeasurementNum(),1,CV_32FC1);
  }
  //	dim_measurement=2*10+pModel->getMeasurementNum();
  //	dim_state=3*10+pModel->getStateNum();
  cvSetIdentity( pKalman->transition_matrix);//, cvRealScalar(pptosBBDD->bbdd.size()*3+12) );
  transMat(pModel->getTransitionMatrix(),pKalman->transition_matrix);

  cvSetIdentity( pKalman->process_noise_cov, cvRealScalar(PROC_COV) );
  transMat(pModel->getProcessNoiseCov(),pKalman->process_noise_cov);

  if(pKalman->MP>0){
    cvSetIdentity( pKalman->measurement_matrix,cvRealScalar(0) );///FIXME
      cvSetIdentity( pKalman->measurement_noise_cov, cvRealScalar(MEAS_COV) );
  }
  if(pModel->getMeasurementNum()!=0)
    {
      transMat(pModel->getMeasurementMatrix(),pKalman->measurement_matrix );
      transMat(pModel->getMeasurementNoiseCov(),pKalman->measurement_noise_cov );
    }
  //covarianza de la medida

  cvSetIdentity( pKalman->error_cov_post, cvRealScalar(MODEL_ERROR_COV));
  cvSetIdentity( pKalman->error_cov_pre, cvRealScalar(MODEL_ERROR_COV));
  //cvSetIdentity( pKalman->state_post, cvRealScalar(0));
  cout<<"end set param kalman"<<endl;

}

/** Mahalanobis test **/
void CKalman::Test()
{

  UpdateJacob();

  CvMat* innov;
  CvMat* C;
  CvMat* H;
  CvMat* P;
  CvMat* R;
  CvMat* T2;
  CvMat* T3;
  CvMat* T4;

  innov=cvCreateMat(2,1,CV_32FC1);
  C=cvCreateMat(2,2,CV_32FC1);
  H=cvCreateMat(2,12+fdims,CV_32FC1);
  P=cvCreateMat(12+fdims,12+fdims,CV_32FC1);
  R=cvCreateMat(2,2,CV_32FC1);
  T2=cvCreateMat(2,12+fdims,CV_32FC1);
  T3=cvCreateMat(2,1,CV_32FC1);
  T4=cvCreateMat(1,1,CV_32FC1);

  int m_i=pModel->getMeasurementNum();
  int s_i=12;//
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){

        for(int col=0;col<12;col++)
	  {
     	    cvmSet(H,0,col,cvmGet(pKalman->measurement_matrix,m_i,col));
            cvmSet(H,1,col,cvmGet(pKalman->measurement_matrix,m_i+1,col));//FIXME COMPROBAR
	  }
        for (int d=0;d<fdims;d++){
	  cvmSet(H,0,12+d,cvmGet(pKalman->measurement_matrix,m_i,s_i+d));
        }
        //cvmSet(H,0,13,cvmGet(pKalman->measurement_matrix,m_i,s_i+1));
        //cvmSet(H,0,14,cvmGet(pKalman->measurement_matrix,m_i,s_i+2));
        for (int d=0;d<fdims;d++){
	  cvmSet(H,1,12+d,cvmGet(pKalman->measurement_matrix,m_i+1,s_i+d));
        }

//    cvmSet(H,1,12,cvmGet(pKalman->measurement_matrix,m_i+1,s_i+0));
//    cvmSet(H,1,13,cvmGet(pKalman->measurement_matrix,m_i+1,s_i+1));
//    cvmSet(H,1,14,cvmGet(pKalman->measurement_matrix,m_i+1,s_i+2));

     for (int col=0;col<12;col++)
	  for (int row=0; row<12;row++)
	    {
	      cvmSet(P,row,col,cvmGet(pKalman->error_cov_pre,row,col));
	    }

     for (int col=0;col<fdims;col++)
	  for (int row=0; row<fdims;row++)
	    {
	      cvmSet(P,row+12,col+12,cvmGet(pKalman->error_cov_pre,s_i+row,s_i+col));
	    }
    for (int col=0;col<12;col++)
	  for (int row=0; row<fdims;row++)
	    {
	      cvmSet(P,row+12,col,cvmGet(pKalman->error_cov_pre,s_i+row,col));
	    }
    for (int col=0;col<fdims;col++)
	  for (int row=0; row<12;row++)
	    {
	      cvmSet(P,row,col+12,cvmGet(pKalman->error_cov_pre,row,col+s_i));
	    }
    cvSetIdentity(R,cvRealScalar(MEAS_COV));

    /* temp2 = H*P'(k) */
    cvMatMulAdd( H,P,0,T2 );
    /* C = temp2*Ht + R */
    cvGEMM( T2,H, 1,R, 1, C, CV_GEMM_B_T );

        /* temp5 = z(k) - H*x'(k) */
	cvmSet(innov,0,0,(*It)->pto.x-(*It)->projx);
	cvmSet(innov,1,0,(*It)->pto.y-(*It)->projy);

        /* temp3=C^-1 * (z-h)*/
	cvSolve( C,innov,T3, CV_SVD );
        /* temp2 = (z-h) temp3t */
	cvGEMM(innov,T3,1,NULL,0,  T4 ,CV_GEMM_A_T);

    	if (cvmGet(T4,0,0)>xi[30])
	  {
	    cout<<"test ko: "<<cvmGet(T4,0,0)<<" xi: "<<xi[30]<<" id "<<(*It)->ID<<endl;
	    (*It)->state=st_no_view;
	    pMap->visible--;
	  }else{
	  cout<<"test OK: "<<cvmGet(T4,0,0)<<" xi: "<<xi[30]<<" id "<<(*It)->ID<<endl;
    	}

        m_i+=2;
        s_i+=fdims;

      }//end if inited


    }//end for iterator

  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
        if(sqrt(((*It)->pto.x-(*It)->projx)*((*It)->pto.x-(*It)->projx)+
            ((*It)->pto.y-(*It)->projy)*((*It)->pto.y-(*It)->projy))>150)
          {
            cout<<"fuera por distancia a proj: "<<(*It)->ID<<" Dist: "<<sqrt(((*It)->pto.x-(*It)->projx)*((*It)->pto.x-(*It)->projx)+
                ((*It)->pto.y-(*It)->projy)*((*It)->pto.y-(*It)->projy))<<endl;
            (*It)->state=st_no_view;
            pMap->visible--;
        }//end if for
      }//end if state
    }//end iterator
  cvReleaseMat(&innov);
  cvReleaseMat(&C);
  cvReleaseMat(&H);
  cvReleaseMat(&P);
  cvReleaseMat(&R);
  cvReleaseMat(&T2);
  cvReleaseMat(&T3);
  cvReleaseMat(&T4);

}


void CKalman::UpdateJacob()
{

  int kstate=0;

  cout<<"antes trans "<<endl;
  for(int i =0; i<3;i++){
    cvmSet(trans,i,0,cvmGet(pKalman->state_pre,kstate,0));
    kstate++;
    kstate++;
  }

  cout<<"antes rot "<<endl;
  for(int i =0; i<3;i++){
    cvmSet(rotation,i,0,cvmGet(pKalman->state_pre,kstate,0)+0.0001);
    kstate++;
    kstate++;
  }

  int i =0;

  pDataCam->SetRotation(rotation);
  pDataCam->SetTranslation(trans);
  i=0;
  pModelCam->ProjectPoints();

  cout<<"pmodel->getMeasNum()"<<pModel->getMeasurementNum()<<endl;
  int j=0;

  cvSetIdentity(pKalman->measurement_matrix,cvRealScalar(0));
  if(pModel->getMeasurementNum()){
    transMat(pModel->getMeasurementMatrix(),pKalman->measurement_matrix);
  }

  for (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
	//cout<<j<<" "<<(*It)->wx<<" "<<(*It)->wy<<" "<<(*It)->wz<<" "<<(*It)->projx<<" "<<(*It)->projy<<" "<<cvmGet((*It)->dpdw,0,0)<<endl;
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),0,cvmGet((*It)->dpdt,0,0));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),2,cvmGet((*It)->dpdt,0,1));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),4,cvmGet((*It)->dpdt,0,2));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),6,cvmGet((*It)->dpdr,0,0));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),8,cvmGet((*It)->dpdr,0,1));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),10,cvmGet((*It)->dpdr,0,2));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum(),cvmGet((*It)->dpdw,0,0));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+1,cvmGet((*It)->dpdw,0,1));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+2,cvmGet((*It)->dpdw,0,2));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+3,cvmGet((*It)->dpdw,0,3));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+4,cvmGet((*It)->dpdw,0,4));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+5,cvmGet((*It)->dpdw,0,5));
	i++;

	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),0,cvmGet((*It)->dpdt,1,0));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),2,cvmGet((*It)->dpdt,1,1));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),4,cvmGet((*It)->dpdt,1,2));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),6,cvmGet((*It)->dpdr,1,0));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),8,cvmGet((*It)->dpdr,1,1));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),10,cvmGet((*It)->dpdr,1,2));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum(),cvmGet((*It)->dpdw,1,0));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+1,cvmGet((*It)->dpdw,1,1));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+2,cvmGet((*It)->dpdw,1,2));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+3,cvmGet((*It)->dpdw,1,3));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+4,cvmGet((*It)->dpdw,1,4));
	cvmSet(pKalman->measurement_matrix,i+pModel->getMeasurementNum(),j+pModel->getStateNum()+5,cvmGet((*It)->dpdw,1,5));
	i++;
	j+=fdims;

      }//end if inited
      else if((*It)->state==st_no_view){
	j+=fdims;

      }
    }

}
/**
 * etapa de corrección del filtro de kalman <br>
 * en esta etapa se proyectan todos los puntos de la base de datos y se halla la jacobiana del modelo del sensor <br>
 *  temp2 = H*P'(k) <br>
 *  temp3 = temp2*Ht + R <br>
 *  temp4 = inv(temp3)*temp2 = Kt(k) <br>
 *  K(k^t)
 *  temp5 = z(k) - H*x'(k)
 *  x(k) = x'(k) + K(k)*temp5
 *  P(k) = P'(k) - K(k)*temp2
 **/

void CKalman::Correct()
{

  UpdateJacob();
  cout<<"correction"<<endl;

//  cout<<"1"<<endl;
  /* temp2 = H*P'(k) */
  cvMatMulAdd( pKalman->measurement_matrix,
	       pKalman->error_cov_pre, 0, pKalman->temp2 );
  /* temp3 = temp2*Ht + R */
     cvGEMM( pKalman->temp2, pKalman->measurement_matrix, 1,
	pKalman->measurement_noise_cov, 1, pKalman->temp3, CV_GEMM_B_T );

    /* temp4 = inv(temp3)*temp2 = Kt(k) */
  cvSolve(pKalman->temp3, pKalman->temp2, pKalman->temp4, CV_SVD );
//  cout<<"8"<<endl;
  /* K(k) */
  cvTranspose( pKalman->temp4, pKalman->gain );

  /* temp5 = z(k) - H*x'(k) */

  if(pModel->getMeasurementNum()!=0)
    {
      CvMat *tempmeas;
      tempmeas=pModel->getMeasurementVector();
      for(int kk=0;kk<pModel->getMeasurementNum();kk++)
        {
          cvmSet(measurement,kk,0,cvmGet(tempmeas,kk,0));
        }
          //		cvReleaseMat(&tempmeas);
    }
  int ii=pModel->getMeasurementNum();
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
        cvmSet(measurement,ii,0,(*It)->pto.x);
        ii++;
        cvmSet(measurement,ii,0,(*It)->pto.y);
        ii++;
      }
    }

  //FIXME esta parte se ejecuta pero solo tiene efecto cuando hay medida de movimiento de camara.
  if(pModel->getMeasurementNum()!=0)
    {
      cvGEMM( pKalman->measurement_matrix, pKalman->state_pre, -1, measurement, 1, pKalman->temp5 );
    }

  ii=pModel->getMeasurementNum();
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
        cout<<"linearidad--> "<<" Jacob x:"<< cvmGet(pKalman->temp5,ii,0)<<" y: ";
        cout<<cvmGet(pKalman->temp5,ii+1,0)<< " modelo x: "<<(*It)->pto.x-(*It)->projx;
        cout<<" y: "<<(*It)->pto.y-(*It)->projy<<endl;
        cvmSet(pKalman->temp5,ii,0,(*It)->pto.x-(*It)->projx);
        ii++;
        cvmSet(pKalman->temp5,ii,0,(*It)->pto.y-(*It)->projy);
        ii++;
      }
    }

  /* x(k) = x'(k) + K(k)*temp5 */
  cvMatMulAdd( pKalman->gain, pKalman->temp5, pKalman->state_pre, pKalman->state_post );

  /* P(k) = P'(k) - K(k)*temp2 */
  cvGEMM( pKalman->gain, pKalman->temp2, -1, pKalman->error_cov_pre, 1,
	  pKalman->error_cov_post, 0 );

  int kstate=0;
  for(int i =0; i<3;i++){
    cvmSet(trans,i,0,cvmGet(pKalman->state_post,kstate++,0));
    kstate++;
  }
  for(int i =0; i<3;i++){
    cvmSet(rotation,i,0,cvmGet(pKalman->state_post,kstate++,0));
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
          (*It)->wx=cvmGet(pKalman->state_post,ii,0);
          (*It)->wx_s=cvmGet(pKalman->error_cov_post,ii,ii);
          ii++;
          (*It)->wy=cvmGet(pKalman->state_post,ii,0);
          (*It)->wy_s=cvmGet(pKalman->error_cov_post,ii,ii);
          ii++;
          (*It)->wz=cvmGet(pKalman->state_post,ii,0);
          (*It)->wz_s=cvmGet(pKalman->error_cov_post,ii,ii);
          ii++;
          (*It)->theta=cvmGet(pKalman->state_post,ii,0);
          ii++;
          (*It)->phi=cvmGet(pKalman->state_post,ii,0);
          ii++;
          (*It)->rho=cvmGet(pKalman->state_post,ii,0);
          ii++;
        }
    }


}
void CKalman::Correct_FAST()
{

  UpdateJacob();
  cout<<"correction"<<endl;
  CvMat *Paa, *Haa;
  Paa = cvCreateMat((pKalman->MP/2)*fdims+pModel->getStateNum(),
		    (pKalman->MP/2)*fdims+pModel->getStateNum(),CV_32FC1);
  Haa = cvCreateMat(pKalman->MP,(pKalman->MP/2)*fdims+pModel->getStateNum(),CV_32FC1);//num_medidas/2*6
  int i_aa=0;
  int j_aa=0;
  bool cols[pKalman->DP];
  int i_c=0;
//  cout<<"1"<<endl;
/** genero un array que contiene las columnas que se ven(1) o que no se ven(0)**/
  for (int j=0 ; j<pModel->getStateNum();j++)
    {
      cols[i_c]=1;
      i_c++;
    }
//  cout<<"2"<<endl;
  for (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
	for (int j=0;j<fdims; j++)
	  {
	    cols[i_c]=1;
	    i_c++;
	  }
      }else if((*It)->state==st_no_view)
	{
	  for (int j=0;j<fdims;j++)
	    {
	      cols[i_c]=0;
	      i_c++;
	    }
	}

    }
//  cout<<"3"<<endl;
/** genero una submatriz de medida con las columnas que son **/
  for (int j=0;j< i_c; j++)
    {
      if (cols[j]==1)
        {
	  for (int i=0; i<pKalman->MP;i++)
	    {
	      cvmSet(Haa,i,i_aa,cvmGet(pKalman->measurement_matrix,i,j));
	    }

	  i_aa++;
        }
    }
//  cout<<"4"<<endl;
  i_aa=0;
  j_aa=0;
  for (int i=0; i<i_c; i++)
    {
      if(cols[i]==1)
        {
	  j_aa=0;
	  for (int j=0; j<i_c; j++)
            {
	      if(cols[j]==1)
                {
		  cvmSet(Paa,i_aa,j_aa,cvmGet(pKalman->error_cov_pre,i,j));
		  j_aa++;
                }
            }
	  i_aa++;
        }
    }
//  cout<<"5"<<endl;
  /* t2 = Haa*Paa'(k) */
  CvMat *t2;
  t2=cvCreateMat(pKalman->MP,(pKalman->MP/2)*fdims+pModel->getStateNum(),CV_32FC1);//num_medidas/2*6
  cvMatMulAdd(Haa,Paa,0,t2);
//  cout<<"6"<<endl;
  /* t3 = t2*Haa^t + R = Haa*Paa*Haa^t + R*/
  CvMat *t3;
  t3=cvCreateMat(pKalman->MP,pKalman->MP,CV_32FC1);
  cvGEMM(t2,Haa,1,pKalman->measurement_noise_cov,1,t3,CV_GEMM_B_T);
  //cout<<"7"<<endl;
  /* temp2 = H*P'(k) */
  cvMatMulAdd( pKalman->measurement_matrix,
	       pKalman->error_cov_pre, 0, pKalman->temp2 );
  /* temp3 = temp2*Ht + R */
  /*    cvGEMM( pKalman->temp2, pKalman->measurement_matrix, 1,
	pKalman->measurement_noise_cov, 1, pKalman->temp3, CV_GEMM_B_T );*/

    /* temp4 = inv(temp3)*temp2 = Kt(k) */
  cvSolve( t3, pKalman->temp2, pKalman->temp4, CV_SVD );
//  cout<<"8"<<endl;
  /* K(k) */
  cvTranspose( pKalman->temp4, pKalman->gain );

  /* temp5 = z(k) - H*x'(k) */

  if(pModel->getMeasurementNum()!=0)
    {
      CvMat *tempmeas;
      tempmeas=pModel->getMeasurementVector();
      for(int kk=0;kk<pModel->getMeasurementNum();kk++)
	{
	  cvmSet(measurement,kk,0,cvmGet(tempmeas,kk,0));
	}
      //		cvReleaseMat(&tempmeas);
    }
  int ii=pModel->getMeasurementNum();
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
	cvmSet(measurement,ii,0,(*It)->pto.x);
	ii++;
	cvmSet(measurement,ii,0,(*It)->pto.y);
	ii++;
      }
    }

  //FIXME esta parte se ejecuta pero solo tiene efecto cuando hay medida de movimiento de camara.
  if(pModel->getMeasurementNum()!=0)
    {
      cvGEMM( pKalman->measurement_matrix, pKalman->state_pre, -1, measurement, 1, pKalman->temp5 );
    }

  ii=pModel->getMeasurementNum();
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
	cvmSet(pKalman->temp5,ii,0,(*It)->pto.x-(*It)->projx);
	ii++;
	cvmSet(pKalman->temp5,ii,0,(*It)->pto.y-(*It)->projy);
	ii++;
      }
    }
  cout<<"pto.x - projx "<<endl;
  for (int i=0 ; i<(pKalman->MP); i++)
    {
      for (int j=0;j<1;j++)
	{
	  cout << cvmGet(pKalman->temp5, i,j)<<" ";
	}
      cout <<endl;
    }

  /*cout<<"measurement: "<<endl;
    for (int i=0; i<(pKalman->MP);i++)
    {
    cout<<" "<<cvmGet(measurement,i,0);
    }
    cout<<endl;*/

  /* x(k) = x'(k) + K(k)*temp5 */
  cvMatMulAdd( pKalman->gain, pKalman->temp5, pKalman->state_pre, pKalman->state_post );

  /* P(k) = P'(k) - K(k)*temp2 */
  cvGEMM( pKalman->gain, pKalman->temp2, -1, pKalman->error_cov_pre, 1,
	  pKalman->error_cov_post, 0 );


  int kstate=0;
  for(int i =0; i<3;i++){
    cvmSet(trans,i,0,cvmGet(pKalman->state_post,kstate++,0));
    kstate++;
  }
  for(int i =0; i<3;i++){
    cvmSet(rotation,i,0,cvmGet(pKalman->state_post,kstate++,0));
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
	  (*It)->wx=cvmGet(pKalman->state_post,ii,0);
	  (*It)->wx_s=cvmGet(pKalman->error_cov_post,ii,ii);
	  ii++;
	  (*It)->wy=cvmGet(pKalman->state_post,ii,0);
	  (*It)->wy_s=cvmGet(pKalman->error_cov_post,ii,ii);
	  ii++;
	  (*It)->wz=cvmGet(pKalman->state_post,ii,0);
	  (*It)->wz_s=cvmGet(pKalman->error_cov_post,ii,ii);
	  ii++;
	  (*It)->theta=cvmGet(pKalman->state_post,ii,0);
	  ii++;
	  (*It)->phi=cvmGet(pKalman->state_post,ii,0);
	  ii++;
	  (*It)->rho=cvmGet(pKalman->state_post,ii,0);
	  ii++;
	}
    }
  cvReleaseMat(&Paa);
  cvReleaseMat(&Haa);
  cvReleaseMat(&t2);
  cvReleaseMat(&t3);

  //    cvReleaseMat(&rotation);
  //    cvReleaseMat(&trans);

}
/**
 * actualiza las matrizes del filtro de kalman para aceptar otro numero de entradas
 *
 **/
void CKalman::UpdateMatrixSize()
{

  int inited_vis=0;
  int inited_no_vis=0;

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

  int n=inited_vis*2+pModel->getMeasurementNum();
  int s=(inited_no_vis+inited_vis)*fdims+pModel->getStateNum();
  int oldDP,oldMP,oldCP;
  if (pKalman!=0){
    oldDP=pKalman->DP;
    oldMP=pKalman->MP;
    oldCP=pKalman->CP;
  }else {
    oldDP=0;//FIXME Quizas sea get estate num
    oldCP=0;
    oldMP=0;
  }
  //	int npoints=inited_vis-((oldDP-pModel->getStateNum())/fdims);
  int npoints=(s-oldDP)/fdims;/// numero de puntos nuevos

  SetKalman(pKalman,s,n,oldCP);

  cout<<"updateMatrixSize("<<n<<") "<<npoints<<endl;

  cvSetIdentity( pKalman->measurement_noise_cov, cvRealScalar(MEAS_COV) );
  if(pModel->getMeasurementNum()>0)
    {
	    transMat(pModel->getMeasurementNoiseCov(),pKalman->measurement_noise_cov );
    }

  pKalman->MP=n;
  cvReleaseMat(&measurement);
  measurement=cvCreateMat( n,1,CV_32FC1);

  //anada datos nuevo parametro
  //FIXME COMPROBAR LO DE NUMERO DE PUNTOS >0 SE PUEDE REDUCIR LA MEDIDA Y QUE NO ME ENTER
  if (npoints>0){
    //transition_matrix
    //cout<<"0"<<endl;
    /** Se inicializa el modelo con una matriz identidad (diagonal 1s) **/
    CvMat *submat;
    submat=cvCreateMatHeader(fdims*npoints,fdims*npoints,CV_32FC1 );
    if(oldDP==0)
    {
//	cout<<"debate 2 oldDP "<<oldDP<<" getmeasnum "<<pModel->getStateNum()<<endl;
//      cvGetSubRect(pKalman->transition_matrix, submat,
//	cvRect(pModel->getStateNum(),pModel->getStateNum(),fdims*npoints,fdims*npoints));
    }else{
        cout<<"debate 1 oldDP "<<oldDP<<" getmeasnum "<<pModel->getMeasurementNum()<<endl;
        cvGetSubRect(pKalman->transition_matrix, submat,
                    cvRect(oldDP,oldDP,fdims*npoints,fdims*npoints));
        cvSetIdentity(submat,cvRealScalar(1));
        cvReleaseMatHeader(&submat);
    }
    //  cout<<"1"<<endl;

    //measurement_matrix
    /** Se anula esta matriz y se inicializa solo la parte correspondiente al sensor.
    	Esta matriz se debe actualizar en cada paso y se hace en la etapa de corrección
    	llamando a la función UpdateJacob()
    **/
    cvSetIdentity(pKalman->measurement_matrix,cvRealScalar(0));
    /** si el modelo del vehículo proporciona medidas se añaden al modelo de medida **/
    if(pModel->getMeasurementNum()){
      transMat(pModel->getMeasurementMatrix(),pKalman->measurement_matrix);
    }

//    cout<<"2"<<endl;

    /** Covarianza del error del modelo del proceso se inicializac a una diagonal PROC_COV **/
    //cvSetIdentity(pKalman->process_noise_cov,cvRealScalar(PROC_COV));

    submat=cvCreateMatHeader(fdims*npoints,fdims*npoints,CV_32FC1 );
    cvGetSubRect(pKalman->process_noise_cov, submat, cvRect(oldDP,oldDP,fdims*npoints,fdims*npoints));
    cvSetIdentity(submat,cvRealScalar(PROC_COV));
    cvReleaseMatHeader(&submat);

//    for (int i =0; i<6; i++) cvmSet(pKalman->process_noise_cov,i,i,0.01);///FIXME NO LO PILLA DEL MODELO
//    for (int i =17;i<fdims*npoints+pModel->getStateNum(); i+=6) cvmSet(pKalman->process_noise_cov,i,i,1);
//    cout<<"3"<<endl;

    /**
     ** Covarianza del estado del proceso a posteriori se inicializa usando valores
     ** propagados por el modelo
     ** FIXME IMPLEMENTACIÓN DEPENDIENTE DE LA PARAMETRIZACIÓN
     **/

    int startpoint=(oldDP-pModel->getStateNum())/fdims;
/*
    submat=cvCreateMatHeader(fdims*npoints,fdims*npoints,CV_32FC1 );
    CvMat *submat2;
    CvMat *submat3;
    CvMat *jPos, *jPix,*jBig;
    CvMat *ptemp;
    CvMat *temp2;
    CvMat *ptemp2;

    submat2=cvCreateMatHeader(fdims*npoints,12,CV_32FC1 );
    submat3=cvCreateMatHeader(12,fdims*npoints,CV_32FC1 );

    cvGetSubRect(pKalman->error_cov_post, submat, cvRect(oldDP,oldDP,fdims*npoints,fdims*npoints));
    cvGetSubRect(pKalman->error_cov_post, submat2, cvRect(0,oldDP,12,fdims*npoints));
    cvGetSubRect(pKalman->error_cov_post, submat3, cvRect(oldDP,0,fdims*npoints,12));

    cvSetIdentity(submat,cvRealScalar(ERROR_COV));

    jPos=cvCreateMat(6,6,CV_32FC1);
    jPix=cvCreateMat(6,2,CV_32FC1);
    jBig=cvCreateMat(12,9,CV_32FC1);
    ptemp=cvCreateMat(9,9,CV_32FC1);
    temp2=cvCreateMat(12,9,CV_32FC1);
    ptemp2=cvCreateMat(12,12,CV_32FC1);

    cvSetIdentity(ptemp,cvRealScalar(0));
    cvSetIdentity(jBig,cvRealScalar(0));


    for(int i=0; i<6; i++)
      for(int j=0; j<6;j++)
        cvmSet(ptemp,i,j,cvmGet(pKalman->error_cov_post,2*i,2*j));

    cvmSet(ptemp,6,6,4);//covarianza del error de localización de un pixel
    cvmSet(ptemp,7,7,4);//covarianza del error de localización de un pixel
    cvmSet(ptemp,8,8,0.5);//covarianza de rho

    int id=0;
    int start=0;
    for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
      {
        if ((*It)->state>=st_inited){
          if (start<startpoint) start++;
          else {
            pModelCam->getJInit(jPos,jPix,(*It)->pto);
            cvSetIdentity(jBig,cvRealScalar(1));

            for(int i=0; i<6; i++)
              for(int j=0; j<6;j++)
                cvmSet(jBig,6+i,j,cvmGet(jPos,i,j));//derivada de feature respecto pos

            for(int i=0; i<6; i++)
              for(int j=0; j<2;j++)
                cvmSet(jBig,6+i,6+j,cvmGet(jPix,i,j));//derivada de feature respecto de pix

            cvmSet(jBig,11,8,1);//derivada de rho respecto de rho

            cvMatMul(jBig,ptemp,temp2);
            cvGEMM(temp2,jBig,1,NULL,0,ptemp2,CV_GEMM_B_T);
            for(int i=0; i<6; i++)
              for(int j=0; j<6;j++){
                cvmSet(submat,id*fdims+i,id*fdims+j,cvmGet(ptemp2,6+i,6+j));
                cvmSet(submat2,id*fdims+i,j*2,cvmGet(ptemp2,6+i,j));
                cvmSet(submat3,i*2,id*fdims+j,cvmGet(ptemp2,i,6+j));
              }
            id++;
          }//end else if star>start
        }//end if >inited
      }//end iter


    cvReleaseMatHeader(&submat);
    cvReleaseMatHeader(&submat2);
    cvReleaseMatHeader(&submat3);
    cvReleaseMat(&jPos);
    cvReleaseMat(&jPix);
    cvReleaseMat(&jBig);
    cvReleaseMat(&ptemp);
    cvReleaseMat(&temp2);
    cvReleaseMat(&ptemp2);*/


    int estado_recorrido=oldDP;
    int start=0;
    CvMat *h=cvCreateMat(3,1,CV_32FC1);
    for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
        if ((*It)->state>=st_inited){
          if (start<startpoint) start++;
          else {
		cvmSet(h,0,0,cos((*It)->theta)*sin((*It)->phi));
   		cvmSet(h,1,0,-sin((*It)->theta));
   		cvmSet(h,2,0,cos((*It)->theta)*cos((*It)->phi));
   		cvNormalize( h,h);

		NewPointCov(estado_recorrido,h,(*It)->pto.x,(*It)->pto.y);
		estado_recorrido+=6;
	  }
        }
    }
    cvReleaseMat(&h);
    //error_cov_pre
    /**Covarianza del estado del proceso a priori se inicializa a matriz identidad.
       Esto es poco importante. **/
    submat=cvCreateMatHeader(fdims*npoints,fdims*npoints,CV_32FC1 );
    cvGetSubRect(pKalman->error_cov_pre, submat, cvRect(oldDP,oldDP,fdims*npoints,fdims*npoints));
    cvSetIdentity(submat,cvRealScalar(ERROR_COV));
    cvReleaseMatHeader(&submat);
//    cout<<"5"<<endl;


  }//end if npoints >0

  //	state_post=wx wy wz
  int kstate=pModel->getStateNum();
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited||(*It)->state==st_no_view){
        cvmSet(pKalman->state_post,kstate,0,(*It)->wx);
        kstate++;
        cvmSet(pKalman->state_post,kstate,0,(*It)->wy);
        kstate++;
        cvmSet(pKalman->state_post,kstate,0,(*It)->wz);
        kstate++;
        cvmSet(pKalman->state_post,kstate,0,(*It)->theta);
        kstate++;
        cvmSet(pKalman->state_post,kstate,0,(*It)->phi);
        kstate++;
        cvmSet(pKalman->state_post,kstate,0,(*It)->rho);
        kstate++;
      }
    }
}

/**
 * copia una matriz en la esquina superior izquierad de una mas grande
 * @param o_mat matriz de origen
 * @param d_mat matriz de destino
 **/
void CKalman::transMat(CvMat* o_mat, CvMat* d_mat)
{
  CvMat *submat;
  submat=cvCreateMatHeader( o_mat->rows,o_mat->cols,CV_32FC1 );
  cvGetSubRect( d_mat, submat, cvRect(0, 0, o_mat->cols, o_mat->rows));
  cvCopy( o_mat, submat );
  cvReleaseMatHeader(&submat);
}


/**
 * Cambia los punteros de la estructura pKalman para asignar mas memoria de la reservada en la estructura pKalmanMem
 **/
void CKalman::SetKalman(CvKalman*pk,int state, int meas, int input)
{

  pKalman->DP = state;
  pKalman->MP = meas;
  pKalman->CP = input;
  cout<<"state "<<state<<" meas "<<meas<<" input "<<input<<endl;


  /*    CV_CALL( kalman->state_pre = cvCreateMat( DP, 1, CV_32FC1 ));
	cvZero( kalman->state_pre );*/
  pKalman->state_pre = cvCreateMatHeader( state, 1, CV_32FC1 );
  cvGetSubRect(pKalmanMem->state_pre,pKalman->state_pre,
	       cvRect(0, 0, 1,state));
  //    CV_CALL( kalman->state_post = cvCreateMat( DP, 1, CV_32FC1 ));
  pKalman->state_post = cvCreateMatHeader( state, 1, CV_32FC1 );
  cvGetSubRect(pKalmanMem->state_post,pKalman->state_post,
	       cvRect(0, 0, 1,state));
  //    CV_CALL( kalman->transition_matrix = cvCreateMat( DP, DP, CV_32FC1 ));
  pKalman->transition_matrix = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(pKalmanMem->transition_matrix,pKalman->transition_matrix,
	       cvRect(0, 0, state,state));
  //    if( CP > 0 )
  //    {
  //        CV_CALL( kalman->control_matrix = cvCreateMat( DP, CP, CV_32FC1 ));
  if(input>0){
    pKalman->control_matrix = cvCreateMatHeader( state,input, CV_32FC1 );
    cvGetSubRect(pKalmanMem->control_matrix,pKalman->control_matrix,
		 cvRect(0, 0, input,state));
  }else{
      pKalman->control_matrix=0;
  }
  //  CV_CALL( kalman->measurement_matrix = cvCreateMat( MP, DP, CV_32FC1 ));
  if (meas>0){
    pKalman->measurement_matrix = cvCreateMatHeader(  meas,state, CV_32FC1 );
    cvGetSubRect(pKalmanMem->measurement_matrix,pKalman->measurement_matrix,
		 cvRect(0, 0, state,meas));
  }
  //    CV_CALL( kalman->process_noise_cov = cvCreateMat( DP, DP, CV_32FC1 ));
  pKalman->process_noise_cov = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(pKalmanMem->process_noise_cov,pKalman->process_noise_cov,
	       cvRect(0, 0, state,state));
  //	    CV_CALL( kalman->measurement_noise_cov = cvCreateMat( MP, MP, CV_32FC1 ));
  if (meas>0){
    pKalman->measurement_noise_cov = cvCreateMatHeader( meas,meas, CV_32FC1 );
    cvGetSubRect(pKalmanMem->measurement_noise_cov,pKalman->measurement_noise_cov,
		 cvRect(0, 0, meas,meas));
  }
  //    	CV_CALL( kalman->error_cov_pre = cvCreateMat( DP, DP, CV_32FC1 ));
  pKalman->error_cov_pre = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(pKalmanMem->error_cov_pre,pKalman->error_cov_pre,
	       cvRect(0, 0, state,state));
  //    	CV_CALL( kalman->error_cov_post = cvCreateMat( DP, DP, CV_32FC1 ));
  pKalman->error_cov_post = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(pKalmanMem->error_cov_post,pKalman->error_cov_post,
	       cvRect(0, 0, state,state));
  //	CV_CALL( kalman->gain = cvCreateMat( DP, MP, CV_32FC1 ));
  if (meas>0){
    pKalman->gain = cvCreateMatHeader(  state,meas, CV_32FC1 );
    cvGetSubRect(pKalmanMem->gain,pKalman->gain,
		 cvRect(0, 0, meas,state));
  }
  //  CV_CALL( kalman->temp1 = cvCreateMat( DP, DP, CV_32FC1 ));
  pKalman->temp1 = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(pKalmanMem->temp1,pKalman->temp1,//fixme
	       cvRect(0, 0, state,state));
  //  CV_CALL( kalman->temp2 = cvCreateMat( MP, DP, CV_32FC1 ));
  if (meas>0){
    pKalman->temp2 = cvCreateMatHeader(  meas,state, CV_32FC1 );
    cvGetSubRect(pKalmanMem->temp2,pKalman->temp2,
		 cvRect(0, 0, state,meas));
  }
  //    CV_CALL( kalman->temp3 = cvCreateMat( MP, MP, CV_32FC1 ));
  if (meas>0){
    pKalman->temp3 = cvCreateMatHeader( meas,meas, CV_32FC1 );
    cvGetSubRect(pKalmanMem->temp3,pKalman->temp3,
		 cvRect(0, 0, meas,meas));
  }
  //    CV_CALL( kalman->temp4 = cvCreateMat( MP, DP, CV_32FC1 ));
  if (meas>0){
    pKalman->temp4 = cvCreateMatHeader( meas,state, CV_32FC1 );
    cvGetSubRect(pKalmanMem->temp4,pKalman->temp4,
		 cvRect(0, 0, state,meas));
  }
  //    CV_CALL( kalman->temp5 = cvCreateMat( MP, 1, CV_32FC1 ));
  if (meas>0){
    pKalman->temp5 = cvCreateMatHeader(meas,1, CV_32FC1 );
    cvGetSubRect(pKalmanMem->temp5,pKalman->temp5,
		 cvRect(0, 0, 1,meas));
  }
}
void CKalman::NewPointCov(int old_state,CvMat *h,int xpix,int ypix)
{
    CvMat *dm_dt,*dm_dr,*dm_dpix;
    double h0=cvmGet(h,0,0);
    double h1=cvmGet(h,1,0);
    double h2=cvmGet(h,2,0);
    dm_dt=cvCreateMat(6,3,CV_32FC1);
    dm_dr=cvCreateMat(6,3,CV_32FC1);
    dm_dpix=cvCreateMat(6,2,CV_32FC1);

    /** Derivada de feature respecto posicion **/
    cvSetIdentity(dm_dt,cvRealScalar(0));
    for (int i =0; i<3;i++) cvmSet(dm_dt,i,i,1);

    /** Derivada de feature respecto rotacion **/
    CvMat *dTh_dh=cvCreateMat(1,3,CV_32FC1);
    double d=h0*h0+h2*h2;
    double k=1/sqrt(1+(-h1/sqrt(d)));
    cvmSet(dTh_dh,0,0,k*h1*h0/sqrt(d*d*d));
    cvmSet(dTh_dh,0,1,k*-1/sqrt(d));
    cvmSet(dTh_dh,0,2,k*h1*h2/sqrt(d*d*d));

    CvMat *dPh_dh=cvCreateMat(1,3,CV_32FC1);
    double k2=1/sqrt(1+(h0/h2)*(h0/h2));
    cvmSet(dPh_dh,0,0,k2/h2);
    cvmSet(dPh_dh,0,1,0);
    cvmSet(dPh_dh,0,2,k2*(-h0)/(h2*h2));

    CvMat *dh_dr=cvCreateMat(3,3,CV_32FC1);
    CvMat *RotMat = cvCreateMat(3,3,CV_32FC1);
    CvMat *RotJ=cvCreateMat(9,3,CV_32FC1);
    cvRodrigues2(pDataCam->rotation, RotMat,RotJ);
    CvMat *RotJ_dr0=cvCreateMatHeader(3,3,CV_32FC1);
    CvMat *RotJ_dr1=cvCreateMatHeader(3,3,CV_32FC1);
    CvMat *RotJ_dr2=cvCreateMatHeader(3,3,CV_32FC1);
    cvGetSubRect(RotJ,RotJ_dr0,cvRect(0, 0, 3,3));
    cvGetSubRect(RotJ,RotJ_dr1,cvRect(0, 3, 3,3));
    cvGetSubRect(RotJ,RotJ_dr2,cvRect(0, 6, 3,3));


    CvMat *InvF =cvCreateMat(3,3,CV_32FC1);
    cvInv(pDataCam->calibration, InvF);
    CvMat *Pix=cvCreateMat(3,1,CV_32FC1);
    cvmSet(Pix,0,0,xpix);
    cvmSet(Pix,1,0,ypix);
    cvmSet(Pix,2,0,1);

    CvMat *temp= cvCreateMat(3,1,CV_32FC1);
    cvGEMM(InvF,Pix,1,NULL,0,temp,0);
    CvMat *dh_dr0=cvCreateMat(3,1,CV_32FC1);
    cvGEMM(RotJ_dr0,temp,1,NULL,0,dh_dr0,0);
    CvMat *dh_dr1=cvCreateMat(3,1,CV_32FC1);
    cvGEMM(RotJ_dr1,temp,1,NULL,0,dh_dr1,0);
    CvMat *dh_dr2=cvCreateMat(3,1,CV_32FC1);
    cvGEMM(RotJ_dr2,temp,1,NULL,0,dh_dr2,0);
    for (int i =0 ; i<3;i++){
        cvmSet(dh_dr,i,0,cvmGet(dh_dr0,i,0));
        cvmSet(dh_dr,i,1,cvmGet(dh_dr1,i,0));
        cvmSet(dh_dr,i,2,cvmGet(dh_dr2,i,0));
    }

    CvMat *dTh_dr = cvCreateMat(1,3,CV_32FC1);
    CvMat *dPh_dr = cvCreateMat(1,3,CV_32FC1);
    cvGEMM(dTh_dh,dh_dr,1,NULL,0,dTh_dr,0);
    cvGEMM(dPh_dh,dh_dr,1,NULL,0,dPh_dr,0);

    /** derivade de feature respecto de pixel visto **/
    CvMat *dh_dpix_ =cvCreateMat(3,3,CV_32FC1);//Aunque es de 3x3 solo se usa las 2 1º cols.
    CvMat *dh_dpix =cvCreateMatHeader(3,2,CV_32FC1);
    cvGetSubRect(dh_dpix_,dh_dpix,cvRect(0, 0, 2,3));//Aunque es de 3x3 solo se usa las 2 1º cols.

    cvGEMM(pDataCam->rotMat,InvF,1,NULL,0,dh_dpix_,CV_GEMM_A_T);

    CvMat *dTh_dpix=cvCreateMat(1,2,CV_32FC1);
    cvGEMM(dTh_dh,dh_dpix,1,NULL,0,dTh_dpix,0);

    CvMat *dPh_dpix=cvCreateMat(1,2,CV_32FC1);
    cvGEMM(dPh_dh,dh_dpix,1,NULL,0,dPh_dpix,0);

    /** construccion de la jacobiana **/
    CvMat *Jacob = cvCreateMat(old_state+fdims,old_state+3,CV_32FC1);
    cvSetIdentity(Jacob,cvRealScalar(0));
    //inserto P_old
    for (int i =0; i<old_state;i++)
	cvmSet(Jacob,i,i,1);//sub-matriz identidad
    //inserto dm_dt
    for (int i =0; i<3;i++)
	for (int j = 0; j<6; j++){
	    cvmSet(Jacob,j+old_state,2*i,cvmGet(dm_dt,j,i));
	}

    //inserto dm_dr
    for (int i =0; i<3;i++){
         cvmSet(Jacob,old_state+3,2*i+6,cvmGet(dTh_dr,0,i));
         cvmSet(Jacob,old_state+4,2*i+6,cvmGet(dPh_dr,0,i));
    }

    //inserto dh_dpix
    for (int i =0; i<2;i++){
        cvmSet(Jacob,old_state+3,old_state+i,cvmGet(dTh_dpix,0,i));
        cvmSet(Jacob,old_state+4,old_state+i,cvmGet(dPh_dpix,0,i));
    }

    //inserto dh_drho=1
    cvmSet(Jacob,old_state+fdims-1,old_state+3-1,1);
    /** matrix varianzas bruto **/
    CvMat *temp_cov =cvCreateMat(old_state+3,old_state+3,CV_32FC1);
    cvSetIdentity(temp_cov,cvRealScalar(0));
    for (int i =0; i<old_state;i++)
 	for(int j =0; j<old_state;j++)
	    cvmSet(temp_cov,j,i,cvmGet(pKalman->error_cov_post,j,i));
    cvmSet(temp_cov,old_state,old_state,4);//error de determinacion de un pixel
    cvmSet(temp_cov,old_state+1,old_state+1,4);//error de determinación de un pixel
    cvmSet(temp_cov,old_state+2,old_state+2,0.3);//sigma rho cuadrado
    CvMat *temp_cov2 =cvCreateMat(old_state+fdims,old_state+3,CV_32FC1);
    cvGEMM(Jacob,temp_cov,1,NULL,0,temp_cov2,0);

    CvMat *state_post=cvCreateMatHeader(old_state+fdims,old_state+fdims,CV_32FC1);
    cvGetSubRect(pKalman->error_cov_post,state_post,cvRect(0, 0, old_state+fdims,old_state+fdims));

    cvGEMM(temp_cov2,Jacob,1,NULL,0,state_post,CV_GEMM_B_T);
    cvReleaseMat(&temp_cov);
    cvReleaseMat(&temp_cov2);
    cvReleaseMat(&Jacob);
    cvReleaseMatHeader(&dh_dpix);
    cvReleaseMat(&dh_dpix_);
    cvReleaseMat(&dPh_dpix);
    cvReleaseMat(&dTh_dpix);
    cvReleaseMat(&dTh_dr);
    cvReleaseMat(&dTh_dr);
    cvReleaseMat(&dh_dr);
    cvReleaseMat(&dPh_dh);
    cvReleaseMat(&dTh_dh);
    cvReleaseMat(&dm_dt);
    cvReleaseMat(&dm_dr);
    cvReleaseMat(&dm_dpix);

}
/** imprime todas all matrices de pKalman **/
void CKalman::Print(int iter)
{
 /* cout<<"--------------------Kalman-----------------------"<<endl;
  cout<<"Estados: "<<pKalman->DP<<" Medidas: "<<pKalman->MP<<" Control: "<<pKalman->DP<<endl;
  cout<<"------------------------------------------------"<<endl;
  cout<<"state_pre"<<endl;
  printMat(pKalman->state_pre);
  cout<<"state_post"<<endl;
  printMat(pKalman->state_post);
  cout<<"transition_matrix"<<endl;
  printMat(pKalman->transition_matrix);
  cout<<"control_matrix"<<endl;
  printMat(pKalman->control_matrix);
  cout<<"measurement_matrix"<<endl;
  printMat(pKalman->measurement_matrix);
  cout<<"process_noise_cov"<<endl;
  printMat(pKalman->process_noise_cov);
  cout<<"measurement_noise_cov"<<endl;
  printMat(pKalman->measurement_noise_cov);
  cout<<"error_cov_pre"<<endl;
  printMat(pKalman->error_cov_pre);
  cout<<"error_cov_post"<<endl;
  printMat(pKalman->error_cov_post);
  cout<<"kalman gain"<<endl;
  printMat(pKalman->gain);*/
  cvNamedWindow( "kalman", 1 );
  IplImage *im;
  IplImage *im2;
  im=cvCreateImageHeader(cvSize(pKalman->error_cov_post->height,pKalman->error_cov_post->width),IPL_DEPTH_32F,1);
  im2=cvCreateImageHeader(cvSize(pKalman->error_cov_post->height,pKalman->error_cov_post->width),IPL_DEPTH_32F,1);
  im2=cvGetImage(pKalman->error_cov_post,im);
  IplImage *im3;
  im3=cvCreateImage(cvSize(pKalman->error_cov_post->height,pKalman->error_cov_post->width),IPL_DEPTH_8U,1);
  cout<<"hola"<<endl;
  cvConvertScale(im2, im3,555);
  cout<<"adios"<<endl;
  char fname[100];
/*  sprintf(fname, "cov%d.tif",iter);
  cvSaveImage(fname,im3);
  cvShowImage("kalman",im3);
  cvNamedWindow( "kalman_cov_pre", 1 );

  im2=cvGetImage(pKalman->error_cov_post,im);
  cvShowImage("kalman_cov_pre",im);
*/
}
}
