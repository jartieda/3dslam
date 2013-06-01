#include "cslam.h"
#include "Openvis3d.h"
#include "OpenCVImageAdapter.h"


CSlam::CSlam(XMLNode* n):numVisibleMin(10),SD(0),MD(0),CD(0), levels(6),hScale(0.5),vScale(0.5),lineWidth(2),first(0)
{
    //ctor
    xMainNode= n;

    ///Load movement Model
    const char * t;
    t=xMainNode->getChildNode("model").getChildNode("SD").getText();
    sscanf(t,"%d",&modelSD);
    std::cout<<"modelSD IS : "<<modelSD<<std::endl;
    t=xMainNode->getChildNode("model").getChildNode("MD").getText();
    sscanf(t,"%d",&modelMD);
    std::cout<<"modelMD IS : "<<modelMD<<std::endl;
    t=xMainNode->getChildNode("model").getChildNode("CD").getText();
    sscanf(t,"%d",&modelCD);
    std::cout<<"modelCD IS : "<<modelMD<<std::endl;
    t=xMainNode->getChildNode("model").getChildNode("avalue").getText();
    sscanf(t,"%lf",&avalue);
    t=xMainNode->getChildNode("model").getChildNode("alpha").getText();
    sscanf(t,"%lf",&alpha);

    ReserveMemory();

    //t=xMainNode->getChildNode("model").getChildNode("A").getText();
    //CvMat *Atemp= String2CvMat((char *)t,modelSD,modelSD);
    //transMat(Atemp, AMem);
    SD=modelSD;
    MD=modelMD;
    CD=modelCD;
//Camera Calibration
    t=xMainNode->getChildNode("Camera").getChildNode("fx").getText();
    float fx;
    sscanf(t,"%f",&fx);
    t=xMainNode->getChildNode("Camera").getChildNode("fy").getText();
    float fy;
    sscanf(t,"%f",&fy);
    t=xMainNode->getChildNode("Camera").getChildNode("cx").getText();
    float cx;
    sscanf(t,"%f",&cx);
    t=xMainNode->getChildNode("Camera").getChildNode("cy").getText();
    float cy;
    sscanf(t,"%f",&cy);
    IntrinsicParam=cvCreateMat(3,3,CV_32FC1);
    cvZero (IntrinsicParam);
    cvmSet(IntrinsicParam,0,0,fx);
    cvmSet(IntrinsicParam,1,1,fy);
    cvmSet(IntrinsicParam,2,2,1);
    cvmSet(IntrinsicParam,0,2,cx);
    cvmSet(IntrinsicParam,1,2,cy);

    t=xMainNode->getChildNode("Camera").getChildNode("width").getText();
    float width;
    sscanf(t,"%f",&width);
    t=xMainNode->getChildNode("Camera").getChildNode("height").getText();
    float height;
    sscanf(t,"%f",&height);
    t=xMainNode->getChildNode("Camera").getChildNode("k1").getText();
    float k1;
    sscanf(t,"%f",&k1);
    t=xMainNode->getChildNode("Camera").getChildNode("k2").getText();
    float k2;
    sscanf(t,"%f",&k2);
    t=xMainNode->getChildNode("Camera").getChildNode("k3").getText();
    float k3;
    sscanf(t,"%f",&k3);
    t=xMainNode->getChildNode("Camera").getChildNode("k4").getText();
    float k4;
    sscanf(t,"%f",&k4);
    DistortionParam=cvCreateMat(4,1,CV_32FC1);
    cvZero (DistortionParam);
    cvmSet(DistortionParam,0,0,k1);
    cvmSet(DistortionParam,1,0,k2);
    cvmSet(DistortionParam,2,0,k3);
    cvmSet(DistortionParam,3,0,k4);
    t = xMainNode->getChildNode("Detector").getText();
    if (strcmp(t,"Surf")==0)
    {
        t=xMainNode->getChildNode("Surf").getChildNode("cornercount").getText();
        sscanf(t,"%d",&cornercount);
        t=xMainNode->getChildNode("Surf").getChildNode("min_distance").getText();
        sscanf(t,"%f",&min_distance);
        Detector=1;
    }else{
        t=xMainNode->getChildNode("Harris").getChildNode("searchZoneSize").getText();
        sscanf(t,"%d",&searchZoneSize);
        t=xMainNode->getChildNode("Harris").getChildNode("patternSize").getText();
        sscanf(t,"%d",&patternSize);
        t=xMainNode->getChildNode("Harris").getChildNode("minCorrVal").getText();
        sscanf(t,"%f",&minCorrVal);
        t=xMainNode->getChildNode("Harris").getChildNode("MinBorderGreyDist").getText();
        sscanf(t,"%f",&MinBorderGreyDist);
        t=xMainNode->getChildNode("Harris").getChildNode("cornercount").getText();
        sscanf(t,"%d",&cornercount);
        t=xMainNode->getChildNode("Harris").getChildNode("quality_level").getText();
        sscanf(t,"%f",&quality_level);
        t=xMainNode->getChildNode("Harris").getChildNode("min_distance").getText();
        sscanf(t,"%f",&min_distance);
        printf("Harris: %d %d %f %f %d %f %f\n",searchZoneSize, patternSize, minCorrVal, MinBorderGreyDist, cornercount, quality_level,min_distance);
        Detector=0;
    }

   storage = cvCreateMemStorage(0);
   feat= cvCreateSeq( CV_32FC3, /* sequence of integer elements */
                      sizeof(CvSeq), /* header size - no extra fields */
                      sizeof(CvPoint3D32f), /* element size */
                      storage /* the container storage */ );

   storage_old = cvCreateMemStorage(0);
   feat_old = cvCreateSeq( CV_32FC3, /* sequence of integer elements */
                      sizeof(CvSeq), /* header size - no extra fields */
                      sizeof(CvPoint3D32f), /* element size */
                      storage /* the container storage */ );

    t=xMainNode->getChildNode("InvDepth").getText();
    sscanf(t,"%f",&depth);

    // fonts for data out
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
Xp=0; X=0;  A=0;  B=0;  H=0;  Q=0;  R=0; Pp=0; K=0;  P=0;  control=0;


}
CvMat* CSlam::String2CvMat(char *s,int cols, int rows)
{
    std::string ss = s;
    std::vector<std::string> vs;
    CvMat *m= cvCreateMat(rows,cols,CV_32FC1);
    std::cout<<"m"<<m<<std::endl;
    StringSplit(ss," ",&vs);
    double v;
    int read=0;
    for (int i=0; i< rows;i++){
        for (int j=0; j<cols;j++){
            sscanf(vs[read].c_str(),"%lf", &v);
            read++;
            cvmSet(m,i,j,v);
        }
    }
    return m;
}
void CSlam::StringSplit(std::string str, std::string delim, std::vector<std::string> *results)
{
    unsigned int cutAt;
    while( (cutAt = str.find_first_of(delim)) != str.npos )
    {
        if(cutAt > 0)
        {
            results->push_back(str.substr(0,cutAt));
        }
        str = str.substr(cutAt+1);
    }
    if(str.length() > 0)
    {
        results->push_back(str);
    }
    first=0;
}
CSlam::~CSlam()
{
    //dtor
}

void CSlam::run(IplImage* img)
{

if (first!=0){
        std::cout<<"run"<<std::endl;
         MemMat2WorkMat();
        std::cout<<"predict"<<std::endl;
  //       PrintKalman();
         predict();
    //     PrintKalman();
        std::cout<<"project"<<std::endl;
         projectAllPoints(Xp);
        std::cout<<"match"<<std::endl;
        //matchFile(img);
        if (Detector==0)
         matchHarris(img);
        else
         matchSurf(img);
         MemMat2WorkMat();
        std::cout<<"test"<<std::endl;
         test();
        std::cout<<"correct"<<std::endl;
         MemMat2WorkMat();
         correct();
      // PrintKalman();
        std::cout<<"add"<<std::endl;
        if (Detector==0)
         addHarris(img);
        else
         addSurf(img);
        //PrintKalman();
         //addFile(img);
         MemMat2WorkMat();
        std::cout<<"project"<<std::endl;
         projectAllPoints(X);
        std::cout<<"data Out"<<std::endl;
         Draw(img);
    }
    first=1;
    old_img = cvCloneImage(img);

}
void CSlam::ReserveMemory()
{

     XpMem= cvCreateMat(1000,1,CV_32FC1);
     cvZero(XpMem);
     cvmSet(XpMem,3,0,1);//quaternion (1,0,0,0) is 0 rotation

     Simul_State=cvCreateMat(13,1,CV_32FC1);
     cvZero(Simul_State);
     cvmSet(Simul_State,3,0,1);//quaternion (1,0,0,0) is 0 rotation

     XMem= cvCreateMat(1000,1,CV_32FC1);
     cvZero(XMem);
     cvmSet(XpMem,3,0,1);//quaternion (1,0,0,0) is 0 rotation

     AMem= cvCreateMat(1000,1000,CV_32FC1);
     cvZero(AMem);
     if(modelCD>0){
        BMem= cvCreateMat(1000,modelCD,CV_32FC1);
     cvZero(BMem);
     }
     HMem=cvCreateMat(300,1000,CV_32FC1);
     HFullMem=cvCreateMat(300,1000,CV_32FC1);
     QMem=cvCreateMat(1000,1000,CV_32FC1);   ///< process noise covariance matrix (Q)
     for (int i = 0 ; i<12; i++)
        cvmSet(QMem,i,i,0.1);
     RMem=cvCreateMat(600,600,CV_32FC1); ///< measurement noise covariance matrix (R)
     cvSetIdentity( RMem, cvRealScalar(MEAS_NOISE_COV) );

     PpMem=cvCreateMat(1000,1000,CV_32FC1);       ///< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
     KMem=cvCreateMat(1000,600,CV_32FC1);                /// Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)*/
     PMem=cvCreateMat(1000,1000,CV_32FC1);      /// posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) */

     predictionMem=cvCreateMat(600,1,CV_32FC1); ///Measurement prediction

     measurementMem=cvCreateMat(600,1,CV_32FC1); ///Measurement done

     temp1Mem=cvCreateMat(1000,1000,CV_32FC1);
     temp2Mem=cvCreateMat(600,1000,CV_32FC1);
     temp3Mem=cvCreateMat(600,600,CV_32FC1);
     temp4Mem=cvCreateMat(600,1000,CV_32FC1);
     temp5Mem=cvCreateMat(600,1,CV_32FC1);

}

void CSlam::v2q(CvMat* v, CvMat* q)
{
    double theta = sqrt(cvmGet(v,0,0)*cvmGet(v,0,0)+
                        cvmGet(v,1,0)*cvmGet(v,1,0)+
                        cvmGet(v,2,0)*cvmGet(v,2,0));
    if (theta != 0.0){
    CvMat* vn = cvCreateMat(3,1,CV_32FC1);
    for (int i = 0 ; i<v->height; i++)
        cvmSet(vn,i,0,cvmGet(v,i,0)/theta);

    cvmSet(q,0,0,cos(theta/2));
    for (int i = 1; i <4; i++)
        cvmSet(q,i,0,cvmGet(vn,i-1,0)*sin(theta/2));

    cvReleaseMat(&vn);
    }else{
        cvmSet(q,0,0,1);
        cvmSet(q,1,0,0);
        cvmSet(q,2,0,0);
        cvmSet(q,3,0,0);
    }
}

void CSlam::dv2qdv(CvMat *v, CvMat *dqdv)
{
    double theta = sqrt(cvmGet(v,0,0)*cvmGet(v,0,0)+
                        cvmGet(v,1,0)*cvmGet(v,1,0)+
                        cvmGet(v,2,0)*cvmGet(v,2,0));
    if (theta == 0) theta = 0.000000000001;
    CvMat* _vn = cvCreateMat(3,1,CV_32FC1);
    for (int i = 0 ; i<v->height; i++)
        cvmSet(_vn,i,0,cvmGet(v,i,0)/theta);
    float * vn = _vn->data.fl;
    float * _v = v->data.fl;
    CvMat * dThVn_dv = cvCreateMat(4,3,CV_32FC1);
    cvmSet(dThVn_dv,0,0,vn[0]);cvmSet(dThVn_dv,0,1,vn[1]);cvmSet(dThVn_dv,0,2,vn[2]);

    cvmSet(dThVn_dv, 1,0,(1/theta)-(vn[0]/theta));
    cvmSet(dThVn_dv, 1,1,-_v[0]*_v[1]/(theta*theta*theta));
    cvmSet(dThVn_dv, 1,2,-_v[0]*_v[2]/(theta*theta*theta));

    cvmSet(dThVn_dv, 2,0,-_v[1]*_v[0]/(theta*theta*theta));
    cvmSet(dThVn_dv, 2,1,(1/theta)-(vn[1]/theta));
    cvmSet(dThVn_dv, 2,2,-_v[1]*_v[2]/(theta*theta*theta));

    cvmSet(dThVn_dv, 3,0,-_v[2]*_v[0]/(theta*theta*theta));
    cvmSet(dThVn_dv, 3,1,-_v[2]*_v[1]/(theta*theta*theta));
    cvmSet(dThVn_dv, 3,2,(1/theta)-(vn[2]/theta));

    CvMat * dqdThVn = cvCreateMat(4,4,CV_32FC1);

    cvmSet(dqdThVn, 0,0,-sin(theta/2.0)*0.5);
    cvmSet(dqdThVn, 0,1,0);
    cvmSet(dqdThVn, 0,2,0);
    cvmSet(dqdThVn, 0,3,0);
    cvmSet(dqdThVn, 1,0,vn[0]*cos(theta/2)*0.5);
    cvmSet(dqdThVn, 1,1,sin(theta/2));
    cvmSet(dqdThVn, 1,2,0);
    cvmSet(dqdThVn, 1,3,0);

    cvmSet(dqdThVn, 2,0,vn[1]*cos(theta/2)*0.5);
    cvmSet(dqdThVn, 2,1,0);
    cvmSet(dqdThVn, 2,2,sin(theta/2));
    cvmSet(dqdThVn, 2,3,0);

    cvmSet(dqdThVn, 3,0,vn[2]*cos(theta/2)*0.5);
    cvmSet(dqdThVn, 3,1,0);
    cvmSet(dqdThVn, 3,2,0);
    cvmSet(dqdThVn, 3,3,sin(theta/2));

    cvMatMul(dqdThVn,dThVn_dv,dqdv);

}
void CSlam::quatMult(CvMat* _q, CvMat* _p,CvMat* dest)
{
    float* p;
    float* q;
    float* d;
    p = _p->data.fl;
    q = _q->data.fl;
    d = dest->data.fl;
    d[0]= p[0]*q[0]-p[1]*q[1]-p[2]*q[2]-p[3]*q[3];
    d[1]= q[0]*p[1]+p[0]*q[1]+q[2]*p[3]-q[3]*p[2];
    d[2]= q[0]*p[2]+p[0]*q[2]+q[3]*p[1]-q[1]*p[3];
    d[3]= q[0]*p[3]+p[0]*q[3]+q[1]*p[2]-q[2]*p[1];

}
/** Quaterion product derivative with respect to first argument **/
void CSlam::dq3dq1(CvMat* q2, CvMat *der)
{
    float* q=q2->data.fl;
    cvmSet(der,0,0,q[0]);cvmSet(der,0,1,-q[1]);cvmSet(der,0,2,-q[2]);cvmSet(der,0,3,-q[3]);
    cvmSet(der,1,0,q[1]);cvmSet(der,1,1,q[0]);cvmSet(der,1,2,q[3]); cvmSet(der,1,3,-q[2]);
    cvmSet(der,2,0,q[2]);cvmSet(der,2,1,-q[3]);cvmSet(der,2,2,q[0]);cvmSet(der,2,3,q[1]);
    cvmSet(der,3,0,q[3]);cvmSet(der,3,1,q[2]);cvmSet(der,3,2,-q[1]);cvmSet(der,3,3,q[0]);

}
/** Quaterion product derivative with respect to second argument **/
void CSlam::dq3dq2(CvMat* q2, CvMat *der)
{
    float* q=q2->data.fl;
    cvmSet(der,0,0,q[0]);cvmSet(der,0,1,-q[1]);cvmSet(der,0,2,-q[2]);cvmSet(der,0,3,-q[3]);
    cvmSet(der,1,0,q[1]);cvmSet(der,1,1,q[0]);cvmSet(der,1,2,-q[3]); cvmSet(der,1,3,q[2]);
    cvmSet(der,2,0,q[2]);cvmSet(der,2,1,q[3]);cvmSet(der,2,2,q[0]);cvmSet(der,2,3,-q[1]);
    cvmSet(der,3,0,q[3]);cvmSet(der,3,1,-q[2]);cvmSet(der,3,2,q[1]);cvmSet(der,3,3,q[0]);
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
void CSlam::predict()
{
  float D_t=1;
  /* update the state */
  /* x'(k) = A*x(k) */
  /** x'(k) = f(x(k)) **/
  CvMat *r = cvCreateMatHeader(3,1,CV_32FC1);
  CvMat *r_p = cvCreateMatHeader(3,1,CV_32FC1);
  cvGetSubRect(X,r, cvRect(0, 0, 1,3));
  cvGetSubRect(Xp,r_p, cvRect(0, 0, 1,3));
  CvMat *v = cvCreateMatHeader(3,1,CV_32FC1);
  CvMat *v_p = cvCreateMatHeader(3,1,CV_32FC1);
  cvGetSubRect(X,v, cvRect(0, 7, 1,3));
  cvGetSubRect(Xp,v_p, cvRect(0, 7, 1,3));
  CvMat *q = cvCreateMatHeader(4,1,CV_32FC1);
  CvMat *q_p = cvCreateMatHeader(4,1,CV_32FC1);
  cvGetSubRect(X,q, cvRect(0, 4, 1,4));
  cvGetSubRect(Xp,q_p, cvRect(0, 4, 1,4));
  CvMat *w = cvCreateMatHeader(3,1,CV_32FC1);
  CvMat *w_p = cvCreateMatHeader(3,1,CV_32FC1);
  cvGetSubRect(X,w, cvRect(0, 10, 1,3));
  cvGetSubRect(Xp,w_p, cvRect(0, 10, 1,3));

  /** r'= r+v*Dt **/
  cvAddWeighted(r,1,v,D_t,0,r_p);
  /** q'=q x v2q(w*Dt) **/
  CvMat *wdt = cvCreateMat(3,1,CV_32FC1);
  CvMat *q_w = cvCreateMat(4,1,CV_32FC1);
  cvConvertScale( w,wdt, D_t );
  v2q(wdt,q_w);
  quatMult(q,q_w,q_p);
  /** v' = v **/
  cvCopy(v,v_p);
  /** w' = w **/
  cvCopy(w,w_p);
  /** x_feat' = x_feat **/
  for (int i = 13; i < Xp->height; i++)
  {
    cvmSet(Xp,i,0,cvmGet(X,i,0));
  }

  /** update error covariance matrices **/
  CvMat *_A= cvCreateMatHeader(13,13,CV_32FC1);
  cvGetSubRect(A,_A, cvRect(0, 0, 13,13));
  cvSetIdentity(_A);
  for (int i=0; i < 3; i++)
    cvmSet(_A,i,7+i,D_t);

  CvMat *dqpdq= cvCreateMatHeader(3,3,CV_32FC1);
  cvGetSubRect(A,dqpdq,cvRect(3,3,4,4));//pos 3,3 size 4,4
  dq3dq1(q_w, dqpdq);  //le paso el el vector sobre el que no derivo que son los coef del que derivo.

  /** dq/dw = dq3dq2*dv2qdv*d_t **/
  CvMat *_dq3dq2 = cvCreateMat(4,4,CV_32FC1);
  dq3dq2(q,_dq3dq2);

  CvMat * dqdv = cvCreateMat(4,3,CV_32FC1);
  dv2qdv(w,dqdv);

  CvMat *dq_by_dw = cvCreateMatHeader(4,3,CV_32FC1);
  cvGetSubRect(A,dq_by_dw,cvRect(10,3,3,4));//pos 3,3 size 4,4
  cvMatMul(_dq3dq2,dqdv,dq_by_dw);

    /** update Q matrix **/
    /** Q = G* Pn *G' **/
    /** Pn = Diag [ a*Dt a*Dt a*Dt alpha*Dt alpha*Dt alpha*Dt ] **/
    CvMat *Pn = cvCreateMat (6,6,CV_32FC1);
    cvZero(Pn);
    for (int i = 0; i<3; i++) cvmSet(Pn,i,i,avalue*avalue*D_t*D_t);
    for (int i = 3; i<6; i++) cvmSet(Pn,i,i,alpha*alpha*D_t*D_t);
    /** G= dX_by_dVOmega **/
    CvMat *G =cvCreateMatHeader(13,6,CV_32FC1);
    cvGetSubRect(A,G,cvRect(7,0,6,13));

    CvMat *_Q = cvCreateMatHeader(13,13,CV_32FC1);
    cvGetSubRect(Q,_Q,cvRect(0,0,13,13));

    CvMat *temp = cvCreateMat(13,6,CV_32FC1);
    cvGEMM(G,Pn,1,NULL,0,temp,0);//temp = G*Pn
    cvGEMM(temp,G,1,NULL,0,_Q,CV_GEMM_B_T);
  /* temp1 = A*P(k) */
  cvMatMulAdd( A, P, 0, temp1 );

  /* P'(k) = temp1*At + Q */
  cvGEMM( temp1, A, 1, Q, 1,
                   Pp, CV_GEMM_B_T );
//    printf("------------A-------------\n");
//    printMat(A);
//    printf("-------------P------------\n");
//    printMat(P);
//    printf("------------Q-------------\n");
//    printMat(Q);
//    printf("-------------Pp-----------\n");
//    printMat(Pp);

   // cvCopy(P,Pp);///FIXME FIXME
//    /** Normalize quaternion **/
//    NormalizeQuatCov(Xp,Pp);
//    double mod = sqrt(cvmGet(Xp,3,0)*cvmGet(Xp,3,0)+
//                      cvmGet(Xp,4,0)*cvmGet(Xp,4,0)+
//                      cvmGet(Xp,5,0)*cvmGet(Xp,5,0)+
//                      cvmGet(Xp,6,0)*cvmGet(Xp,6,0));
//    for (int i = 3; i<7; i++)
//        cvmSet(Xp,i,0,cvmGet(Xp,i,0)/mod);
//


//  printf("G\n");
//  printMat(G);
//  printf("Pn\n");
//  printMat(Pn);
//  printf("Q\n");
//  printMat(Q);
//  printf("Pp\n");
//  printMat(Pp);
  cvReleaseMatHeader(&G);
  cvReleaseMatHeader(&_Q);
  cvReleaseMat(&temp);
  cvReleaseMat(&Pn);
  cvReleaseMat (&_dq3dq2);
  cvReleaseMat(&dqdv);
  cvReleaseMatHeader(&dq_by_dw);
  cvReleaseMatHeader(&_A);
  cvReleaseMatHeader(&dqpdq);
  cvReleaseMatHeader(&wdt);
  cvReleaseMatHeader(&q_w);
  cvReleaseMatHeader(&q);
  cvReleaseMatHeader(&q_p);
  cvReleaseMatHeader(&v);
  cvReleaseMatHeader(&v_p);
  cvReleaseMatHeader(&r);
//  cvReleaseMatHeader(&rp);


//  cvMatMulAdd( A, X, 0, Xp );
//
//  if( control && CD > 0 )
//    {
//      /* x'(k) = x'(k) + B*u(k) */
//      cvMatMulAdd( B, control, Xp, Xp);
//    }
}
void CSlam::correct()
{

//  UpdateJacob();
if (MD<1) {
    for(int i = 0; i<SD; i++)
        cvmSet(X,i,0,cvmGet(Xp,i,0));
    std::cout<<"WARNING: No measures"<<std::endl;
    return;
}
  /* temp2 = H*P'(k) */
  cvMatMulAdd( H,Pp, 0, temp2 );
  /* temp3 = temp2*Ht + R */
  cvGEMM( temp2, H, 1,R, 1, temp3, CV_GEMM_B_T );

  /* temp4 = inv(temp3)*temp2 = Kt(k) */
  cvSolve(temp3, temp2, temp4, CV_SVD );
//  cout<<"8"<<endl;
  /* K(k) */
  cvTranspose( temp4, K );

  /* temp5 = z(k) - H*x'(k) */

  if(modelMD!=0)
    {
//      CvMat *tempmeas;
//      tempmeas=pModel->getMeasurementVector();
//      for(int kk=0;kk<pModel->getMeasurementNum();kk++)
//        {
//          cvmSet(measurement,kk,0,cvmGet(tempmeas,kk,0));
//        }
//	  cvReleaseMat(&tempmeas);
    }
  int ii=modelMD;
//  for   (int j =0;j<visible.size();j++)
//    {
//        if (visible.at(j)==true){
//        cvmSet(measurementVisible,ii,0,measurement[2*j]);
//        ii++;
//        cvmSet(measurementVisible,ii,0,measurement[2*j+1]);
//        ii++;
//        }
//    }

  //FIXME esta parte se ejecuta pero solo tiene efecto cuando hay medida de movimiento de camara.
  if(modelMD!=0)
    {
        std::cout<<"Error not implemented"<<std::endl;
  //    cvGEMM( H, Xp, -1, measurementVisible, 1, temp5 );
    }

  ii=modelMD;
  for   (unsigned int j =0;j<visible.size();j++)
    {
        if (visible.at(j)==true){
            //std::cout<<"linearidad--> "<<" Jacob x:"<< cvmGet(temp5,ii,0)-320<<" y: "<<cvmGet(temp5,ii+1,0)-240;
            std::cout<< " error prediccion x: "<< measurement[2*j]-prediction[2*j];
            std::cout<<" y: "<<measurement[2*j+1]-prediction[2*j+1]<<std::endl;

            cvmSet(temp5,ii,0,measurement[2*j]-prediction[2*j]);
            ii++;
            cvmSet(temp5,ii,0,measurement[2*j+1]-prediction[2*j+1]);
            ii++;
            std::cout<<ii<<" "<<temp5->height<<" vis: "<<visible.size()<<" j:"<<j<<" vis[j] "<<visible[j]<<std::endl;
        }
    }
std::cout<<"prematmuladd"<<std::endl;
  /* x(k) = x'(k) + K(k)*temp5 */
  cvMatMulAdd( K,temp5, Xp, X );


  /* P(k) = P'(k) - K(k)*temp2 */
  cvGEMM( K, temp2, -1, Pp, 1,P, 0 );

    /** Normalize quaternion **/
    NormalizeQuatCov(X,P);

    double mod = sqrt(cvmGet(X,3,0)*cvmGet(X,3,0)+
                      cvmGet(X,4,0)*cvmGet(X,4,0)+
                      cvmGet(X,5,0)*cvmGet(X,5,0)+
                      cvmGet(X,6,0)*cvmGet(X,6,0));
    for (int i = 3; i<7; i++)
        cvmSet(X,i,0,cvmGet(X,i,0)/mod);
std::cout<<"end correc"<<std::endl;
}
void CSlam::test()
{
    if (visible.size()>0)
    {
        pairingsBest = 0;
        maxLevels = visible.size();
        //std::vector<int> H;
//          JCBB(0,visible.size());
//        H_.clear();
//        for (unsigned int i = 0 ; i < BestH.size() ; i++ )
//        {
//            std::cout<<BestH[i]<<" ";
//            visible[i]=BestH[i];
//        }
//        std::cout<<pairingsBest<<std::endl;
        JCBB_incremental(0,visible.size(),NULL,NULL,NULL,NULL);
        H_.clear();
        for (unsigned int i = 0 ; i < visible.size() ; i++ )
        {
            if ( i < BestH.size()){
                std::cout<<BestH[i]<<" ";
                visible[i]=BestH[i];
            }else{
                visible[i]=0;
            }
        }
        std::cout<<pairingsBest<<std::endl;
        visNum=0;
        for (unsigned int i = 0 ; i<visible.size(); i++){
            if (visible.at(i)== true)
                visNum++;
        }
        MD=modelMD+visNum*2;
    }
}
/**
 * copia una matriz en la esquina superior izquierad de una mas grande
 * @param o_mat matriz de origen
 * @param d_mat matriz de destino
 **/
void CSlam::transMat(CvMat* o_mat, CvMat* d_mat)
{
  CvMat *submat;
  submat=cvCreateMatHeader( o_mat->rows,o_mat->cols,CV_32FC1 );
  cvGetSubRect( d_mat, submat, cvRect(0, 0, o_mat->cols, o_mat->rows));
  cvCopy( o_mat, submat );
  cvReleaseMatHeader(&submat);
}
/** Select a part of the reserved memory to create matrixes.
* IMPORTANT: This function doesnt fill any value
**/
void CSlam::MemMat2WorkMat()
{

    int state = SD;
    int meas = MD;
    int input = CD;
  /*     kalman->state_pre = cvCreateMat( DP, 1, CV_32FC1 ));
	cvZero( kalman->state_pre );*/
  Xp = cvCreateMatHeader( state, 1, CV_32FC1 );
  cvGetSubRect(XpMem,Xp, cvRect(0, 0, 1,state));
  //     kalman->state_post = cvCreateMat( DP, 1, CV_32FC1 ));
  X = cvCreateMatHeader( state, 1, CV_32FC1 );
  cvGetSubRect(XMem,X, cvRect(0, 0, 1,state));
  //     kalman->transition_matrix = cvCreateMat( DP, DP, CV_32FC1 ));
  A = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(AMem,A, cvRect(0, 0, state,state));
  //    if( CP > 0 )
  //    {
  //         kalman->control_matrix = cvCreateMat( DP, CP, CV_32FC1 ));
  if(input>0){
    B = cvCreateMatHeader( state,input, CV_32FC1 );
    cvGetSubRect(BMem,B,cvRect(0, 0, input,state));
  }else{
      B=0;
  }
  //   kalman->measurement_matrix = cvCreateMat( MP, DP, CV_32FC1 ));
  if (meas>0){
    H = cvCreateMatHeader(  meas,state, CV_32FC1 );
    cvGetSubRect(HMem,H,cvRect(0, 0, state,meas));
  }
  if (visible.size()>0){
    HFull = cvCreateMatHeader( 2* visible.size(),state, CV_32FC1 );
    cvGetSubRect(HFullMem,HFull,cvRect(0, 0, state,2*visible.size()));
  }
  //     kalman->process_noise_cov = cvCreateMat( DP, DP, CV_32FC1 ));
  Q = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(QMem,Q, cvRect(0, 0, state,state));
  //	     kalman->measurement_noise_cov = cvCreateMat( MP, MP, CV_32FC1 ));
  if (meas>0){
    R = cvCreateMatHeader( meas,meas, CV_32FC1 );
    cvGetSubRect(RMem,R,cvRect(0, 0, meas,meas));
  }
  //    	 kalman->error_cov_pre = cvCreateMat( DP, DP, CV_32FC1 ));
  Pp = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(PpMem,Pp, cvRect(0, 0, state,state));
  //    	 kalman->error_cov_post = cvCreateMat( DP, DP, CV_32FC1 ));
  P = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(PMem,P, cvRect(0, 0, state,state));
  //	 kalman->gain = cvCreateMat( DP, MP, CV_32FC1 ));
  if (meas>0){
    K = cvCreateMatHeader(  state,meas, CV_32FC1 );
    cvGetSubRect(KMem,K, cvRect(0, 0, meas,state));
  }
//  if (meas>0){
//    predictionVisible= cvCreateMatHeader(  meas,1, CV_32FC1 );
//    cvGetSubRect(predictionMem,predictionVisible, cvRect(0, 0, 1,meas));
//  }
//  if (meas>0){
//    measurementVisible= cvCreateMatHeader(  meas,1, CV_32FC1 );
//    cvGetSubRect(measurementMem,measurementVisible, cvRect(0, 0, 1,meas));
//  }
  //   kalman->temp1 = cvCreateMat( DP, DP, CV_32FC1 ));
  temp1 = cvCreateMatHeader( state, state, CV_32FC1 );
  cvGetSubRect(temp1Mem,temp1,//fixme
	       cvRect(0, 0, state,state));
  //   kalman->temp2 = cvCreateMat( MP, DP, CV_32FC1 ));
  if (meas>0){
    temp2 = cvCreateMatHeader(  meas,state, CV_32FC1 );
    cvGetSubRect(temp2Mem,temp2,
		 cvRect(0, 0, state,meas));
  }
  //     kalman->temp3 = cvCreateMat( MP, MP, CV_32FC1 ));
  if (meas>0){
    temp3 = cvCreateMatHeader( meas,meas, CV_32FC1 );
    cvGetSubRect(temp3Mem,temp3,
		 cvRect(0, 0, meas,meas));
  }
  //     kalman->temp4 = cvCreateMat( MP, DP, CV_32FC1 ));
  if (meas>0){
    temp4 = cvCreateMatHeader( meas,state, CV_32FC1 );
    cvGetSubRect(temp4Mem,temp4,
		 cvRect(0, 0, state,meas));
  }
  //     kalman->temp5 = cvCreateMat( MP, 1, CV_32FC1 ));
  if (meas>0){
    temp5 = cvCreateMatHeader(meas,1, CV_32FC1 );
    cvGetSubRect(temp5Mem,temp5, cvRect(0, 0, 1,meas));
  }
}
void CSlam::matchSurf(IplImage *img)
{
    if (keys.size()>0){
        IplImage *grey =cvCreateImage(cvGetSize(img),8,1);
        cvCvtColor(img,grey,CV_RGB2GRAY);

        IplImage *grey2 =cvCreateImage(cvGetSize(img),8,1);
        cvCvtColor(img,grey2,CV_RGB2GRAY);

        surf.img = grey;
        cvClearSeq( feat );
        surf.find_features( grey,feat, levels);
//        if(keys.size()>0)
//            testOpticalFlow(img, old_img, 1, 15, 1, 15);
        double ang;
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
             delete[] desc;
        }
        int key[64];
        int projx, projy;
        for ( unsigned int i =0 ;i<keys.size();i++) {
            ///FIXME Esto tiene que estar relacionado con un valor mas claro del surf
             projx=(int)prediction[i*2];
             projy=(int)prediction[i*2+1];
             if(projx>(9*levels+1) && projx<img->width-(9*levels+1) &&
                projy>(9*levels+1) && projy<img->height-(9*levels+1))
             {
                 for (int d=0; d<64;d++)
                 {
                     key[d]=(keys[i])[d];
                 }
                 /*int p = surf.nearest_neighbor_2(key,samples1,feat->total,
                                               cvPoint((int)projx,(int)projy),
                                               new_points,50);*/
                 std::vector<int> result;
                 int n = surf.nearest_neighbor_classify3(key,samples1,feat->total,result);
                 float ic;
                 float best_ic=10000;
                 int best_ic_n=0;
                 if (n!=-1){
                     for (unsigned int k= 0 ; k<result.size(); k++)
                     {
                         std::cout<<"result: "<<result[k]<<" ";
                         ic = ic_test(i, cvmGet(new_points,result[k],0), cvmGet(new_points,result[k],1)) ;
                         if (ic<best_ic){
                             best_ic= ic;
                             best_ic_n=k;
                         }
                     }
                     if (n != result[best_ic_n]) {
                         std::cout<<"Capturado un punto por IC-Test n:"<<n<<" res:"<<result[best_ic_n]<<std::endl;
                         n = result[best_ic_n];
                     }
                 }
                 //FIXME Use joint compatibility test here
                 if(n==-1)//en caso de no encorar fallo
                 {
                     //cout<<"no match "<<endl;
                     if(visible.at(i)==true)
                     {
                        //std::cout << "borropuntos: "<<(*It)->pto.x<<" "<<(*It)->pto.y<<std::endl;
                        visible[i]=0;//false
                     }

//                    measurement[2*i]=0;
//                    measurement[2*i+1]=0;
                 }
                 else//encaso de exito
                 {
//                     std::cout<<
//                     std::cout<<measurement[2*i]<<" "<<measurement[2*i+1]<<" "<<cvGetReal2D(imgU1,measurement[2*i+1],measurement[2*i])<<" ";
//                     std::cout<<cvGetReal2D(imgV1,measurement[2*i+1],measurement[2*i])<<" ";
//                     std::cout<<measurement[2*i]+cvGetReal2D(imgU1,measurement[2*i+1],measurement[2*i])<<" ";
//                     std::cout<<measurement[2*i+1]+cvGetReal2D(imgV1,measurement[2*i+1],measurement[2*i])<<" ";
//                     std::cout<<cvmGet(new_points,n,0)<<" "<<cvmGet(new_points,n,1)<<std::endl;

                       measurement[2*i]=cvmGet(new_points,n,0);
                       measurement[2*i+1]=cvmGet(new_points,n,1);
                       if(visible.at(i)==false)
                       {
                         visible[i]=1;//true
                       }
                 }
            }else
            {
                if(visible.at(i)==true)
                 {

                     visible[i]=0;//false;
                 }
                measurement[2*i]=0;
                measurement[2*i+1]=0;
            } //end if proj dentro borde
        }//end for cada elemento del mapa

        int vis = 0;
        for (unsigned int k = 0; k<visible.size(); k++)
        {
            if (visible.at(k)==true) {
                vis++;
            }
        }
        MD=modelMD+2*vis;

        cvShowImage("win",grey2);
        cvWaitKey(100);

        cvReleaseImage(&grey);
        cvReleaseImage(&grey2);
        delete[] samples1;
        cvReleaseMat(&new_points);
    }
}
void CSlam::matchHarris(IplImage *img)
{
if (keys.size()>0){
    CvMat*_tmp2 = cvCreateMatHeader(visible.size()*2,SD,CV_32FC1);
    CvMat*_tmp3 = cvCreateMatHeader(visible.size()*2,visible.size()*2,CV_32FC1);
    CvMat*_R = cvCreateMat(visible.size()*2,visible.size()*2,CV_32FC1);
    cvZero(_R);
    cvGetSubRect(temp2Mem,_tmp2,
		 cvRect(0, 0, SD,visible.size()*2));
    cvGetSubRect(temp3Mem,_tmp3,
		 cvRect(0, 0, visible.size()*2,visible.size()*2));

  /* temp2 = H*P'(k) */
  cvMatMulAdd( HFull,Pp, 0, _tmp2 );
  /* temp3 =temp2*Ht + R */
  cvGEMM( _tmp2, HFull, 1,_R, 1, _tmp3, CV_GEMM_B_T );

    IplImage *zonabusca;
    IplImage *corr;
    CvPoint maxLoc;
    int ancho, alto;
    zonabusca=cvCreateImage( cvSize( searchZoneSize, searchZoneSize), 8, 1 );
    ancho= cvGetSize(zonabusca).width-patternSize +1;
    alto = cvGetSize(zonabusca).height-patternSize+1;
    corr = cvCreateImage( cvSize( ancho, alto), 32, 1 );

   IplImage *grey;
   grey=cvCreateImage( cvGetSize(img), 8, 1 );
   CvPoint temp;
   cvCvtColor(img,grey,CV_RGB2GRAY);

   IplImage *patron;
   patron = cvCreateImageHeader( cvSize(patternSize,patternSize),8,1);
   int projx;
   int projy;
   CvMat *S = cvCreateMatHeader(2,2,CV_32FC1);
   CvMat *Sinv = cvCreateMat(2,2,CV_32FC1);
   CvMat *innov = cvCreateMat(2,1,CV_32FC1);
   CvMat *_temp = cvCreateMat(2,1,CV_32FC1);
   CvMat *chi = cvCreateMat(1,1,CV_32FC1);
   for (unsigned int i =0 ;i<keys.size();i++) {
        projx=(int)prediction[i*2];
        projy=(int)prediction[i*2+1];
        patron->imageData=(char*)keys[i];
        cvGetSubRect(_tmp3,S,cvRect(2*i,2*i,2,2));
        cvInvert(S,Sinv);

        double suma=0;
        double suma1 = 0;
        double suma2 = 0;
        double maxcorr = 0;
        double nx, ny;
        bool nomatch = true;

        for ( int ii = -searchZoneSize/2; ii<searchZoneSize/2; ii++)//width
            for (int jj = -searchZoneSize/2; jj< searchZoneSize/2 ; jj++)//height
            {
                nx = ii + projx;
                ny = jj + projy;
                cvmSet(innov,0,0,ii);
                cvmSet(innov,1,0,jj);
                cvMatMul(Sinv,innov,_temp);
                cvGEMM(innov,_temp,1,NULL,0,chi,CV_GEMM_A_T);
                if (cvmGet(chi,0,0)< 3)//5.99)
                {
                    suma1 = 0;
                    suma2 = 0;
                    suma=0;
                    for (int iii = 0 ; iii< patron->width; iii++)
                        for (int jjj = 0; jjj< patron->height; jjj++)
                        {
                            double greyx = iii+nx-patron->width/2;
                            double greyy = jjj+ny-patron->height/2;
                            if (greyx>0 &&greyx<grey->width&&greyy>0 &&greyy<grey->height)
                            {
                                double patronPix = (uchar) patron->imageData[iii+jjj*patron->widthStep];//patronScalar.val[0];
                                double greyPix = (uchar) grey->imageData[(int)(greyx+greyy*grey->widthStep)];//greyScalar.val[0];
                                suma += patronPix*greyPix;
                                suma1 += (patronPix*patronPix);
                                suma2 += (greyPix*greyPix);
                            }
                        }
                    suma = suma / sqrt(suma1*suma2);

                    if (suma > maxcorr )
                    {
                        maxcorr = suma;
                        maxLoc.x =(int) nx;
                        maxLoc.y =(int) ny;
                    }
                }
            }

          if (maxcorr > minCorrVal)
          {
              nomatch= false ;
            temp.x=maxLoc.x;
            temp.y=maxLoc.y;
          }
          else {
           printf("hey not enought corr\n");
          }

        if (nomatch == true )
        {
            //std::cout<<"no match "<<sqrt(dx*dx+dy*dy)<<" "<<maxVal<<" "<<maxVal2<<std::endl;
            std::cout<<"no match "<<maxcorr<<" "<<std::endl;
            measurement[2*i]=temp.x;
            measurement[2*i+1]=temp.y;
            if(visible.at(i)==true)
            {
                visible[i]=0;
            }
        }
        else
        {
            std::cout<<"si match "<<maxcorr<<" "<<std::endl;
            measurement[2*i]=temp.x;
            measurement[2*i+1]=temp.y;
            if(visible.at(i)==false)
            {
                visible[i]=1;//true
            }
        }
    }//end for
    cvReleaseMatHeader(&S);
    cvReleaseMat(&Sinv);
    cvReleaseMat(&innov);
    cvReleaseMat(&_temp);
    cvReleaseMat(&chi);

    printf("endMatch");
    cvReleaseImage(&grey);
    cvReleaseImageHeader(&patron);
    cvReleaseMat(&_R);
    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true)
            visNum++;
    }
}
}
void CSlam::matchFile(IplImage *img)
{

    CvPoint3D64f ptos[21];
    for (int i = 0; i<7; i++){
        for (int j = 0 ; j<3; j++){
            ptos[i*3+j].x=3*j-3;
            ptos[i*3+j].y=2*i;
            ptos[i*3+j].z=10;
            if (j==0)
                ptos[i*3+j].x+=1;

        }
    }
    CvPoint2D64f m;
    CvMat* dpdr=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdt=cvCreateMat(2,3,CV_32FC1);
    CvMat* dpdf=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdc=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdk=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdw=cvCreateMat(2,6,CV_32FC1);


    for (unsigned int i =0 ;i<keys.size();i++)
      {
        ProjectPoints3( &ptos[i],
                            Simul_State,
                            IntrinsicParam,
                            DistortionParam,
                            &m, dpdr, dpdt, dpdf, dpdc, dpdk , dpdw);
        printf("x: %f ,y: %f\n",m.x,m.y);
        if (m.x>0 && m.y>0 &&m.x<640 && m.y<480)
        {
            std::cout<<"si match "<<m.x<<" "<<m.y<<std::endl;
            measurement[2*i]=m.x;
            measurement[2*i+1]=m.y;
            if(visible.at(i)==false)
            {
                visible[i]=1;//true
            }
        }
        else
        {
            std::cout<<"no match "<<m.x<<" "<<m.y<<std::endl;
            measurement[2*i]=m.x;
            measurement[2*i+1]=m.y;
            if(visible.at(i)==true)
            {
                visible[i]=0;
            }
        }
    }//end for

    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true)
            visNum++;
    }

}
void CSlam::addFile(IplImage* img){

    IplImage *grey, *patron;
    grey=cvCreateImage( cvGetSize(img), 8, 1 );
    patron=cvCreateImage( cvSize(patternSize,patternSize), 8, 1 );
    cvCvtColor(img,grey,CV_RGB2GRAY);

    CvPoint3D64f ptos[21];
    for (int i = 0; i<7; i++){
        for (int j = 0 ; j<3; j++){
            ptos[i*3+j].x=3*j-3;
            ptos[i*3+j].y=2*i;
            ptos[i*3+j].z=10;
            if (j==0)
                ptos[i*3+j].x+=1;
        }
    }
    CvPoint2D64f m;
    CvMat* dpdr=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdt=cvCreateMat(2,3,CV_32FC1);
    CvMat* dpdf=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdc=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdk=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdw=cvCreateMat(2,6,CV_32FC1);
    CvMat *h= cvCreateMat(6,1,CV_32FC1);

    visNum=0;
    int maxvis = -1;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true)
        { visNum++;
            maxvis= i;
        }
    }

    for (int i = maxvis+1; i<21; i++)//empezar desde visnum evita meter repetidos
    {
        ProjectPoints3( &ptos[i],
                            Simul_State,
                            IntrinsicParam,
                            DistortionParam,
                            &m, dpdr, dpdt, dpdf, dpdc, dpdk , dpdw);
        printf("x: %f ,y: %f\n",m.x,m.y);
        cvGetRectSubPix( grey, patron,cvPoint2D32f(m.x,m.y) );//
        if (m.x>0 && m.y>0 &&m.x<640 && m.y<480){
              //int step = (patron->widthStep/sizeof(char)-1);
              char *samples1=new  char[patron->width*patron->height];
              for (int dj= 0; dj<patron->height;dj++){
                 for (int di=0; di<patron->width; di++){
                      samples1[dj*patron->width+di]=(char)*(patron->imageData+(dj*patron->width+di));
                 }
             }
             visible.push_back(true);
             measurement.push_back(m.x);
             measurement.push_back(m.y);
             prediction.push_back(0.0);
             prediction.push_back(0.0);
             keys.push_back((int*) samples1);
             //insertar puntos en invdepht aqui
             InverseParam(&h,cvPoint((int)m.x,(int)m.y));
             AddPointToCovMatrix(m.x,m.y);
             //SD+=6;
             //MemMat2WorkMat();
             std::cout<<SD<<std::endl;
             for ( int k = 6; k>0;k--){
                cvmSet(X,SD-k,0,cvmGet(h,6-k,0));
                //updates model matrix
                cvmSet(A,SD-k,SD-k,1);
             }
             std::cout<<"end"<<std::endl;
        }
    }

    cvReleaseImage(&grey);
    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true) visNum++;
    }
    MD=modelMD+visNum*2;
//    cvCopyImage(img,old_img);
    cvmSet(Simul_State,1,0,cvmGet(Simul_State,1,0)+0.1);
}
  void CSlam::addSurf(IplImage* img)
  {
         /** count numbre of visible points **/
    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true) visNum++;
    }
    /** if visible < numVisibleMin add points**/
    std::vector<int *> temp_keys_old;
    std::vector<int *> temp_keys;

    if (visNum<numVisibleMin)
    {
        //get features from old_image
        IplImage *grey_old =cvCreateImage(cvGetSize(old_img),8,1);
        cvCvtColor(old_img,grey_old,CV_RGB2GRAY);
        surf.img = grey_old;
        cvClearSeq( feat_old );
        surf.find_features( grey_old,feat_old, levels);
        float ang_old;
        int *desc_old;
        int *samples1_old;
        int *samples;
        samples=new int[64*feat_old->total];
        CvMat *new_points_old = cvCreateMat(feat_old->total,4,CV_32FC1) ;
        //CvMat *h_old= cvCreateMat(6,1,CV_32FC1);
        for( int i = 0; i < feat_old->total; i++ )
        {
             CvPoint3D32f point_old = *(CvPoint3D32f*)cvGetSeqElem( feat_old, i );
             ang_old=surf.orientation((int)point_old.x,(int)point_old.y,(int)point_old.z);
             desc_old=surf.descriptor((int)point_old.x,(int)point_old.y,(int)point_old.z,ang_old);

             cvmSet(new_points_old,i,0,point_old.x) ;
             cvmSet(new_points_old,i,1,point_old.y) ;
             cvmSet(new_points_old,i,2,point_old.z) ;
             cvmSet(new_points_old,i,3,ang_old) ;
             samples1_old=new int[64];
             for (int di=0; di<64; di++){
                  samples1_old[di]=(unsigned char )(desc_old[di]+128);
             }
             temp_keys_old.push_back(samples1_old);
              for (int di=0; di<64; di++){
                  samples[i*64+di]=(unsigned char )(desc_old[di]+128);
             }
            if ( desc_old != NULL)
             delete[] desc_old;
        }

        IplImage *grey =cvCreateImage(cvGetSize(img),8,1);
        cvCvtColor(img,grey,CV_RGB2GRAY);
        surf.img = grey;
        cvClearSeq( feat );
        surf.find_features( grey,feat, levels);

        float ang;
        int *desc;
        int *samples1;
        CvMat *new_points = cvCreateMat(feat->total,4,CV_32FC1) ;
        CvMat *h= cvCreateMat(6,1,CV_32FC1);
        for( int i = 0; i < feat->total; i++ )
        {
             CvPoint3D32f point = *(CvPoint3D32f*)cvGetSeqElem( feat, i );
             ang=surf.orientation((int)point.x,(int)point.y,(int)point.z);
             desc=surf.descriptor((int)point.x,(int)point.y,(int)point.z,ang);

             cvmSet(new_points,i,0,point.x) ;
             cvmSet(new_points,i,1,point.y) ;
             cvmSet(new_points,i,2,point.z) ;
             cvmSet(new_points,i,3,ang) ;
             samples1=new int[64];
             for (int di=0; di<64; di++){
                  samples1[di]=(unsigned char )(desc[di]+128);
             }
             temp_keys.push_back(samples1);
             delete[] desc;

        }
        int Nn;

        for(unsigned int f=0; f<temp_keys.size();f++)
        {

            Nn=surf.nearest_neighbor_classify(temp_keys[f], samples,temp_keys_old.size()) ;
            if (Nn!=-1)
            {
                 //Nearest Neightbour
                 bool onenear = false;
                 double x = cvmGet(new_points,f,0);
                 double y = cvmGet(new_points,f,1);
                 int mx = (int)min_distance;
                 int my = (int)min_distance;
                for (unsigned int i = 0 ; i<visible.size();i++)
                {

                    if(((measurement[2*i] - mx)<x) && ((measurement[2*i]+mx) > x )&&
                       ((measurement[2*i+1] - my)<y) && ((measurement[2*i+1]+my) > y ))
                       {
                          onenear= true;
                       }

                    if(((prediction[2*i] - mx)<x) && ((prediction[2*i]+mx) > x )&&
                       ((prediction[2*i+1] - my)<y) && ((prediction[2*i+1]+my) > y ))
                       {
                          onenear= true;
                       }
                }
                if (onenear == false ) {
                     visible.push_back(true);
                     measurement.push_back(cvmGet(new_points,f,0));
                     measurement.push_back(cvmGet(new_points,f,1));
                     prediction.push_back(0.0);
                     prediction.push_back(0.0);
                     keys.push_back(temp_keys[f]);
                     //insertar puntos en invdepht aqui
                     InverseParam(&h,cvPoint((int) cvmGet(new_points,f,0),(int)cvmGet(new_points,f,1)));
                     AddPointToCovMatrix(cvmGet(new_points,f,0),cvmGet(new_points,f,1));
                     //SD+=6;
                     //MemMat2WorkMat();
                     std::cout<<SD<<std::endl;
                     for ( int k = 6; k>0;k--){
                        cvmSet(X,SD-k,0,cvmGet(h,6-k,0));
                     //updates model matrix
                        cvmSet(A,SD-k,SD-k,1);
                     }

                }
            }
        }

    }
    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true) visNum++;
    }
    MD=modelMD+visNum*2;
    //cvCopyImage(img,old_img);

  }

  void CSlam::addHarris(IplImage* img){
    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true) visNum++;
    }
    /** if visible < numVisibleMin add points**/
    std::vector<int *> temp_keys_old;
    std::vector<int *> temp_keys;
    CvMat *h= cvCreateMat(6,1,CV_32FC1);
    if (visNum<numVisibleMin)
    {
        //cvCvtColor(f,grey,CV_RGB2GRAY);
        CvPoint2D32f corners[100];
        IplImage *grey, *eig_image, *temp, *patron;
        grey=cvCreateImage( cvGetSize(img), 8, 1 );
        patron=cvCreateImage( cvSize(patternSize,patternSize), 8, 1 );
        eig_image = cvCreateImage( cvGetSize(img), 32, 1 );
        temp=cvCreateImage(cvGetSize(img),32,1);
        cvCvtColor(img,grey,CV_RGB2GRAY);

        CvPoint pts[30];

        cvGoodFeaturesToTrack(grey,eig_image,temp,corners,&cornercount,quality_level,min_distance, NULL);

        //*keys = cvCreateMat(cornercount,128,CV_32FC1) ;
        //*points = cvCreateMat(cornercount,4,CV_32FC1) ;
        //cout<<"esquinas encontradas"<<cornercount<<endl;
        for (int i=0;i<cornercount;i++){
          pts[i]=cvPointFrom32f( corners[i]);
              //Nearest Neightbour
                 bool onenear = false;
                 double x = pts[i].x;
                 double y = pts[i].y;
                 int mx = (int)min_distance;
                 int my = (int)min_distance;
                for (unsigned int j = 0 ; j<visible.size();j++)
                {
                    if(((measurement[2*j] - mx)<x) && ((measurement[2*j]+mx) > x )&&
                       ((measurement[2*j+1] - my)<y) && ((measurement[2*j+1]+my) > y ))
                       {
                          onenear= true;
                       }
                    if(((prediction[2*j] - mx)<x) && ((prediction[2*j]+mx) > x )&&
                       ((prediction[2*j+1] - my)<y) && ((prediction[2*j+1]+my) > y ))
                       {
                          onenear= true;
                       }
                }
                if (onenear == false ) {
                     cvGetRectSubPix( grey, patron,corners[i] );//
                      //int step = (patron->widthStep/sizeof(char)-1);
                      char *samples1=new  char[patron->width*patron->height];
                      for (int dj= 0; dj<patron->height;dj++){
                         for (int di=0; di<patron->width; di++){
                              samples1[dj*patron->width+di]=(char)*(patron->imageData+(dj*patron->width+di));
                         }
                     }
                    visible.push_back(true);
                     measurement.push_back(x);
                     measurement.push_back(y);
                     prediction.push_back(0.0);
                     prediction.push_back(0.0);
                     keys.push_back((int*) samples1);
                     //insertar puntos en invdepht aqui
                     InverseParam(&h,cvPoint((int)x,(int)y));
                     AddPointToCovMatrix(x,y);
                     //SD+=6;
                     //MemMat2WorkMat();
                     std::cout<<SD<<std::endl;
                     for ( int k = 6; k>0;k--){
                        cvmSet(X,SD-k,0,cvmGet(h,6-k,0));
                     //updates model matrix
                        cvmSet(A,SD-k,SD-k,1);
                     }
                }
        }
        cvReleaseImage(&grey);
        cvReleaseImage(&eig_image);
        cvReleaseImage(&temp);
    }
    visNum=0;
    for (unsigned int i = 0 ; i<visible.size(); i++){
        if (visible.at(i)== true) visNum++;
    }
    MD=modelMD+visNum*2;
    //cvCopyImage(img,old_img);
  }



  /** Dreivative of the rotation matrix elements by the quaternion elements **/
  void CSlam::dR_by_dq(CvMat *dRdr,CvMat *r_vec)
    {
        double r=2* cvmGet(r_vec,0,0);
        double x=2* cvmGet(r_vec,1,0);
        double y=2* cvmGet(r_vec,2,0);
        double z=2* cvmGet(r_vec,3,0);
        cvmSet(dRdr,0,0,r);cvmSet(dRdr,0,1,x);cvmSet(dRdr,0,2,-y);cvmSet(dRdr,0,3,-z);
        cvmSet(dRdr,1,0,-z);cvmSet(dRdr,1,1,y);cvmSet(dRdr,1,2,x);cvmSet(dRdr,1,3,-r);
        cvmSet(dRdr,2,0,y);cvmSet(dRdr,2,1,z);cvmSet(dRdr,2,2,r);cvmSet(dRdr,2,3,x);
        cvmSet(dRdr,3,0,z);cvmSet(dRdr,3,1,y);cvmSet(dRdr,3,2,x);cvmSet(dRdr,3,3,r);
        cvmSet(dRdr,4,0,r);cvmSet(dRdr,4,1,-x);cvmSet(dRdr,4,2,y);cvmSet(dRdr,4,3,-z);
        cvmSet(dRdr,5,0,-x);cvmSet(dRdr,5,1,-r);cvmSet(dRdr,5,2,z);cvmSet(dRdr,5,3,y);
        cvmSet(dRdr,6,0,-y);cvmSet(dRdr,6,1,z);cvmSet(dRdr,6,2,-r);cvmSet(dRdr,6,3,x);
        cvmSet(dRdr,7,0,x);cvmSet(dRdr,7,1,r);cvmSet(dRdr,7,2,z);cvmSet(dRdr,7,3,y);
        cvmSet(dRdr,8,0,r);cvmSet(dRdr,8,1,-x);cvmSet(dRdr,8,2,-y);cvmSet(dRdr,8,3,z);
    }

/**
  *la funcion de proyeccion es la misma que opencv pero ampliada para devolver la derivada
  *de los puntos respecto de los puntos
  **/

void CSlam::ProjectPoints3( CvPoint3D64f* M,
                            CvMat* state,
                            CvMat* I,
                            CvMat* dist_coeffs,
                            CvPoint2D64f* m, CvMat* dpdr,
                            CvMat* dpdt, CvMat* dpdf,
                            CvMat* dpdc, CvMat* dpdk ,CvMat* dpdw)
{
    int  j;
    CvMat* t_vec=cvCreateMat(3,1,CV_32FC1);
    CvMat* q_vec=cvCreateMat(4,1,CV_32FC1);
    CvMat* r_vec=cvCreateMat(3,3,CV_32FC1);
    getTransRot(state,t_vec,q_vec,r_vec);

    float *R;
    R= r_vec->data.fl;
    float  dRdr[36];
    float *t;//[3];
    //double k[4] = {0,0,0,0};
    float *k = dist_coeffs->data.fl;
    double fx, fy, cx, cy;

    float *dpdr_p = 0,*dpdw_p = 0, *dpdt_p = 0, *dpdk_p = 0, *dpdf_p = 0, *dpdc_p = 0;
    int dpdr_step = 0, dpdt_step = 0, dpdk_step = 0, dpdf_step = 0, dpdc_step = 0,dpdw_step=0;

    fx = cvmGet(I,0,0);//a[0];
    fy = cvmGet(I,1,1);//a[4];
    cx = cvmGet(I,0,2);//a[2];
    cy = cvmGet(I,1,2);//a[5];

    //CvMat _R = cvMat( 3, 3, CV_32FC1, R );

    CvMat _dRdr = cvMat( 9, 4, CV_32FC1, dRdr );
    dR_by_dq(&_dRdr,q_vec);

    //cvRodrigues2( r_vec, &_R, &_dRdr );

    t=t_vec->data.fl;

    double X = M->x-t[0], Y = M->y-t[1], Z = M->z-t[2];
    double xp = R[0]*X + R[1]*Y + R[2]*Z ;
    double yp = R[3]*X + R[4]*Y + R[5]*Z ;
    double zp = R[6]*X + R[7]*Y + R[8]*Z ;
    double r2, r4, a1, a2, a3, cdist;
    double xd, yd;
    double x,y,z;
    z = zp ? 1./zp : 1;
    x =xp* z;
    y =yp* z;

    r2 = x*x + y*y;
    r4 = r2*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + k[0]*r2 + k[1]*r4;
    xd = x*cdist + k[2]*a1 + k[3]*a2;
    yd = y*cdist + k[2]*a3 + k[3]*a1;

    m->x = xd*fx + cx;
    m->y = yd*fy + cy;
    if (dpdr != NULL ){
        dpdr_p = dpdr->data.fl;
        dpdr_step = dpdr->step/sizeof(dpdr_p[0]);
        dpdw_p = dpdw->data.fl;
        dpdw_step = dpdw->step/sizeof(dpdw_p[0]);
        dpdt_p = dpdt->data.fl;
        dpdt_step = dpdt->step/sizeof(dpdt_p[0]);
        dpdf_p = dpdf->data.fl;
        dpdf_step = dpdf->step/sizeof(dpdf_p[0]);
        dpdc_p = dpdc->data.fl;
        dpdc_step = dpdc->step/sizeof(dpdc_p[0]);
        dpdk_p = dpdk->data.fl;
        dpdk_step = dpdk->step/sizeof(dpdk_p[0]);

        /* Desrivada de pix respecto de Distorsion? Centro imagen?
            dp / dc = [ 1 0
                        0 1 ] */
        dpdc_p[0] = 1; dpdc_p[1] = 0;
        dpdc_p[dpdc_step] = 0;
        dpdc_p[dpdc_step+1] = 1;
        //dpdc_p += dpdc_step*2;
        /* Derivada de pix respecto de focal */
        /* dp / df = [ xd 0
                       0 yd ]; */
        dpdf_p[0] = xd; dpdf_p[1] = 0;
        dpdf_p[dpdf_step] = 0;
        dpdf_p[dpdf_step+1] = yd;
        /* Derivada de pix respecto a k deformacion */
        /* dp/dk = [fx*x*r2 fx*x*r4 fx*a1 fx*a2;
                    fy*y*r2 fy*y*r4 fy*a3 fy*a1] */
        dpdk_p[0] = fx*x*r2;
        dpdk_p[1] = fx*x*r4;
        dpdk_p[2] = fx*a1;
        dpdk_p[3] = fx*a2;
        dpdk_p[dpdk_step] = fy*y*r2;
        dpdk_p[dpdk_step+1] = fy*y*r4;
        dpdk_p[dpdk_step+2] = fy*a3;
        dpdk_p[dpdk_step+3] = fy*a1;

        /*derivad de pixel no distorsionado respecto de trans dpdx notacion confusa */
        /* dp / dRT = [ z 0 -x*z ;
                        0 z -y*z ] la z creo que es 1/z */
        double dxdp[] = { z, 0, -x*z };
        double dydp[] = { 0, z, -y*z };
        /*derivada de pixel no distorsionado respecto de Trans */
        /* dp / dt = dp / dRT * dRt/dT = dp/dRT * R */
        double dydt[3], dxdt[3];
        dxdt[0]=-R[0]*dxdp[0]-R[3]*dxdp[1]-R[6]*dxdp[2];
        dxdt[1]=-R[1]*dxdp[0]-R[4]*dxdp[1]-R[7]*dxdp[2];
        dxdt[2]=-R[2]*dxdp[0]-R[5]*dxdp[1]-R[8]*dxdp[2];
        dydt[0]=-R[0]*dydp[0]-R[3]*dydp[1]-R[6]*dydp[2];
        dydt[1]=-R[1]*dydp[0]-R[4]*dydp[1]-R[7]*dydp[2];
        dydt[2]=-R[2]*dydp[0]-R[5]*dydp[1]-R[8]*dydp[2];
        /*derivada de pixel distorisonado respecto de trans */
        /* dm/dt= dm/dp * dp/dt = ??*/

        for( j = 0; j < 3; j++ )//para cada x y z tranlation
        {
            double dr2dt = 2*x*dxdt[j] + 2*y*dydt[j]; //dr2 / dt
            double dcdist_dt = k[0]*dr2dt + 2*k[1]*r2*dr2dt; //dcdist / dt
            double da1dt = 2*(x*dydt[j] + y*dxdt[j]); //da_n /dt
            double dmxdt = fx*(dxdt[j]*cdist + x*dcdist_dt +
                k[2]*da1dt + k[3]*(dr2dt + 2*x*dxdt[j]));
            double dmydt = fy*(dydt[j]*cdist + y*dcdist_dt +
                k[2]*(dr2dt + 2*y*dydt[j]) + k[3]*da1dt);
            dpdt_p[j] = dmxdt;
            dpdt_p[dpdt_step+j] = dmydt;
        }
        /* derivada de pixel respecto a posición de pixel en mundo
        tiene que salir lo mismo que dp / dt con el signo cambiado
        */
        double dwdW[6];

        dwdW[0]=R[0]*dxdp[0]+R[3]*dxdp[1]+R[6]*dxdp[2];
        dwdW[1]=R[1]*dxdp[0]+R[4]*dxdp[1]+R[7]*dxdp[2];
        dwdW[2]=R[2]*dxdp[0]+R[5]*dxdp[1]+R[8]*dxdp[2];

        dwdW[3]=R[0]*dydp[0]+R[3]*dydp[1]+R[6]*dydp[2];
        dwdW[4]=R[1]*dydp[0]+R[4]*dydp[1]+R[7]*dydp[2];
        dwdW[5]=R[2]*dydp[0]+R[5]*dydp[1]+R[8]*dydp[2];

        for( j = 0; j < 3; j++ )
        {
            double dr2dt = 2*x*dwdW[j] + 2*y*dwdW[j+3];
            double dcdist_dt = k[0]*dr2dt + 2*k[1]*r2*dr2dt;
            double da1dt = 2*(x*dwdW[j+3] + y*dwdW[j]);
            double dmxdt = fx*(dwdW[j]*cdist + x*dcdist_dt +
                    k[2]*da1dt + k[3]*(dr2dt + 4*x*dwdW[j]));
            double dmydt = fy*(dwdW[j+3]*cdist + y*dcdist_dt +
                 k[2]*(dr2dt + 4*y*dwdW[j+3]) + k[3]*da1dt);
            cvmSet(dpdw,0,j,dmxdt);
            cvmSet(dpdw,1,j,dmydt);
        }
        /** d[mx my] / d[q0 q1 q2 q4] **/
        double dx0dr[] =
        {
           X*dRdr[0] + Y*dRdr[1] + Z*dRdr[2],
           X*dRdr[9] + Y*dRdr[10] + Z*dRdr[11],
           X*dRdr[18] + Y*dRdr[19] + Z*dRdr[20],
           X*dRdr[27] + Y*dRdr[28] + Z*dRdr[29]
        };
        double dy0dr[] =
        {
           X*dRdr[3] + Y*dRdr[4] + Z*dRdr[5],
           X*dRdr[12] + Y*dRdr[13] + Z*dRdr[14],
           X*dRdr[21] + Y*dRdr[22] + Z*dRdr[23],
           X*dRdr[30] + Y*dRdr[31] + Z*dRdr[32]
        };
        double dz0dr[] =
        {
           X*dRdr[6] + Y*dRdr[7] + Z*dRdr[8],
           X*dRdr[15] + Y*dRdr[16] + Z*dRdr[17],
           X*dRdr[24] + Y*dRdr[25] + Z*dRdr[26],
           X*dRdr[33] + Y*dRdr[34] + Z*dRdr[35]
        };
//        CvMat* t1=cvCreateMat(3,3,CV_32FC1);
//        CvMat* dPpdR=cvCreateMat(3,9,CV_32FC1);
//        cvZero(dPpdR);
//        cvmSet(dPpdR,0,0,X);cvmSet(dPpdR,0,1,Y);cvmSet(dPpdR,0,2,Z);
//        cvmSet(dPpdR,1,3,X);cvmSet(dPpdR,1,4,Y);cvmSet(dPpdR,1,5,Z);
//        cvmSet(dPpdR,2,6,X);cvmSet(dPpdR,2,7,Y);cvmSet(dPpdR,2,8,Z);
//        cvMatMul(dPpdR,&_dRdr,t1);
//        CvMat* dpdPp=cvCreateMat(2,3,CV_32FC1);
//        CvMat* t2 = cvCreateMat(2,3,CV_32FC1);
//        cvmSet(dpdPp,0,0,1/zp);cvmSet(dpdPp,0,1,0);cvmSet(dpdPp,0,2,-xp/(zp*zp));
//        cvmSet(dpdPp,1,0,0);cvmSet(dpdPp,1,1,1/zp);cvmSet(dpdPp,1,2,-yp/(zp*zp));
//        cvMatMul(dpdPp,t1,t2);
//        CvMat* dcdistdp=cvCreateMat(1,2,CV_32FC1);
//        cvmSet(dcdistdp,0,0,k[0]*2*x+k[1]*4*r2*x);
//        cvmSet(dcdistdp,0,1,k[0]*2*y+k[1]*4*r2*y);
//        CvMat* da1dp= cvCreateMat(1,2,CV_32FC1);
//        cvmSet(da1dp,0,0,2*y);
//        cvmSet(da1dp,0,1,2*x);
//        CvMat* da2dp= cvCreateMat(1,2,CV_32FC1);
//        cvmSet(da2dp,0,0,6*x);
//        cvmSet(da2dp,0,1,2*y);
//        CvMat* da3dp= cvCreateMat(1,2,CV_32FC1);
//        cvmSet(da3dp,0,0,2*x);
//        cvmSet(da3dp,0,1,6*y);
//        CvMat* dPddp=cvCreateMat(2,2,CV_32FC1);
//        cvmSet(dPddp,0,0,cdist+x*cvmGet(dcdistdp,0,0)+k[2]*cvmGet(da1dp,0,0)+k[3]*cvmGet(da2dp,0,0));
//        cvmSet(dPddp,0,1,x*cvmGet(dcdistdp,0,1)+k[2]*cvmGet(da1dp,0,1)+k[3]*cvmGet(da2dp,0,1));
//        cvmSet(dPddp,1,0,y*cvmGet(dcdistdp,0,0)+k[2]*cvmGet(da2dp,0,0)+k[3]*cvmGet(da3dp,0,0));
//        cvmSet(dPddp,1,1,cdist+y*cvmGet(dcdistdp,0,1)+k[2]*cvmGet(da2dp,0,1)+k[3]*cvmGet(da3dp,0,1));
//        CvMat *t3=cvCreateMat(2,3,CV_32FC1);
//        cvMatMul(dPddp,t2,t3);
//        CvMat *dmdPd=cvCreateMat(2,2,CV_32FC1);
//        cvmSet(dmdPd,0,0,fx);cvmSet(dmdPd,0,1,0);
//        cvmSet(dmdPd,1,0,0);cvmSet(dmdPd,1,1,fy);
//        CvMat *dmdRrod=cvCreateMat(2,3,CV_32FC1);
//        cvMatMul(dmdPd,t3,dmdRrod);

        for( j = 0; j < 4; j++ )
        {
           double dxdr = z*(dx0dr[j] - x*dz0dr[j]);
           double dydr = z*(dy0dr[j] - y*dz0dr[j]);
           double dr2dr = 2*x*dxdr + 2*y*dydr;
           double dcdist_dr = k[0]*dr2dr + 2*k[1]*r2*dr2dr;
           double da1dr = 2*(x*dydr + y*dxdr);
           double dmxdr = fx*(dxdr*cdist + x*dcdist_dr +
                    k[2]*da1dr + k[3]*(dr2dr + 2*x*dxdr));
           double dmydr = fy*(dydr*cdist + y*dcdist_dr +
                    k[2]*(dr2dr + 2*y*dydr) + k[3]*da1dr);
           dpdr_p[j] = dmxdr;
           dpdr_p[dpdr_step+j] = dmydr;
        }
    }
    cvReleaseMat(&t_vec);
    cvReleaseMat(&q_vec);
    cvReleaseMat(&r_vec);
}

void CSlam::InverseDepth2Depth(double x, double y, double z, double theta, double phi, double rho, double *outx, double*outy, double *outz)
{
   CvMat * m;
   m=cvCreateMat(3,1,CV_32FC1);
   //CvMat *obj;
   //obj=cvCreateMat(4,3,CV_32FC1);
   cvmSet(m,0,0,cos(theta)*sin(phi));//(*It)->theta)*sin((*It)->phi));
   cvmSet(m,1,0,-sin(theta));//(*It)->theta));
   cvmSet(m,2,0,cos(theta)*cos(phi));//cos((*It)->theta)*cos((*It)->phi));
   cvNormalize( m, m);
   *outx=x +cvmGet(m,0,0)/rho;
   *outy=y +cvmGet(m,1,0)/rho;
   *outz=z +cvmGet(m,2,0)/rho;
    cvReleaseMat(&m);
    //cvReleaseMat(&obj);
}
void CSlam::getTransRot(CvMat *state, CvMat *t, CvMat *q, CvMat *Rot)
{
    float *s;
    s = state->data.fl;
    cvmSet(t,0,0,s[0]);
    cvmSet(t,1,0,s[1]);
    cvmSet(t,2,0,s[2]);
    cvmSet(q,0,0,s[3]);
    cvmSet(q,1,0,s[4]);
    cvmSet(q,2,0,s[5]);
    cvmSet(q,3,0,s[6]);
    cvmSet(Rot,0,0,s[3]*s[3]+s[4]*s[4]-s[5]*s[5]-s[6]*s[6]);
    cvmSet(Rot,1,0,2*(s[4]*s[5]+s[3]*s[6]));
    cvmSet(Rot,2,0,2*(s[6]*s[4]-s[3]*s[5]));

    cvmSet(Rot,0,1,2*(s[4]*s[5]-s[3]*s[6]));
    cvmSet(Rot,1,1,s[3]*s[3]-s[4]*s[4]+s[5]*s[5]-s[6]*s[6]);
    cvmSet(Rot,2,1,2*(s[5]*s[6]+s[3]*s[4]));

    cvmSet(Rot,0,2,2*(s[6]*s[4]+s[3]*s[5]));
    cvmSet(Rot,1,2,2*(s[5]*s[6]-s[3]*s[4]));
    cvmSet(Rot,2,2,s[3]*s[3]-s[4]*s[4]-s[5]*s[5]+s[6]*s[6]);
}
void CSlam::projectAllPoints(CvMat *state)
{
//    int i = 0;
    float *fstate;
    fstate = state->data.fl;
    int step = state->step/sizeof(fstate[0]);

    double x,y,z,theta,phi,rho;
    double ox,oy,oz;

    fstate+=(modelSD*step);
    CvPoint2D64f proj;
    CvMat* dpdr=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdt=cvCreateMat(2,3,CV_32FC1);
    CvMat* dpdf=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdc=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdk=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdw=cvCreateMat(2,6,CV_32FC1);
//    CvMat *dpdt_ =cvCreateMat(2,6,CV_32FC1);
//    cvZero(dpdt_);
//    CvMat *dpdr_ =cvCreateMat(2,6,CV_32FC1);
//    cvZero(dpdr_);
    CvMat *dpdW = cvCreateMat(2,6,CV_32FC1);
    cvZero(dpdW);
    if(MD>0) cvZero(H);
    int j=0;//predicion vector index
    int nstate = modelSD;
    int s=0;
    for (unsigned int i = 0 ; i < visible.size(); i++)
    {
        x=*fstate;
        fstate++;
        y=*fstate;
        fstate++;
        z=*fstate;
        fstate++;
        theta=*fstate;
        fstate++;
        phi=*fstate;
        fstate++;
        rho=*fstate;
        fstate++;
        InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
        CvPoint3D64f p= cvPoint3D64f(ox,oy,oz);
        ProjectPoints3(&p,state, IntrinsicParam,DistortionParam,
                            &proj,  dpdr,dpdt, dpdf,
                            dpdc, dpdk , dpdw);
        prediction[j]=proj.x;
        prediction[j+1]=proj.y;

//        for (int k = 0; k<3; k++)
//        {
//            cvmSet(dpdt_,0,2*k,cvmGet(dpdt,0,k));
//            cvmSet(dpdt_,1,2*k,cvmGet(dpdt,1,k));
//            cvmSet(dpdr_,0,2*k,cvmGet(dpdr,0,k));
//            cvmSet(dpdr_,1,2*k,cvmGet(dpdr,1,k));
//        }

        fillInvParamDeriv(dpdw, dpdW, theta,phi,rho);
        if ((visible.at(i)==true)){
            CopyMat(dpdt,H,s,0);
            CopyMat(dpdr,H,s,3);
            CopyMat(dpdW,H,s,nstate);
            s+=2;
        }
        CopyMat(dpdt,HFull,j,0);
        CopyMat(dpdr,HFull,j,3);
        CopyMat(dpdW,HFull,j,nstate);

        nstate+=6;
        j+=2;
    }
    cvReleaseMat(&dpdt);
    cvReleaseMat(&dpdr);
    cvReleaseMat(&dpdf);
    cvReleaseMat(&dpdc);
    cvReleaseMat(&dpdk);
    cvReleaseMat(&dpdw);
    cvReleaseMat(&dpdW);

}
void CSlam::fillInvParamDeriv(CvMat* dpdw, CvMat* dpdW, float theta, float phi, float rho)
{
      for (int k = 0; k<3; k++)
        {
            cvmSet(dpdW,0,k,cvmGet(dpdw,0,k));
            cvmSet(dpdW,1,k,cvmGet(dpdw,1,k));
        }

        float s_th=sin(theta);
        float c_th=cos(theta);
        float s_ph=sin(phi);
        float c_ph=cos(phi);
        /** \f$ \frac{dp}{d\theta}= \frac{dx}{dx}(\frac{cos(\theta)}{\rho})\f$ **/
        cvmSet(dpdW,0,3,cvmGet(dpdw,0,0)*(-s_th*s_ph)/rho+
                          cvmGet(dpdw,0,1)*(-c_th     )/rho+
                          cvmGet(dpdw,0,2)*(-s_th*c_ph)/rho);
        /** \f$ \frac{dp}{d\phi}= \frac{dx}{dX}  (\frac{cos(\phi)}{\rho}) +
        \frac{dx}{dZ} (\frac{-sin(\phi)}{\rho})\f$ **/
        cvmSet(dpdW,0,4,cvmGet(dpdw,0,0)*(c_th*c_ph)/rho+
                          cvmGet(dpdw,0,1)*(0.0    )/rho+
                          cvmGet(dpdw,0,2)*(-c_th*s_ph)/rho);

        /** \f$ \frac{dp}{d\rho}=\frac{dx}{dX} (\frac{-sin(\phi)}{\rho^2}) +
        * \frac{dx}{dY} (\frac{sin(\theta)}{\rho^2}) +
        * \frac{dx}{dZ} (\frac{-cos(\phi)}{\rho^2}) \f$ **/
        cvmSet(dpdW,0,5,cvmGet(dpdw,0,0)*(-c_th*s_ph)/(rho*rho)+
                          cvmGet(dpdw,0,1)*(s_th     )/(rho*rho)+
                          cvmGet(dpdw,0,2)*(-c_th*c_ph)/(rho*rho));

        /** \f$ \frac{dy}{d\theta}= \frac{dy}{dX}(\frac{cos(\theta)}{\rho})\f$ **/
        cvmSet(dpdW,1,3,cvmGet(dpdw,1,0)*(-s_th*s_ph)/rho+
                           cvmGet(dpdw,1,1)*(-c_th     )/rho+
                           cvmGet(dpdw,1,2)*(-s_th*c_ph)/rho);

        /** \f$ \frac{dy}{d\phi}=\frac{dy}{dX}(\frac{cos(\phi)}{\rho}) +
        \frac{dy}{dZ}(\frac{-sin(\phi)}{\rho}) \f$ **/
        cvmSet(dpdW,1,4,cvmGet(dpdw,1,0)*(c_th*c_ph)/rho+
                            cvmGet(dpdw,1,1)*(0.0    )/rho+
                            cvmGet(dpdw,1,2)*(-c_th*s_ph)/rho);

        /** \f$ \frac{dp}{d\rho}=\frac{dy}{dX} (\frac{-sin(\phi)}{\rho^2}) +
        * \frac{dy}{dX} (\frac{sin(\theta)}{\rho^2}) +
        * \frac{dy}{dZ} (\frac{-cos(\phi)}{\rho^2}) \f$ **/
        cvmSet(dpdW,1,5,cvmGet(dpdw,1,0)*(-c_th*s_ph)/(rho*rho)+
                            cvmGet(dpdw,1,1)*(s_th     )/(rho*rho)+
                            cvmGet(dpdw,1,2)*(-c_th*c_ph)/(rho*rho));

}
void CSlam::InverseParam(CvMat** h,CvPoint pto)
{
     CvMat *pixel;
     pixel=cvCreateMat(3,1,CV_32FC1);
     cvmSet(pixel,0,0,pto.x);
     cvmSet(pixel,1,0,pto.y);
     cvmSet(pixel,2,0,1);
     CvMat *inv;
     inv=cvCreateMat(3,3,CV_32FC1);
     cvInvert(IntrinsicParam,inv,CV_SVD);
     CvMat *temp1;
     temp1=cvCreateMat(3,1,CV_32FC1);
     cvGEMM(inv,pixel,1,NULL,0,temp1);//CUIDADO ESTA A 0 TRANSLATION
    CvMat *_R = cvCreateMat( 3, 3, CV_32FC1 );
    CvMat *t = cvCreateMat(3,1,CV_32FC1);
    CvMat* q=cvCreateMat(4,1,CV_32FC1);
    getTransRot(X,t,q,_R);

    cvInvert(_R,inv);
    CvMat *t1= cvCreateMat(3,1,CV_32FC1);
    cvGEMM(inv,temp1,1,NULL,0,t1);
    cvmSet(*h,0,0,cvmGet(t,0,0));
    cvmSet(*h,1,0,cvmGet(t,1,0));
    cvmSet(*h,2,0,cvmGet(t,2,0));
    cvmSet(*h,3,0,atan2(-cvmGet(t1,1,0),sqrt(cvmGet(t1,0,0)*cvmGet(t1,0,0)+cvmGet(t1,2,0)*cvmGet(t1,2,0))));
    cvmSet(*h,4,0,atan2(cvmGet(t1,0,0),cvmGet(t1,2,0)));
    cvmSet(*h,5,0,1./depth);

    cvReleaseMat(&t);
    cvReleaseMat(&q);
	cvReleaseMat(&pixel);
	cvReleaseMat(&inv);
	cvReleaseMat(&temp1);
	cvReleaseMat(&_R);
	cvReleaseMat(&t1);
}
/** imprime una matriz **/
void CSlam::printMat(CvMat *m)
{
  std::cout<<"------------------------------------------------"<<std::endl;
  if(m!=0){
    for (int i= 0;i <m->rows;i++)
      {
        for(int j =0; j<m->cols;j++)
          {
            std::cout<<" "<<cvmGet(m,i,j);
          }
        std::cout<<std::endl;
      }
  }
 std::cout<<"------------------------------------------------"<<std::endl;
}
void CSlam::Draw(IplImage *framecopy)
{
       //int rvis=0;
       Disp_out( framecopy);
   for (unsigned int i = 0 ; i < visible.size(); i++)
    {
        sprintf(strID,"%d",i);
        if (visible.at(i)==true){
            if(BestH.size()==visible.size()){
            if (BestH[i]==1){
                //visible y pasa filtro verde
            cvPutText (framecopy,strID,cvPoint((int)measurement[2*i],(int)measurement[2*i+1]), &font, cvScalar(0,255,0));
            }else
            {
                //visible y no pasa filtro rojo
            cvPutText (framecopy,strID,cvPoint((int)measurement[2*i],(int)measurement[2*i+1]), &font, cvScalar(0,0,255));
            }
            }else{
                //visible y pasa filtro verde
            cvPutText (framecopy,strID,cvPoint((int)measurement[2*i],(int)measurement[2*i+1]), &font, cvScalar(0,255,0));

            }
        }else
        {
            //No visible azul
            cvPutText (framecopy,strID,cvPoint((int)measurement[2*i],(int)measurement[2*i+1]), &font, cvScalar(255,0,0));
        }
        //prediccion en amarillo
        cvPutText (framecopy,strID,cvPoint((int)prediction[2*i],(int) prediction[2*i+1]), &font, cvScalar(0,255,255));
    }
    IplImage* t = cvCreateImage(cvSize(P->height,P->width),8,1);
    cvConvertScale(P,t,255,0);
    cvSaveImage("slam_p.tif",t);
    cvShowImage("SLAM", framecopy );
    cvSaveImage("slam.jpg",framecopy);
}

void CSlam::PrintKalman()
{
  std::cout<<"--------------------Kalman-----------------------"<<std::endl;
  std::cout<<"Estados: "<<SD<<" Medidas: "<<MD<<" Control: "<<CD<<std::endl;
  std::cout<<"------------------------------------------------"<<std::endl;
  std::cout<<"state_pre"<<std::endl;
  printMat(Xp);
  std::cout<<"state_post"<<std::endl;
  printMat(X);
  std::cout<<"transition_matrix"<<std::endl;
  printMat(A);
  std::cout<<"control_matrix"<<std::endl;
  printMat(B);
  std::cout<<"measurement_matrix"<<std::endl;
  printMat(H);
  std::cout<<"process_noise_cov"<<std::endl;
  printMat(Q);
  std::cout<<"measurement_noise_cov"<<std::endl;
  printMat(R);
  std::cout<<"error_cov_pre"<<std::endl;
  printMat(Pp);
  std::cout<<"error_cov_post"<<std::endl;
  printMat(P);
  std::cout<<"kalman gain"<<std::endl;
  printMat(K);

}
void CSlam::CopyMat(CvMat* o, CvMat* d, int row,int col)
{
    float *of;
    float *df;
    of = o->data.fl;
    df = d->data.fl;
    int ostep = o->step/sizeof(of[0]);
    int dstep = d->step/sizeof(df[0]);
    int dfrow,dfcol;
    for (int i=0; i<o->height; i++){
        for (int j=0; j<o->width; j++){
            dfrow=row+i;
            dfcol=col+j;
            df[dfrow*dstep+dfcol]=of[i*ostep+j];
        }
    }
}
float CSlam::ic_test(int pointNum, float measx, float measy)
{
    float *fstate;
    fstate = Xp->data.fl;

    double x,y,z,theta,phi,rho;
    double ox,oy,oz;

    CvPoint2D64f proj;
    CvMat* dpdr=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdt=cvCreateMat(2,3,CV_32FC1);
    CvMat* dpdf=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdc=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdk=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdw=cvCreateMat(2,6,CV_32FC1);
    CvMat *dpdW = cvCreateMat(2,6,CV_32FC1);
    cvZero(dpdW);

    //IC_Test var
    CvMat* innov;
    CvMat* C;
    CvMat* H_;
    CvMat* P_;
    CvMat* R_;
    CvMat* T2;
    CvMat* T3;
    CvMat* T4;
    int fdims=6;
    innov=cvCreateMat(2,1,CV_32FC1);
    C=cvCreateMat(2,2,CV_32FC1);
    H_=cvCreateMat(2,modelSD+fdims,CV_32FC1);
    P_=cvCreateMat(modelSD+fdims,modelSD+fdims,CV_32FC1);
    R_=cvCreateMat(2,2,CV_32FC1);
    T2=cvCreateMat(2,modelSD+fdims,CV_32FC1);
    T3=cvCreateMat(2,1,CV_32FC1);
    T4=cvCreateMat(1,1,CV_32FC1);

    int s_i=modelSD+pointNum*6;//
    fstate+=s_i;
    x=*fstate;        fstate++;
    y=*fstate;        fstate++;
    z=*fstate;        fstate++;
    theta=*fstate;    fstate++;
    phi=*fstate;      fstate++;
    rho=*fstate;      fstate++;

    InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
    CvPoint3D64f p= cvPoint3D64f(ox,oy,oz);
    ProjectPoints3(&p, Xp,IntrinsicParam,DistortionParam,
                        &proj,  dpdr,dpdt, dpdf,
                        dpdc, dpdk , dpdw);

    fillInvParamDeriv(dpdw, dpdW, theta,phi,rho);

    CopyMat(dpdt,H_,0,0);
    CopyMat(dpdr,H_,0,3);
    CopyMat(dpdW,H_,0,modelSD);

    for (int col=0;col<modelSD;col++)
        for (int row=0; row<modelSD;row++)
        {
          cvmSet(P_,row,col,cvmGet(Pp,row,col));
        }
    for (int col=0;col<fdims;col++)
        for (int row=0; row<fdims;row++)
        {
          cvmSet(P_,row+modelSD,col+modelSD,cvmGet(Pp,s_i+row,s_i+col));
        }
    for (int col=0;col<modelSD;col++)
      for (int row=0; row<fdims;row++)
        {
          cvmSet(P_,row+modelSD,col,cvmGet(Pp,s_i+row,col));
        }
    for (int col=0;col<fdims;col++)
      for (int row=0; row<modelSD;row++)
        {
          cvmSet(P_,row,col+modelSD,cvmGet(Pp,row,col+s_i));
        }
    cvSetIdentity(R_,cvRealScalar(MEAS_NOISE_COV));

    /* temp2 = H*P'(k) */
    cvMatMulAdd( H_,P_,0,T2 );
    /* C = temp2*Ht + R */
    cvGEMM( T2,H_, 1,R_, 1, C, CV_GEMM_B_T );

    /* temp5 = z(k) - H*x'(k) */

    cvmSet(innov,0,0,measx-proj.x);
    cvmSet(innov,1,0,measy-proj.y);

    /* temp3=C^-1 * (z-h)*/
    cvSolve( C,innov,T3, CV_SVD );
    /* temp2 = (z-h) temp3t */
    cvGEMM(innov,T3,1,NULL,0,  T4 ,CV_GEMM_A_T);

    if (cvmGet(T4,0,0)>2.2)//18DOF 6DOF=10.864940
    {
        std::cout<<"test ko : "<<cvmGet(T4,0,0)<<" xi: "<<10.864940<<" id "<<pointNum<<std::endl;
    }else{
        std::cout<<"test OK: "<<cvmGet(T4,0,0)<<" xi: "<<10.864940<<" id "<<pointNum<<std::endl;
    }

    double distance= cvmGet(T4,0,0);
    cvReleaseMat(&innov);
    cvReleaseMat(&C);
    cvReleaseMat(&H_);
    cvReleaseMat(&P_);
    cvReleaseMat(&R_);
    cvReleaseMat(&T2);
    cvReleaseMat(&T3);
    cvReleaseMat(&T4);

    cvReleaseMat(&dpdt);
    cvReleaseMat(&dpdr);
    cvReleaseMat(&dpdf);
    cvReleaseMat(&dpdc);
    cvReleaseMat(&dpdk);
    cvReleaseMat(&dpdw);
    cvReleaseMat(&dpdW);

    return distance;
}
void CSlam::NormalizeQuatCov(CvMat *state, CvMat *_P)
{
    float q[4];
    for (int i = 3; i< 7; i++)
    {
        q[i-3]=cvmGet(state,i,0);
    }
    CvMat * _M = cvCreateMat(SD,SD,CV_32FC1);
    cvSetIdentity (_M);
    CvMat *M = cvCreateMatHeader(4,4,CV_32FC1);
    cvGetSubRect(_M,M,cvRect(3,3,4,4));

    double sq = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    double sq32 = pow(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3],3/2);

    cvmSet(M,0,0,1/sq-(q[0]*q[0]/sq32));
    cvmSet(M,0,1,-q[0]*q[1]/sq32);
    cvmSet(M,0,2,-q[0]*q[2]/sq32);
    cvmSet(M,0,3,-q[0]*q[3]/sq32);

    cvmSet(M,1,0,-q[1]*q[0]/sq32);
    cvmSet(M,1,1,1/sq-(q[1]*q[1]/sq32));
    cvmSet(M,1,2,-q[1]*q[2]/sq32);
    cvmSet(M,1,3,-q[1]*q[3]/sq32);

    cvmSet(M,2,0,-q[2]*q[0]/sq32);
    cvmSet(M,2,1,-q[2]*q[1]/sq32);
    cvmSet(M,2,2,1/sq-(q[2]*q[2]/sq32));
    cvmSet(M,2,3,-q[2]*q[3]/sq32);

    cvmSet(M,3,0,-q[3]*q[0]/sq32);
    cvmSet(M,3,1,-q[3]*q[1]/sq32);
    cvmSet(M,3,2,-q[3]*q[2]/sq32);
    cvmSet(M,3,3,1/sq-(q[3]*q[3]/sq32));

    cvMatMul(_M,_P,temp1);
     cvGEMM(temp1,_M,1,NULL,0,_P,CV_GEMM_B_T);
    cvReleaseMat (&_M);
    cvReleaseMatHeader(&M);

}
/**Adds new points to covariance matrix **/
void CSlam::AddPointToCovMatrix(double x, double y)
{
     CvMat *pixel;
     pixel=cvCreateMat(3,1,CV_32FC1);
     cvmSet(pixel,0,0,x);
     cvmSet(pixel,1,0,y);
     cvmSet(pixel,2,0,1);
     CvMat *inv;
     inv=cvCreateMat(3,3,CV_32FC1);
     cvInvert(IntrinsicParam,inv,CV_SVD);
     CvMat *temp1;
     temp1=cvCreateMat(3,1,CV_32FC1);
     cvGEMM(inv,pixel,1,NULL,0,temp1);//CUIDADO ESTA A 0 TRANSLATION
    CvMat *_R = cvCreateMat( 3, 3, CV_32FC1 );
    CvMat *t = cvCreateMat(3,1,CV_32FC1);
    CvMat* q=cvCreateMat(4,1,CV_32FC1);
    getTransRot(X,t,q,_R);

    CvMat *invRot;
     invRot=cvCreateMat(3,3,CV_32FC1);
    cvInvert(_R,invRot);
    CvMat *t1= cvCreateMat(3,1,CV_32FC1);
    cvGEMM(invRot,temp1,1,NULL,0,t1);
    double hx = cvmGet(t1,0,0);
    double hy = cvmGet(t1,1,0);
    double hz = cvmGet(t1,2,0);

    CvMat* dydt=cvCreateMat(6,3,CV_32FC1);
    cvZero(dydt);
    CvMat* dydq=cvCreateMat(6,4,CV_32FC1);
    cvZero(dydq);
    CvMat* dydh=cvCreateMat(6,3,CV_32FC1);
    cvZero(dydh);
    /**dy/dt = [diag (1 1 1);zeros(3,3) ]**/
    for (int i=0;i<3;i++)
        cvmSet(dydt,i,i,1);
    /**dy/dq = dy/dm * dh/dR * dR/dq **/
    CvMat *dy_by_dm= cvCreateMat(2,3,CV_32FC1);
    //derivada de theta respecto del mx
    double dtan = 1/(1+((-hy)/sqrt(hx*hx+hz*hz))*((-hy)/sqrt(hx*hx+hz*hz)));

    cvmSet(dy_by_dm,0,0,dtan*(1/sqrt(hx*hx+hz*hz))*-1);
    cvmSet(dy_by_dm,0,1,dtan*(-hy/sqrt(hx*hx+hz*hz))*(1/pow(hx*hx+hz*hz,3/2))*hx);
    cvmSet(dy_by_dm,0,2,dtan*(-hy/sqrt(hx*hx+hz*hz))*(1/pow(hx*hx+hz*hz,3/2))*hz);
     cvmSet(dy_by_dm,1,0,(1/(1+(hx/hz)*(hx/hz))*(1/hz)));
     cvmSet(dy_by_dm,1,1,0);
     cvmSet(dy_by_dm,1,2,-(1/(1+(hx/hz)*(hx/hz)))*hx*(1/(hz*hz)));
    CvMat *dm_by_dR = cvCreateMat(3,9,CV_32FC1);
    cvZero(dm_by_dR);
    cvmSet(dm_by_dR,0,0,cvmGet(temp1,0,0));cvmSet(dm_by_dR,0,3,cvmGet(temp1,1,0)),cvmSet(dm_by_dR,0,6,cvmGet(temp1,2,0));
    cvmSet(dm_by_dR,1,1,cvmGet(temp1,0,0));cvmSet(dm_by_dR,1,4,cvmGet(temp1,1,0)),cvmSet(dm_by_dR,1,7,cvmGet(temp1,2,0));
    cvmSet(dm_by_dR,2,2,cvmGet(temp1,0,0));cvmSet(dm_by_dR,2,5,cvmGet(temp1,1,0)),cvmSet(dm_by_dR,2,8,cvmGet(temp1,2,0));
    CvMat *_dR_by_dq=cvCreateMat(9,4,CV_32FC1);
    dR_by_dq(_dR_by_dq,q);
    CvMat *_temp = cvCreateMat(3,4,CV_32FC1);
    cvMatMul(dm_by_dR,_dR_by_dq,_temp);

    CvMat * _dy_dq = cvCreateMatHeader(2,4,CV_32FC1);
    cvGetSubRect(dydq,_dy_dq, cvRect(0, 3, 4,2));
    cvMatMul(dy_by_dm,_temp,_dy_dq);
    /**dy/duv =dy/r^-1 * dr^-1/df^-1  **/

    CvMat *dhduv= cvCreateMat(3,2,CV_32FC1);
    CvMat *_dhduv = cvCreateMat(3,3,CV_32FC1);
    cvMatMul(invRot,inv,_dhduv);
    cvmSet(dhduv,0,0,cvmGet(_dhduv,0,0)); cvmSet(dhduv,0,1,cvmGet(_dhduv,0,1));
    cvmSet(dhduv,1,0,cvmGet(_dhduv,1,0)); cvmSet(dhduv,1,1,cvmGet(_dhduv,1,1));
    cvmSet(dhduv,2,0,cvmGet(_dhduv,2,0)); cvmSet(dhduv,2,1,cvmGet(_dhduv,2,1));

    CvMat * _dy_dh = cvCreateMatHeader(2,2,CV_32FC1);
    cvGetSubRect(dydh,_dy_dh, cvRect(0, 3, 2,2));
    /** dy/drho = 1 **/
    cvmSet (dydh,5,2,1);

    cvMatMul(dy_by_dm,dhduv,_dy_dh);

     CvMat*A = cvCreateMat(6,SD,CV_32FC1);
     CvMat*B = cvCreateMat(SD,6,CV_32FC1);
     CvMat*C = cvCreateMat(6,6,CV_32FC1);
     CvMat *_P= cvCreateMat(SD+3,SD+3,CV_32FC1);
     CvMat *_temp2= cvCreateMatHeader(SD+3,SD+3,CV_32FC1);
     cvGetSubRect(PMem, _temp2,cvRect(0,0,SD+3,SD+3));
     cvCopy(_temp2,_P);
     cvmSet(_P,SD,SD,4);
     cvmSet(_P,SD+1,SD+1,4);///FIXME CAMBIAR AQUI LOS VALORE DE COVARIANZA
     cvmSet(_P,SD+2,SD+2,0.00005);
     cvReleaseMatHeader(&_temp2);

     CvMat *_PNew = cvCreateMatHeader(SD+6,SD+6,CV_32FC1);
     cvGetSubRect(PMem, _PNew ,cvRect(0,0,SD+6,SD+6));

     CvMat *_J=cvCreateMat(SD+6,SD+3,CV_32FC1);
     cvSetIdentity(_J);
     CopyMat(dydt,_J,SD,0);
     CopyMat(dydq,_J,SD,4);
     CopyMat(dydh,_J,SD,SD);
     CvMat* _temp3=cvCreateMat(SD+6,SD+3,CV_32FC1);
     cvMatMul(_J,_P,_temp3);
     cvGEMM(_temp3,_J,1,NULL,0,_PNew,CV_GEMM_B_T);
     for (int j= 0 ; j<_PNew->width;j++)
        if(cvmGet(_PNew,j,j)==0) cvmSet(_PNew,j,j,0.00000001);
//     cvGEMM(dydh,P,1,NULL,0,A);
//
//     CvMat *temp = cvCreateMat(6,3,CV_32FC1);
//     CvMat *Rs = cvCreateMat(3,3,CV_32FC1);
//     cvZero(Rs);
//     cvmSet(Rs,0,0,MEAS_NOISE_COV);//2 pix sigma
//     cvmSet(Rs,1,1,MEAS_NOISE_COV);//
//     cvmSet(Rs,2,2,0.25);//r=0.5 r^2=0.25 ro=0.1 d=10
//     cvGEMM(dydh,Rs,1,NULL,0,temp);
//     cvGEMM(temp,dydh,1,NULL,0,C,CV_GEMM_B_T);
     SD+=6;///IMP: SD+=6
     MemMat2WorkMat();
//     CopyMat(A,P,SD-6,0);
//     CopyMat(B,P,0,SD-6);
//     CopyMat(C,P,SD-6,SD-6);
    cvReleaseMat(&A);
    cvReleaseMat(&B);
    cvReleaseMat(&C);
    cvReleaseMat(&dydq);
    cvReleaseMat(&dydt);
    cvReleaseMat(&dydh);
    cvReleaseMat(&t);
    cvReleaseMat(&q);
    cvReleaseMat(&_R);
}
void CSlam::Disp_out(IplImage *framecopy)
{
  char ndisp[100];
  sprintf(ndisp,"disp.txt");
  DispFile.open(ndisp);
  CvMat *temp,*temp2,*temp3;
  temp=cvCreateMatHeader(6,6,CV_32FC1);
  CvMat *_temp = cvCreateMat(6,6,CV_32FC1);
  temp2=cvCreateMat(3,6,CV_32FC1);
  temp3=cvCreateMat(3,3,CV_32FC1);
  CvMat* vect=cvCreateMat (6,1,CV_32FC1);
  CvMat* res6=cvCreateMat (6,1,CV_32FC1);
  CvMat* vect2=cvCreateMat(1,6,CV_32FC1);
  CvPoint2D64f proj ;
//  CvMat* m = cvCreateMat(3,1,CV_32FC1);
  float *fstate;
  fstate= X->data.fl;
  float xc,yc,zc,theta,phi,rho;

    for (unsigned int i = 0 ; i < visible.size(); i++)
    {
      if(visible[i]==true)
	  {
          xc=fstate[modelSD+6*i];
          yc=fstate[modelSD+6*i+1];
          zc=fstate[modelSD+6*i+2];
          theta=fstate[modelSD+6*i+3];
          phi=fstate[modelSD+6*i+4];
          rho=fstate[modelSD+6*i+5];

          cvGetSubRect( P,temp,cvRect(modelSD+i*6,modelSD+i*6,6,6) );

          for (int part=0; part<40; part++){
              cvmSet(vect,0,0,randomVector(-12.6,12.6));
              cvmSet(vect,1,0,randomVector(-12.6,12.6));
              cvmSet(vect,2,0,randomVector(-12.6,12.6));
              cvmSet(vect,3,0,randomVector(-12.6,12.6));
              cvmSet(vect,4,0,randomVector(-12.6,12.6));
              cvmSet(vect,5,0,randomVector(-12.6,12.6));
              cvZero(_temp);
              Cholesky(temp,_temp);
              cvMatMul(_temp,vect,res6);
              //cvCopy(vect,res6);
              //cvZero(res6);
              //printMat(temp);

              double ox, oy, oz;
              InverseDepth2Depth( cvmGet(res6,0,0)+xc,
                                  cvmGet(res6,1,0)+yc,
                                  cvmGet(res6,2,0)+zc,
                                  cvmGet(res6,3,0)+theta,
                                  cvmGet(res6,4,0)+phi,
                                  cvmGet(res6,5,0)+rho,
                                  &ox,&oy,&oz );
              DispFile<<ox<<" ";
              DispFile<<oy<<" ";
              DispFile<<oz<<" ";
              DispFile<<i<<std::endl;
            CvMat* nullmat = 0;
            CvPoint3D64f point = cvPoint3D64f(ox,oy,oz);
            ProjectPoints3 (&point,
                            X,
                            IntrinsicParam,DistortionParam,
                            &proj, nullmat,
                            nullmat,nullmat,
                            nullmat, nullmat ,nullmat);

              if (framecopy != NULL){
                 cvCircle (framecopy,cvPoint((int)proj.x,(int)proj.y),1,cvScalar(0,0,255),1 );
              }

            }
          }
    }
    cvReleaseMatHeader(&temp);
    cvReleaseMat(&temp2);
    cvReleaseMat(&temp3);
    cvReleaseMat(&vect);
    cvReleaseMat(&vect2);
    cvReleaseMat(&res6);
    DispFile.close();
}
float CSlam::randomVector(float max,float min)
{
     float x = min;
     float y = max-min;
return  x+ (y*rand()/(RAND_MAX+1.0));

}
void CSlam::JCBB_incremental(int level,int MaxRama,
                 CvMat *C, CvMat* Cinv, CvMat* H, CvMat* innov)
{
    int pairingsH = 0;
    for (unsigned int i = 0 ; i < H_.size() ; i++ )
    {
        if (H_[i]>0) pairingsH++;
    }
    if (level > maxLevels - 1)
    {
        if ( pairingsH > pairingsBest )
        {
            BestH.clear();
            for (unsigned int i = 0 ; i < H_.size() ; i++)
                BestH.push_back(H_[i]);
            pairingsBest = pairingsH;
        }
    }
    else
    {

        if (pairingsBest < MaxRama)
        {
            //Todavia puedo cancelar algun punto
            H_.push_back(0);
//            if (level<10){
//            for (unsigned int i = 0 ; i < H_.size() ; i++ )
//            {
//                std::cout<<H_[i]<<" ";
//            }
//            std::cout<<level<<std::endl;
//            }
            JCBB_incremental(level+1,MaxRama-1,C,Cinv,H,innov);
            H_.pop_back();
        }
        if (visible[level]){
            H_.push_back(1);//voy por el camino de aceptar la feature
//            if(level<10){
//            for (unsigned int i = 0 ; i < H_.size() ; i++ )
//            {
//                std::cout<<H_[i]<<" ";
//            }
//            std::cout<<level<<std::endl;
//            }
            CvMat *Cnext,*Cinvnext,*Hnext,*innovnext;
            if (joint_compatibility_incremental(H_,measurement, C, Cinv,  H, innov,
                                            &Cnext, &Cinvnext, &Hnext, &innovnext))
            {
                JCBB_incremental(level+1,MaxRama,Cnext, Cinvnext, Hnext, innovnext);
            }
            cvReleaseMat(&Cnext);
            cvReleaseMat(&Cinvnext);
            cvReleaseMat(&Hnext);
            cvReleaseMat(&innovnext);
            H_.pop_back();
        }

    }
}
void CSlam::JCBB( int level, int MaxRama)
{
    int pairingsH = 0;
    for (unsigned int i = 0 ; i < H_.size() ; i++ )
    {
        if (H_[i]>0) pairingsH++;
    }
    if (level > maxLevels - 1)
    {
        if ( pairingsH > pairingsBest )
        {
            BestH.clear();
            for (unsigned int i = 0 ; i < H_.size() ; i++)
                BestH.push_back(H_[i]);
            pairingsBest = pairingsH;
        }
    }
    else
    {
        if (visible[level]){
            H_.push_back(1);//voy por el camino de aceptar la feature
            for (unsigned int i = 0 ; i < H_.size() ; i++ )
            {
                std::cout<<H_[i]<<" ";
            }
            std::cout<<level<<std::endl;
            if (joint_compatibility(H_,measurement))
            {
                JCBB(level+1,MaxRama);
            }
            H_.pop_back();
        }
        if (pairingsBest < MaxRama)
        {
            //Todavia puedo cancelar algun punto
            H_.push_back(0);
            for (unsigned int i = 0 ; i < H_.size() ; i++ )
            {
                std::cout<<H_[i]<<" ";
            }
            std::cout<<level<<std::endl;
            JCBB(level+1,MaxRama-1);
            H_.pop_back();
        }
    }
}
bool CSlam::joint_compatibility(std::vector<int> visible_, std::vector<float> Meas)
{
    float *fstate;
    fstate = Xp->data.fl;

    double x,y,z,theta,phi,rho;
    double ox,oy,oz;

    CvPoint2D64f proj;
    CvMat* dpdr=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdt=cvCreateMat(2,3,CV_32FC1);
    CvMat* dpdf=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdc=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdk=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdw=cvCreateMat(2,6,CV_32FC1);

    CvMat *dpdW = cvCreateMat(2,6,CV_32FC1);
    cvZero(dpdW);

    //IC_Test var
    CvMat* innov;
    CvMat* C;
    CvMat* Cinv;
    CvMat* H_;
    CvMat* P_;
    CvMat* R_;
    CvMat* T2;
    CvMat* T3;
    CvMat* T4;
    int fdims=6;
    int visNum_=0;
    for (unsigned int i = 0 ; i < visible_.size(); i++)
    {
        if ( visible_[i]>0 )
        {
            visNum_++;
        }
    }
    //WARNING se usa visible.size() porque se quiere que se use la P completa
    innov=cvCreateMat(2*visNum_,1,CV_32FC1);
    C=cvCreateMat(2*visNum_,2*visNum_,CV_32FC1);
    Cinv=cvCreateMat(2*visNum_,2*visNum_,CV_32FC1);
    H_=cvCreateMat(2*visNum_,modelSD+fdims*visible.size(),CV_32FC1);
    cvZero(H_);
    P_=cvCreateMat(modelSD+fdims*visible.size(),12+fdims*visible.size(),CV_32FC1);
    R_=cvCreateMat(2*visNum_,2*visNum_,CV_32FC1);
    T2=cvCreateMat(2*visNum_,modelSD+fdims*visible.size(),CV_32FC1);
    T3=cvCreateMat(2*visNum_,1,CV_32FC1);
    T4=cvCreateMat(1,1,CV_32FC1);

    int visibleidx = 0;
    for (unsigned int i=0; i< visible_.size();i++)
    {
        if (visible_[i]>0)
        {
            float *fstate_=fstate;
            int s_i=modelSD+i*6;//

            fstate_+=s_i;
            x=*fstate_;        fstate_++;
            y=*fstate_;        fstate_++;
            z=*fstate_;        fstate_++;
            theta=*fstate_;    fstate_++;
            phi=*fstate_;      fstate_++;
            rho=*fstate_;      fstate_++;

            InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
            CvPoint3D64f p = cvPoint3D64f(ox,oy,oz);
            ProjectPoints3(&p, Xp,IntrinsicParam,DistortionParam,
                                &proj,  dpdr,dpdt, dpdf,
                                dpdc, dpdk , dpdw);
            cvmSet(innov,0+2*visibleidx,0,Meas[2*i]-proj.x);
            cvmSet(innov,1+2*visibleidx,0,Meas[2*i+1]-proj.y);

            fillInvParamDeriv(dpdw, dpdW, theta,phi,rho);

            CopyMat(dpdt,H_,2*visibleidx,0);
            CopyMat(dpdr,H_,2*visibleidx,3);
            CopyMat(dpdW,H_,2*visibleidx,modelSD+i*fdims);

            visibleidx++;
        }//end if visible
    }//end for cada punto
    cvSetIdentity(R_,cvRealScalar(MEAS_NOISE_COV));
    /* temp2 = H*P'(k) */
    cvMatMulAdd( H_,P,0,T2 );
    /* C = temp2*Ht + R */
    cvGEMM( T2,H_, 1,R_, 1, C, CV_GEMM_B_T );

    /* temp5 = z(k) - H*x'(k) */
    // done at the loop upwards ^
    /* temp3=C^-1 * (z-h)*/
    cvInvert(C,Cinv);
    //cvSolve( C,innov,T3, CV_SVD );
    cvMatMul(Cinv,innov,T3);
    /* temp2 = (z-h) temp3t */
    cvGEMM(innov,T3,1,NULL,0,  T4 ,CV_GEMM_A_T);

    bool result;
    double dist = 0;

    for (int i = 0 ; i < visNum_*2;i++){
        dist += cvmGet( innov,i,0)*cvmGet( innov,i,0);
    }
    if (cvmGet(T4,0,0)>125)//18DOF 6DOF=10.864940
    {
        result = false;
    }else{
        result = true;
    }

    cvReleaseMat(&innov);
    cvReleaseMat(&C);
    cvReleaseMat(&Cinv);
    cvReleaseMat(&H_);
    cvReleaseMat(&P_);
    cvReleaseMat(&R_);
    cvReleaseMat(&T2);
    cvReleaseMat(&T3);
    cvReleaseMat(&T4);

    cvReleaseMat(&dpdt);
    cvReleaseMat(&dpdr);
    cvReleaseMat(&dpdf);
    cvReleaseMat(&dpdc);
    cvReleaseMat(&dpdk);
    cvReleaseMat(&dpdw);
    cvReleaseMat(&dpdW);

      return result;
}
/**
 * \fn joint_compatibility_incremental
 * \param visible_ vector of 1,0s
 * \param Meas measurement vector
 * \param C_1 covariance matrix in k-1
 * \param Cinv_1 inverse of covariance matrix in k-1
 * \param D_1 mahalanobis distane in k-1
 **/
bool CSlam::joint_compatibility_incremental(std::vector<int> visible_, std::vector<float> Meas,
                                            CvMat *C_1, CvMat* Cinv_1, CvMat* H_1, CvMat* innov_1,
                                            CvMat **C, CvMat** Cinv, CvMat** H_, CvMat** innov)
{
    float *fstate;
    fstate = Xp->data.fl;

    double x,y,z,theta,phi,rho;
    double ox,oy,oz;

    CvPoint2D64f proj;
    CvMat* dpdr=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdt=cvCreateMat(2,3,CV_32FC1);
    CvMat* dpdf=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdc=cvCreateMat(2,2,CV_32FC1);
    CvMat* dpdk=cvCreateMat(2,4,CV_32FC1);
    CvMat* dpdw=cvCreateMat(2,6,CV_32FC1);

    CvMat *dpdW = cvCreateMat(2,6,CV_32FC1);
    cvZero(dpdW);

    //IC_Test var
//    CvMat* innov;
//    CvMat* C;
//    CvMat* Cinv;
//    CvMat* H_;
    CvMat* P_;
    CvMat* R_;
    CvMat* T2;
    CvMat* T3;
    CvMat* T4;

    //Iterative Calc var
    CvMat* w;
    CvMat* wt;
    CvMat* Cij;
    CvMat* Rij;
    CvMat* Hij;
    CvMat* N;
    CvMat* L;
    CvMat* LT;
    CvMat* K;

    CvMat* innov_ij;

    int fdims=6;
    int visNum_=0;
    for (unsigned int i = 0 ; i < visible_.size(); i++)
    {
        if ( visible_[i]>0 )
        {
            visNum_++;
        }
    }

    //WARNING se usa visible.size() porque se quiere que se use la P completa
    *innov=cvCreateMat(2*visNum_,1,CV_32FC1);
    *C=cvCreateMat(2*visNum_,2*visNum_,CV_32FC1);
    *Cinv=cvCreateMat(2*visNum_,2*visNum_,CV_32FC1);
    *H_=cvCreateMat(2*visNum_,modelSD+fdims*visible.size(),CV_32FC1);
    cvZero(*H_);
    P_=cvCreateMat(modelSD+fdims*visible.size(),12+fdims*visible.size(),CV_32FC1);
    R_=cvCreateMat(2*visNum_,2*visNum_,CV_32FC1);
    T2=cvCreateMat(2*visNum_,modelSD+fdims*visible.size(),CV_32FC1);
    T3=cvCreateMat(2*visNum_,1,CV_32FC1);
    T4=cvCreateMat(1,1,CV_32FC1);
    cvSetIdentity(R_,cvRealScalar(MEAS_NOISE_COV));

    if (C_1 == NULL)
    {
         unsigned int i = visible_.size()-1;
    if (visible_[i]>0)
    {
        float *fstate_=fstate;
        int s_i=modelSD+i*6;//

        fstate_+=s_i;
        x=*fstate_;        fstate_++;
        y=*fstate_;        fstate_++;
        z=*fstate_;        fstate_++;
        theta=*fstate_;    fstate_++;
        phi=*fstate_;      fstate_++;
        rho=*fstate_;      fstate_++;

        InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
        CvPoint3D64f p = cvPoint3D64f(ox,oy,oz);
        ProjectPoints3(&p, Xp,IntrinsicParam,DistortionParam,
                            &proj, dpdr,dpdt, dpdf,
                            dpdc, dpdk, dpdw);
        cvmSet(*innov,0,0,Meas[2*i]-proj.x);
        cvmSet(*innov,1,0,Meas[2*i+1]-proj.y);

        fillInvParamDeriv(dpdw, dpdW, theta,phi,rho);

        CopyMat(dpdt,*H_,0,0);
        CopyMat(dpdr,*H_,0,3);
        CopyMat(dpdW,*H_,0,modelSD+i*fdims);

    }//end if visible

        /* temp2 = H*P'(k) */
        cvMatMulAdd( *H_,P,0,T2 );
        /* C = temp2*Ht + R */
        cvGEMM( T2,*H_, 1,R_, 1, *C, CV_GEMM_B_T );

        // temp5 = z(k) - H*x'(k)

        // temp3=C^-1 * (z-h)
        cvInvert(*C,*Cinv);
    }
    else
    {
    //Cinv Sub matrixes
    K = cvCreateMatHeader(C_1->height, C_1->width, CV_32FC1);
    L = cvCreateMatHeader(2,C_1->height, CV_32FC1);
    LT= cvCreateMatHeader(C_1->height,2,CV_32FC1);
    N = cvCreateMatHeader(2,2,CV_32FC1);

    cvGetSubRect(*Cinv,K,cvRect(0,0,C_1->width,C_1->height));
    cvGetSubRect(*Cinv,L,cvRect(0,C_1->height,C_1->height,2));
    cvGetSubRect(*Cinv,LT,cvRect(C_1->width,0,2,C_1->height));
    cvGetSubRect(*Cinv,N,cvRect(C_1->width,C_1->height,2,2));

    //C Sub matrixes
    w = cvCreateMatHeader(2,C_1->width,CV_32FC1);
    wt= cvCreateMatHeader(C_1->height,2,CV_32FC1);
    Cij=cvCreateMatHeader(2,2,CV_32FC1);
    Rij=cvCreateMatHeader(2,2,CV_32FC1);

    cvGetSubRect(*C, w, cvRect(0,C_1->height,C_1->width, 2));
    cvGetSubRect(*C,wt, cvRect(C_1->width,0,2,C_1->height));
    cvGetSubRect(*C,Cij,cvRect(C_1->width,C_1->height,2,2));
    cvGetSubRect(R_,Rij,cvRect(C_1->width,C_1->height,2,2));

    //Hij matrix
    Hij = cvCreateMatHeader(2,SD,CV_32FC1);
    cvGetSubRect(*H_,Hij,cvRect(0,H_1->height,H_1->width,2));
    CopyMat(H_1,*H_,0,0);

    //innov matrix
    innov_ij = cvCreateMatHeader(2,1,CV_32FC1);
    CopyMat(innov_1,*innov,0,0);
    cvGetSubRect(*innov, innov_ij, cvRect(0,C_1->height,1,2));
    unsigned int i = visible_.size()-1;
    if (visible_[i]>0)
    {
        float *fstate_=fstate;
        int s_i=modelSD+i*6;//

        fstate_+=s_i;
        x=*fstate_;        fstate_++;
        y=*fstate_;        fstate_++;
        z=*fstate_;        fstate_++;
        theta=*fstate_;    fstate_++;
        phi=*fstate_;      fstate_++;
        rho=*fstate_;      fstate_++;

        InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
        CvPoint3D64f p = cvPoint3D64f(ox,oy,oz);
        ProjectPoints3(&p, Xp,IntrinsicParam,DistortionParam,
                            &proj, dpdr,dpdt, dpdf,
                            dpdc, dpdk, dpdw);
        cvmSet(innov_ij,0,0,Meas[2*i]-proj.x);
        cvmSet(innov_ij,1,0,Meas[2*i+1]-proj.y);

        fillInvParamDeriv(dpdw, dpdW, theta,phi,rho);

        CopyMat(dpdt,Hij,0,0);
        CopyMat(dpdr,Hij,0,3);
        CopyMat(dpdW,Hij,0,modelSD+i*fdims);

    }//end if visible
    //C Creation
    CopyMat(C_1,*C,0,0);
    //w=Hij*P*H_1'
    CvMat* temp= cvCreateMat(2,SD,CV_32FC1);
    cvMatMul(Hij,P, temp);
    cvGEMM(temp,H_1, 1, B, 0 ,w, CV_GEMM_B_T);
    cvReleaseMat(&temp);
    cvTranspose(w,wt);
    //Cij=Hij*p*Hij'
    QuadForm(P,Hij,Cij);
    cvAdd(Cij,Rij,Cij);

    //Cinv Creation
    //N=(Cij-w*Cinv_1*w')^-1
    CvMat* temp2= cvCreateMat(2,2,CV_32FC1);
    QuadForm(Cinv_1,w,temp2);
    cvSub(Cij,temp2,N);
    cvReleaseMat(&temp2);
    cvInv(N,N);

    //L=-N*W*Cinv_1
    CvMat* temp3= cvCreateMat(2,C_1->width,CV_32FC1);
    cvMatMul(w,Cinv_1,temp3);
    cvGEMM(N,temp3,-1,NULL,0,L);
    cvReleaseMat(&temp3);
    cvTranspose(L,LT);
    //K=Cinv_1+L N^-1 LT);
    CvMat* temp4=cvCreateMat(N->height, N->width, CV_32FC1);
    cvInv(N,temp4);
    QuadForm(temp4,LT,K,Cinv_1);
    }
    //cvSolve( C,innov,T3, CV_SVD );

    cvMatMul(*Cinv,*innov,T3);
    /* temp2 = (z-h) temp3t */
    cvGEMM(*innov,T3,1,NULL,0,  T4 ,CV_GEMM_A_T);

    bool result;
    double dist = 0;

    for (int i = 0 ; i < visNum_*2;i++){
        dist += cvmGet( *innov,i,0)*cvmGet( *innov,i,0);
    }
    if (cvmGet(T4,0,0)>125)//18DOF 6DOF=10.864940
    {
        result = false;
    }else{
        result = true;
    }

//    cvReleaseMatHeader(&w);
//    cvReleaseMatHeader(&wt);
//    cvReleaseMatHeader(&Cij);
//    cvReleaseMatHeader(&Rij);
//    cvReleaseMatHeader(&Hij);
//    cvReleaseMatHeader(&N);
//    cvReleaseMatHeader(&L);
//    cvReleaseMatHeader(&LT);
//    cvReleaseMatHeader(&K);
//    cvReleaseMatHeader(&innov_ij);

//    cvReleaseMat(&innov);
//    cvReleaseMat(&C);
//    cvReleaseMat(&Cinv);
//    cvReleaseMat(&H_);
    cvReleaseMat(&P_);
    cvReleaseMat(&R_);
    cvReleaseMat(&T2);
    cvReleaseMat(&T3);
    cvReleaseMat(&T4);

    cvReleaseMat(&dpdt);
    cvReleaseMat(&dpdr);
    cvReleaseMat(&dpdf);
    cvReleaseMat(&dpdc);
    cvReleaseMat(&dpdk);
    cvReleaseMat(&dpdw);
    cvReleaseMat(&dpdW);


      return result;
}
void CSlam::QuadForm(CvMat *A, CvMat *vect, CvMat* resultado,CvMat *B)
{
    CvMat* temp= cvCreateMat(vect->height, A->width,CV_32FC1);
    cvMatMul(vect,A, temp);
    cvGEMM(temp,vect, 1, B, 1 ,resultado, CV_GEMM_B_T);
    cvReleaseMat(&temp);
}
void CSlam::Cholesky(CvMat *in, CvMat *out)
{
    int i,j,k;
    float sum;
    int n = in->width;

    for ( i = 0; i < n; i++){
        for (j=i; j<n;j++){
            for (sum = cvmGet(in,i,j),k=0;k<i;k++)
                sum -= cvmGet(in,i,k)*cvmGet(in,j,k);
            if ( i == j ){
                if (sum<=0.0){
                    printf("Error cholesky\n");
                    sum=0.0000000001;
                    //exit(-1);
                }
                cvmSet(out,i,i,sqrt(sum));
            }else{
                cvmSet(out,j,i,sum/cvmGet(out,i,i));
            }
        }
    }
}
