#include "modelcam.h"
namespace SLAM{
using namespace std;

CModelCam::CModelCam()
{

}

CModelCam::~CModelCam()
{

}
/**
* proyecta los puntos que existen en pMap <br>
* rellena los puntos y las derivadas <br>
 * En esta funci&oacute;n se aplican las derivadas respecto del las
 variables de parametrización inversa.
 *
**/
void CModelCam::ProjectPoints()
{
CvMat* obj;
CvMat* proj;
obj=cvCreateMat(1,6,CV_32FC1);
proj=cvCreateMat(4,2,CV_32FC1);
CvMat * m;
m=cvCreateMat(3,1,CV_32FC1);
for (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
{
    cvmSet(obj,0,0,(*It)->wx);
    cvmSet(obj,0,1,(*It)->wy);
    cvmSet(obj,0,2,(*It)->wz);
    cvmSet(obj,0,3,(*It)->theta);
    cvmSet(obj,0,4,(*It)->phi);
    cvmSet(obj,0,5,(*It)->rho);

   ///FIXME solo se deber&iacute;a proyectar sobre los puntos iniciados
   cvProject_1_pto(obj,proj,(*It)->dpdr,(*It)->dpdt,(*It)->dpdw);
   (*It)->projx=cvmGet(proj,0,0);
   (*It)->projy=cvmGet(proj,0,1);
   float s_th=sin((*It)->theta);
   float c_th=cos((*It)->theta);
   float s_ph=sin((*It)->phi);
   float c_ph=cos((*It)->phi);
   /** \f$ \frac{dp}{d\theta}= \frac{dx}{dx}(\frac{cos(\theta)}{\rho})\f$ **/
   cvmSet((*It)->dpdw,0,3,cvmGet((*It)->dpdw,0,0)*(-s_th*s_ph)/(*It)->rho+
                          cvmGet((*It)->dpdw,0,1)*(-c_th     )/(*It)->rho+
                          cvmGet((*It)->dpdw,0,2)*(-s_th*c_ph)/(*It)->rho);
   /** \f$ \frac{dp}{d\phi}= \frac{dx}{dX}  (\frac{cos(\phi)}{\rho}) +
   \frac{dx}{dZ} (\frac{-sin(\phi)}{\rho})\f$ **/
   cvmSet((*It)->dpdw,0,4,cvmGet((*It)->dpdw,0,0)*(c_th*c_ph)/(*It)->rho+
                          cvmGet((*It)->dpdw,0,1)*(0.0    )/(*It)->rho+
                          cvmGet((*It)->dpdw,0,2)*(-c_th*s_ph)/(*It)->rho);

   /** \f$ \frac{dp}{d\rho}=\frac{dx}{dX} (\frac{-sin(\phi)}{\rho^2}) +
    * \frac{dx}{dY} (\frac{sin(\theta)}{\rho^2}) +
    * \frac{dx}{dZ} (\frac{-cos(\phi)}{\rho^2}) \f$ **/
   cvmSet((*It)->dpdw,0,5,cvmGet((*It)->dpdw,0,0)*(-c_th*s_ph)/((*It)->rho*(*It)->rho)+
                          cvmGet((*It)->dpdw,0,1)*(s_th     )/((*It)->rho*(*It)->rho)+
                          cvmGet((*It)->dpdw,0,2)*(-c_th*c_ph)/((*It)->rho*(*It)->rho));

   /** \f$ \frac{dy}{d\theta}= \frac{dy}{dX}(\frac{cos(\theta)}{\rho})\f$ **/
    cvmSet((*It)->dpdw,1,3,cvmGet((*It)->dpdw,1,0)*(-s_th*s_ph)/(*It)->rho+
                           cvmGet((*It)->dpdw,1,1)*(-c_th     )/(*It)->rho+
                           cvmGet((*It)->dpdw,1,2)*(-s_th*c_ph)/(*It)->rho);

   /** \f$ \frac{dy}{d\phi}=\frac{dy}{dX}(\frac{cos(\phi)}{\rho}) +
   \frac{dy}{dZ}(\frac{-sin(\phi)}{\rho}) \f$ **/
     cvmSet((*It)->dpdw,1,4,cvmGet((*It)->dpdw,1,0)*(c_th*c_ph)/(*It)->rho+
                            cvmGet((*It)->dpdw,1,1)*(0.0    )/(*It)->rho+
                            cvmGet((*It)->dpdw,1,2)*(-c_th*s_ph)/(*It)->rho);

   /** \f$ \frac{dp}{d\rho}=\frac{dy}{dX} (\frac{-sin(\phi)}{\rho^2}) +
    * \frac{dy}{dX} (\frac{sin(\theta)}{\rho^2}) +
    * \frac{dy}{dZ} (\frac{-cos(\phi)}{\rho^2}) \f$ **/
     cvmSet((*It)->dpdw,1,5,cvmGet((*It)->dpdw,1,0)*(-c_th*s_ph)/((*It)->rho*(*It)->rho)+
                            cvmGet((*It)->dpdw,1,1)*(s_th     )/((*It)->rho*(*It)->rho)+
                            cvmGet((*It)->dpdw,1,2)*(-c_th*c_ph)/((*It)->rho*(*It)->rho));

}
cvReleaseMat(&obj);
cvReleaseMat(&proj);
}

/**
 * funcion para proyectar un punto y conseguir su Jacobiana
 * \param nobj puntos 3D
 * \param nimg punto proyectado de salida
 * \param dpdr \f$\frac{dp}{drot}\f$
 * \param dpdt \f$\frac{dp}{dtrans}\f$
 * \param dpdw \f$\frac{dp}{dpts_3d}\f$
 **/
void CModelCam::cvProject_1_pto(CvMat* nobj, CvMat* nimg,
                        CvMat* dpdr, CvMat* dpdt, CvMat* dpdw)
{
   CvMat* dpdr_;
   CvMat* dpdt_;
   CvMat* dpdf;
   CvMat* dpdc;
   CvMat* dpdk;
   CvMat* dpdw_;
//Esto se hace así porque sino no puede detectar si la matriz es vertical u hor.
   dpdf= cvCreateMat(2*4,2,CV_32FC1);
   dpdc= cvCreateMat(2*4,2,CV_32FC1);
   dpdk= cvCreateMat(2*4,4,CV_32FC1);
   dpdr_= cvCreateMat(2*4,3,CV_32FC1);
   dpdt_= cvCreateMat(2*4,3,CV_32FC1);
   dpdw_= cvCreateMat(2*4,3*4,CV_32FC1);

   CvMat * m;
   m=cvCreateMat(3,1,CV_32FC1);
   CvMat *obj;
   obj=cvCreateMat(4,3,CV_32FC1);
   cvmSet(m,0,0,cos(cvmGet(nobj,0,3))*sin(cvmGet(nobj,0,4)));//(*It)->theta)*sin((*It)->phi));
   cvmSet(m,1,0,-sin(cvmGet(nobj,0,3)));//(*It)->theta));
   cvmSet(m,2,0,cos(cvmGet(nobj,0,3))*cos(cvmGet(nobj,0,4)));//cos((*It)->theta)*cos((*It)->phi));
   cvNormalize( m, m);
   cvmSet(obj,0,0,cvmGet(nobj,0,0) +cvmGet(m,0,0)/cvmGet(nobj,0,5));
   cvmSet(obj,0,1,cvmGet(nobj,0,1) +cvmGet(m,1,0)/cvmGet(nobj,0,5));
   cvmSet(obj,0,2,cvmGet(nobj,0,2) +cvmGet(m,2,0)/cvmGet(nobj,0,5));

   cvProjectPoints3(obj,pDataCam->rotation,pDataCam->translation,pDataCam->calibration,
			pDataCam->distortion, nimg,dpdr_, dpdt_, dpdf, dpdc, dpdk , dpdw_);
if (dpdr!=NULL){
   cvmSet(dpdr,0,0,cvmGet(dpdr_,0,0));
   cvmSet(dpdr,0,1,cvmGet(dpdr_,0,1));
   cvmSet(dpdr,0,2,cvmGet(dpdr_,0,2));
   cvmSet(dpdr,1,0,cvmGet(dpdr_,1,0));
   cvmSet(dpdr,1,1,cvmGet(dpdr_,1,1));
   cvmSet(dpdr,1,2,cvmGet(dpdr_,1,2));
}
if (dpdt!=NULL){
   cvmSet(dpdt,0,0,cvmGet(dpdt_,0,0));
   cvmSet(dpdt,0,1,cvmGet(dpdt_,0,1));
   cvmSet(dpdt,0,2,cvmGet(dpdt_,0,2));
   cvmSet(dpdt,1,0,cvmGet(dpdt_,1,0));
   cvmSet(dpdt,1,1,cvmGet(dpdt_,1,1));
   cvmSet(dpdt,1,2,cvmGet(dpdt_,1,2));
}
if (dpdw!=NULL){
   cvmSet(dpdw,0,0,cvmGet(dpdw_,0,0));
   cvmSet(dpdw,0,1,cvmGet(dpdw_,0,1));
   cvmSet(dpdw,0,2,cvmGet(dpdw_,0,2));
   cvmSet(dpdw,0,3,cvmGet(dpdw_,0,0));
   cvmSet(dpdw,0,4,cvmGet(dpdw_,0,1));
   cvmSet(dpdw,0,5,cvmGet(dpdw_,0,2));

   cvmSet(dpdw,1,0,cvmGet(dpdw_,1,0));
   cvmSet(dpdw,1,1,cvmGet(dpdw_,1,1));
   cvmSet(dpdw,1,2,cvmGet(dpdw_,1,2));
   cvmSet(dpdw,1,3,cvmGet(dpdw_,1,0));
   cvmSet(dpdw,1,4,cvmGet(dpdw_,1,1));
   cvmSet(dpdw,1,5,cvmGet(dpdw_,1,2));
}

cvReleaseMat (&dpdf);
cvReleaseMat (&dpdc);
cvReleaseMat (&dpdk);
cvReleaseMat (&dpdr_);
cvReleaseMat (&dpdt_);
cvReleaseMat (&dpdw_);
cvReleaseMat (&obj);
cvReleaseMat (&m);
}

void CModelCam::setDataCam(CDataCam *p)
{
	pDataCam=p;
}

void CModelCam::setMap(CMap *p)
{
	pMap=p;
}

void CModelCam::cvInverseParam(CvMat** h,CvPoint pto)
{
     CvMat *pixel;
     pixel=cvCreateMat(3,1,CV_32FC1);
     cvmSet(pixel,0,0,pto.x);
     cvmSet(pixel,1,0,pto.y);
     cvmSet(pixel,2,0,1);
     CvMat *inv;
     inv=cvCreateMat(3,3,CV_32FC1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);

     CvMat *temp1;
     temp1=cvCreateMat(3,1,CV_32FC1);
     cvGEMM(inv,pixel,1,NULL,0,temp1);//CUIDADO ESTA A 0 TRANSLATION

     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,*h);

	cvReleaseMat(&pixel);
	cvReleaseMat(&inv);
	cvReleaseMat(&temp1);
}
/**
*la funcion de proyeccion es la misma que opencv pero ampliada para devolver la derivada de los puntos respecto de los puntos
**/
void CModelCam::cvProjectPoints3( const CvMat* obj_points,
                        const CvMat* r_vec,
                        const CvMat* t_vec,
                        const CvMat* A,
                        const CvMat* dist_coeffs,
                        CvMat* img_points, CvMat* dpdr,
                        CvMat* dpdt, CvMat* dpdf,
                        CvMat* dpdc, CvMat* dpdk ,CvMat* dpdw)
{
   CvMat *_M = 0, *_m = 0;
   CvMat *_dpdr = 0, *_dpdt = 0, *_dpdc = 0, *_dpdf = 0, *_dpdk = 0,*_dpdw=0;

   CV_FUNCNAME( "cvProjectPoints3" );

   __BEGIN__;

   int i, j, count;
   int l=-1;
   int calc_derivatives;
   const CvPoint3D64f* M;
   CvPoint2D64f* m;
   double r[3], R[9], dRdr[27], t[3], a[9], k[4] = {0,0,0,0}, fx, fy, cx, cy;
   CvMat _r, _t, _a = cvMat( 3, 3, CV_64F, a ), _k;
   CvMat _R = cvMat( 3, 3, CV_64F, R ), _dRdr = cvMat( 3, 9, CV_64F, dRdr );
   double *dpdr_p = 0,*dpdw_p = 0, *dpdt_p = 0, *dpdk_p = 0, *dpdf_p = 0, *dpdc_p = 0;
   int dpdr_step = 0, dpdt_step = 0, dpdk_step = 0, dpdf_step = 0, dpdc_step = 0,dpdw_step=0;

   if( !CV_IS_MAT(obj_points) || !CV_IS_MAT(r_vec) ||
        !CV_IS_MAT(t_vec) || !CV_IS_MAT(A) ||
        /* CV_IS_MAT(dist_coeffs) ||*/ !CV_IS_MAT(img_points) )
      CV_ERROR( CV_StsBadArg, "One of required arguments is not a valid matrix" );

   count = MAX(obj_points->rows, obj_points->cols);
//   count = 1; //cuidado se ha cambiado para solo aceptar un puntos
   if( CV_IS_CONT_MAT(obj_points->type) && CV_MAT_DEPTH(obj_points->type) == CV_64F &&
       (obj_points->rows == 1 && CV_MAT_CN(obj_points->type) == 3 ||
       obj_points->rows == count && CV_MAT_CN(obj_points->type)*obj_points->cols == 3))
      _M = (CvMat*)obj_points;
   else
   {
      CV_CALL( _M = cvCreateMat( 1, count, CV_64FC3 ));
      CV_CALL( cvConvertPointsHomogenious( obj_points, _M ));
   }

   if( CV_IS_CONT_MAT(img_points->type) && CV_MAT_DEPTH(img_points->type) == CV_64F &&
       (img_points->rows == 1 && CV_MAT_CN(img_points->type) == 2 ||
       img_points->rows == count && CV_MAT_CN(img_points->type)*img_points->cols == 2))
      _m = img_points;
   else
      CV_CALL( _m = cvCreateMat( 1, count, CV_64FC2 ));

   M = (CvPoint3D64f*)_M->data.db;
   m = (CvPoint2D64f*)_m->data.db;

   if( CV_MAT_DEPTH(r_vec->type) != CV_64F && CV_MAT_DEPTH(r_vec->type) != CV_32F ||
       (r_vec->rows != 1 && r_vec->cols != 1 ||
       r_vec->rows*r_vec->cols*CV_MAT_CN(r_vec->type) != 3) &&
       (r_vec->rows != 3 && r_vec->cols != 3 || CV_MAT_CN(r_vec->type) != 1))
      CV_ERROR( CV_StsBadArg, "Rotation must be represented by 1x3 or 3x1 "
            "floating-point rotation vector, or 3x3 rotation matrix" );

   if( r_vec->rows == 3 && r_vec->cols == 3 )
   {
      _r = cvMat( 3, 1, CV_64FC1, r );
      CV_CALL( cvRodrigues2( r_vec, &_r ));
      CV_CALL( cvRodrigues2( &_r, &_R, &_dRdr ));
      cvCopy( r_vec, &_R );
   }
   else
   {
      _r = cvMat( r_vec->rows, r_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(r_vec->type)), r );
      CV_CALL( cvConvert( r_vec, &_r ));
      CV_CALL( cvRodrigues2( &_r, &_R, &_dRdr ) );
  }

   if( CV_MAT_DEPTH(t_vec->type) != CV_64F && CV_MAT_DEPTH(t_vec->type) != CV_32F ||
       t_vec->rows != 1 && t_vec->cols != 1 ||
       t_vec->rows*t_vec->cols*CV_MAT_CN(t_vec->type) != 3 )
      CV_ERROR( CV_StsBadArg,
                "Translation vector must be 1x3 or 3x1 floating-point vector" );

   _t = cvMat( t_vec->rows, t_vec->cols, CV_MAKETYPE(CV_64F,CV_MAT_CN(t_vec->type)), t );
   CV_CALL( cvConvert( t_vec, &_t ));

   if( CV_MAT_TYPE(A->type) != CV_64FC1 && CV_MAT_TYPE(A->type) != CV_32FC1 ||
       A->rows != 3 || A->cols != 3 )
      CV_ERROR( CV_StsBadArg, "Instrinsic parameters must be 3x3 floating-point matrix" );

   CV_CALL( cvConvert( A, &_a ));
   fx = a[0]; fy = a[4];
   cx = a[2]; cy = a[5];

   if( dist_coeffs )
   {
      if( !CV_IS_MAT(dist_coeffs) ||
           CV_MAT_DEPTH(dist_coeffs->type) != CV_64F &&
           CV_MAT_DEPTH(dist_coeffs->type) != CV_32F ||
           dist_coeffs->rows != 1 && dist_coeffs->cols != 1 ||
           dist_coeffs->rows*dist_coeffs->cols*CV_MAT_CN(dist_coeffs->type) != 4 )
         CV_ERROR( CV_StsBadArg,
                   "Distortion coefficients must be 1x4 or 4x1 floating-point vector" );

      _k = cvMat( dist_coeffs->rows, dist_coeffs->cols,
                  CV_MAKETYPE(CV_64F,CV_MAT_CN(dist_coeffs->type)), k );
      CV_CALL( cvConvert( dist_coeffs, &_k ));
   }

   if( dpdr )
   {
      if( !CV_IS_MAT(dpdr) ||
           CV_MAT_TYPE(dpdr->type) != CV_32FC1 &&
           CV_MAT_TYPE(dpdr->type) != CV_64FC1 ||
           dpdr->rows != count*2 || dpdr->cols != 3 )
	{
	 printf("dpdr rows: %d count*2:%d dpdr->cols: %d \n",dpdr->rows,count*2,dpdr->cols);
         CV_ERROR( CV_StsBadArg, "dp/drot must be 2Nx3 floating-point matrix" );
	}

      if( CV_MAT_TYPE(dpdr->type) == CV_64FC1 )
         _dpdr = dpdr;
      else
         CV_CALL( _dpdr = cvCreateMat( 2*count, 3, CV_64FC1 ));
      dpdr_p = _dpdr->data.db;
      dpdr_step = _dpdr->step/sizeof(dpdr_p[0]);
   }

   if( dpdw )
   {
      if( !CV_IS_MAT(dpdw) ||
           CV_MAT_TYPE(dpdw->type) != CV_32FC1 &&
           CV_MAT_TYPE(dpdw->type) != CV_64FC1 ||
           dpdw->rows != count*2 || dpdw->cols != 3*count )
         CV_ERROR( CV_StsBadArg, "dp/dw must be 2Nx3*N floating-point matrix" );

      if( CV_MAT_TYPE(dpdw->type) == CV_64FC1 )
         _dpdw = dpdw;
      else
         CV_CALL( _dpdw = cvCreateMat( 2*count, 3*count, CV_64FC1 ));
      cvSetIdentity(_dpdw,cvScalar(0.0));
      dpdw_p = _dpdw->data.db;
      dpdw_step = _dpdw->step/sizeof(dpdw_p[0]);
   }

   if( dpdt )
   {
      if( !CV_IS_MAT(dpdt) ||
           CV_MAT_TYPE(dpdt->type) != CV_32FC1 &&
           CV_MAT_TYPE(dpdt->type) != CV_64FC1 ||
           dpdt->rows != count*2 || dpdt->cols != 3 )
         CV_ERROR( CV_StsBadArg, "dp/dT must be 2Nx3 floating-point matrix" );

      if( CV_MAT_TYPE(dpdt->type) == CV_64FC1 )
         _dpdt = dpdt;
      else
         CV_CALL( _dpdt = cvCreateMat( 2*count, 3, CV_64FC1 ));
      dpdt_p = _dpdt->data.db;
      dpdt_step = _dpdt->step/sizeof(dpdt_p[0]);
   }

   if( dpdf )
   {
      if( !CV_IS_MAT(dpdf) ||
           CV_MAT_TYPE(dpdf->type) != CV_32FC1 && CV_MAT_TYPE(dpdf->type) != CV_64FC1 ||
           dpdf->rows != count*2 || dpdf->cols != 2 )
         CV_ERROR( CV_StsBadArg, "dp/df must be 2Nx2 floating-point matrix" );

      if( CV_MAT_TYPE(dpdf->type) == CV_64FC1 )
         _dpdf = dpdf;
      else
         CV_CALL( _dpdf = cvCreateMat( 2*count, 2, CV_64FC1 ));
      dpdf_p = _dpdf->data.db;
      dpdf_step = _dpdf->step/sizeof(dpdf_p[0]);
   }

   if( dpdc )
   {
      if( !CV_IS_MAT(dpdc) ||
           CV_MAT_TYPE(dpdc->type) != CV_32FC1 && CV_MAT_TYPE(dpdc->type) != CV_64FC1 ||
           dpdc->rows != count*2 || dpdc->cols != 2 )
         CV_ERROR( CV_StsBadArg, "dp/dc must be 2Nx2 floating-point matrix" );

      if( CV_MAT_TYPE(dpdc->type) == CV_64FC1 )
         _dpdc = dpdc;
      else
         CV_CALL( _dpdc = cvCreateMat( 2*count, 2, CV_64FC1 ));
      dpdc_p = _dpdc->data.db;
      dpdc_step = _dpdc->step/sizeof(dpdc_p[0]);
   }

   if( dpdk )
   {
      if( !CV_IS_MAT(dpdk) ||
           CV_MAT_TYPE(dpdk->type) != CV_32FC1 && CV_MAT_TYPE(dpdk->type) != CV_64FC1 ||
           dpdk->rows != count*2 || (dpdk->cols != 4 && dpdk->cols != 2) )
         CV_ERROR( CV_StsBadArg, "dp/df must be 2Nx4 or 2Nx2 floating-point matrix" );

      if( !dist_coeffs )
         CV_ERROR( CV_StsNullPtr, "dist_coeffs is NULL while dpdk is not" );

      if( CV_MAT_TYPE(dpdk->type) == CV_64FC1 )
         _dpdk = dpdk;
      else
         CV_CALL( _dpdk = cvCreateMat( dpdk->rows, dpdk->cols, CV_64FC1 ));
      dpdk_p = _dpdk->data.db;
      dpdk_step = _dpdk->step/sizeof(dpdk_p[0]);
   }

   calc_derivatives = dpdr || dpdt || dpdf || dpdc || dpdk;

   for( i = 0; i < count; i++ )
   {
      double X = M[i].x-t[0], Y = M[i].y-t[1], Z = M[i].z-t[2];
      double xp = R[0]*X + R[1]*Y + R[2]*Z ;
      double yp = R[3]*X + R[4]*Y + R[5]*Z ;
      double zp = R[6]*X + R[7]*Y + R[8]*Z ;
      double r2, r4, a1, a2, a3, cdist;
      double xd, yd;
      double x,y,z;
      z = zp ? 1./zp : 1;
      x =xp* z; y =yp* z;

      r2 = x*x + y*y;
      r4 = r2*r2;
      a1 = 2*x*y;
      a2 = r2 + 2*x*x;
      a3 = r2 + 2*y*y;
      cdist = 1 + k[0]*r2 + k[1]*r4;
      xd = x*cdist + k[2]*a1 + k[3]*a2;
      yd = y*cdist + k[2]*a3 + k[3]*a1;

      m[i].x = xd*fx + cx;
      m[i].y = yd*fy + cy;

      if( calc_derivatives )
      {
         if( dpdc_p )
         {
            dpdc_p[0] = 1; dpdc_p[1] = 0;
            dpdc_p[dpdc_step] = 0;
            dpdc_p[dpdc_step+1] = 1;
            dpdc_p += dpdc_step*2;
         }

         if( dpdf_p )
         {
            dpdf_p[0] = xd; dpdf_p[1] = 0;
            dpdf_p[dpdf_step] = 0;
            dpdf_p[dpdf_step+1] = yd;
            dpdf_p += dpdf_step*2;
         }

         if( dpdk_p )
         {
            dpdk_p[0] = fx*x*r2;
            dpdk_p[1] = fx*x*r4;
            dpdk_p[dpdk_step] = fy*y*r2;
            dpdk_p[dpdk_step+1] = fy*y*r4;
            if( _dpdk->cols > 2 )
            {
               dpdk_p[2] = fx*a1;
               dpdk_p[3] = fx*a2;
               dpdk_p[dpdk_step+2] = fy*a3;
               dpdk_p[dpdk_step+3] = fy*a1;
            }
            dpdk_p += dpdk_step*2;
         }

         if( dpdt_p )
         {
            double dxdp[] = { z, 0, -x*z };
            double dydp[] = { 0, z, -y*z };
            double dydt[3], dxdt[3];

            dxdt[0]=-R[0]*dxdp[0]-R[3]*dxdp[1]-R[6]*dxdp[2];
            dxdt[1]=-R[1]*dxdp[0]-R[4]*dxdp[1]-R[7]*dxdp[2];
            dxdt[2]=-R[2]*dxdp[0]-R[5]*dxdp[1]-R[8]*dxdp[2];

            dydt[0]=-R[0]*dydp[0]-R[3]*dydp[1]-R[6]*dydp[2];
            dydt[1]=-R[1]*dydp[0]-R[4]*dydp[1]-R[7]*dydp[2];
            dydt[2]=-R[2]*dydp[0]-R[5]*dydp[1]-R[8]*dydp[2];

            for( j = 0; j < 3; j++ )
            {
               double dr2dt = 2*x*dxdt[j] + 2*y*dydt[j];
               double dcdist_dt = k[0]*dr2dt + 2*k[1]*r2*dr2dt;
               double da1dt = 2*(x*dydt[j] + y*dxdt[j]);
               double dmxdt = fx*(dxdt[j]*cdist + x*dcdist_dt +
                        k[2]*da1dt + k[3]*(dr2dt + 2*x*dxdt[j]));
               double dmydt = fy*(dydt[j]*cdist + y*dcdist_dt +
                        k[2]*(dr2dt + 2*y*dydt[j]) + k[3]*da1dt);
               dpdt_p[j] = dmxdt;
               dpdt_p[dpdt_step+j] = dmydt;

            }

            dpdt_p += dpdt_step*2;
         }

         if( dpdw_p )
         {
            double dwdW[6];

            l+=1;
            double dxdp[] = { z, 0, -x*z };
            double dydp[] = { 0, z, -y*z };

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

            cvmSet(_dpdw,l*2,j,dmxdt);
            cvmSet(_dpdw,l*2+1,j,dmydt);
            }

         }
         if( dpdr_p )
         {
            double dx0dr[] =
            {
               X*dRdr[0] + Y*dRdr[1] + Z*dRdr[2],
               X*dRdr[9] + Y*dRdr[10] + Z*dRdr[11],
               X*dRdr[18] + Y*dRdr[19] + Z*dRdr[20]
            };
            double dy0dr[] =
            {
               X*dRdr[3] + Y*dRdr[4] + Z*dRdr[5],
               X*dRdr[12] + Y*dRdr[13] + Z*dRdr[14],
               X*dRdr[21] + Y*dRdr[22] + Z*dRdr[23]
            };
            double dz0dr[] =
            {
               X*dRdr[6] + Y*dRdr[7] + Z*dRdr[8],
               X*dRdr[15] + Y*dRdr[16] + Z*dRdr[17],
               X*dRdr[24] + Y*dRdr[25] + Z*dRdr[26]
            };
            for( j = 0; j < 3; j++ )
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
            dpdr_p += dpdr_step*2;
         }
      }
   }

   if( _m != img_points )
      cvConvertPointsHomogenious( _m, img_points );
   if( _dpdr != dpdr )
      cvConvert( _dpdr, dpdr );
   if( _dpdt != dpdt )
      cvConvert( _dpdt, dpdt );
   if( _dpdf != dpdf )
      cvConvert( _dpdf, dpdf );
   if( _dpdc != dpdc )
      cvConvert( _dpdc, dpdc );
   if( _dpdk != dpdk )
      cvConvert( _dpdk, dpdk );
   if( _dpdw != dpdw )
      cvConvert( _dpdw, dpdw );

   __END__;

   if( _M != obj_points )
      cvReleaseMat( &_M );
   if( _m != img_points )
      cvReleaseMat( &_m );
   if( _dpdr != dpdr )
      cvReleaseMat( &_dpdr );
   if( _dpdt != dpdt )
      cvReleaseMat( &_dpdt );
   if( _dpdf != dpdf )
      cvReleaseMat( &_dpdf );
   if( _dpdc != dpdc )
      cvReleaseMat( &_dpdc );
   if( _dpdk != dpdk )
      cvReleaseMat( &_dpdk );
   if( _dpdw != dpdw )
      cvReleaseMat( &_dpdw );
}
/** cálculo de la derivada de la inicialización de los parámetros respecto de
 * la posción y de los puntos <br>
 * se calcula por incrementos finitos
 **/
void CModelCam::getJInit(CvMat *Jpos, CvMat *Jpix, CvPoint pto)
{
     CvPoint2D32f fpto;
     fpto.x=pto.x;
     fpto.y=pto.y;
     CvMat *pixel;
     float diff=0.01;
     pixel=cvCreateMat(3,1,CV_32FC1);
     CvMat *h;
     h=cvCreateMat(3,1,CV_32FC1);
     double wx,wy,wz,theta,phi;
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     CvMat *inv;
     inv=cvCreateMat(3,3,CV_32FC1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);

     CvMat *temp1;
     temp1=cvCreateMat(3,1,CV_32FC1);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);

     cvInvert(pDataCam->rotMat,inv);

     cvGEMM(inv,temp1,1,NULL,0,h);

	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 wx=cvmGet(pDataCam->translation,0,0);
	 wy=cvmGet(pDataCam->translation,1,0);
	 wz=cvmGet(pDataCam->translation,2,0);
	 theta=atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0)));
     phi=atan2(cvmGet(h,0,0),cvmGet(h,2,0));
     //(*It)->rho=1./200000.;

     fpto.x+=diff;
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpix,0,0,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpix,1,0,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpix,2,0,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpix,3,0,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpix,4,0,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpix,5,0,0);//(*It)->rho=1./200000.;
     fpto.x-=diff;

     fpto.y+=diff;
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpix,0,1,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpix,1,1,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpix,2,1,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpix,3,1,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpix,4,1,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpix,5,1,0);//(*It)->rho=1./200000.;
     fpto.y-=diff;

     cvmSet(pDataCam->translation,0,0,cvmGet(pDataCam->translation,0,0)+diff);
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpos,0,0,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpos,1,0,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpos,2,0,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpos,3,0,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpos,4,0,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpos,5,0,0);//(*It)->rho=1./200000.;
     cvmSet(pDataCam->translation,0,0,cvmGet(pDataCam->translation,0,0)-diff);

     cvmSet(pDataCam->translation,1,0,cvmGet(pDataCam->translation,1,0)+diff);
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpos,0,1,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpos,1,1,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpos,2,1,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpos,3,1,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpos,4,1,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpos,5,1,0);//(*It)->rho=1./200000.;
     cvmSet(pDataCam->translation,1,0,cvmGet(pDataCam->translation,1,0)-diff);

     cvmSet(pDataCam->translation,2,0,cvmGet(pDataCam->translation,2,0)+diff);
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpos,0,2,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpos,1,2,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpos,2,2,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpos,3,2,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpos,4,2,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpos,5,2,0);//(*It)->rho=1./200000.;
     cvmSet(pDataCam->translation,2,0,cvmGet(pDataCam->translation,2,0)-diff);

     cvmSet(pDataCam->rotation,0,0,cvmGet(pDataCam->rotation,0,0)+diff);
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpos,0,3,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpos,1,3,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpos,2,3,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpos,3,3,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpos,4,3,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpos,5,3,0);//(*It)->rho=1./200000.;
     cvmSet(pDataCam->rotation,0,0,cvmGet(pDataCam->rotation,0,0)-diff);

     cvmSet(pDataCam->rotation,1,0,cvmGet(pDataCam->rotation,1,0)+diff);
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpos,0,4,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpos,1,4,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpos,2,4,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpos,3,4,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpos,4,4,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpos,5,4,0);//(*It)->rho=1./200000.;
     cvmSet(pDataCam->rotation,1,0,cvmGet(pDataCam->rotation,1,0)-diff);

     cvmSet(pDataCam->rotation,2,0,cvmGet(pDataCam->rotation,2,0)+diff);
     cvmSet(pixel,0,0,fpto.x);
     cvmSet(pixel,1,0,fpto.y);
     cvmSet(pixel,2,0,1);
     cvInvert(pDataCam->calibration,inv,CV_SVD);
     cvGEMM(inv,pixel,1,pDataCam->translation,-1,temp1);
     cvInvert(pDataCam->rotMat,inv);
     cvGEMM(inv,temp1,1,NULL,0,h);
	 // pModelCam->cvInverseParam(&h,(*It)->pto);
	 cvmSet(Jpos,0,5,(wx-cvmGet(pDataCam->translation,0,0))/diff);
	 cvmSet(Jpos,1,5,(wy-cvmGet(pDataCam->translation,1,0))/diff);
	 cvmSet(Jpos,2,5,(wz-cvmGet(pDataCam->translation,2,0))/diff);
	 cvmSet(Jpos,3,5,(theta-atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0))))/diff);
     cvmSet(Jpos,4,5,(phi-atan2(cvmGet(h,0,0),cvmGet(h,2,0)))/diff);
     cvmSet(Jpos,5,5,0);//(*It)->rho=1./200000.;
     cvmSet(pDataCam->rotation,2,0,cvmGet(pDataCam->rotation,2,0)-diff);

     cvReleaseMat(&pixel);
     cvReleaseMat(&inv);
	 cvReleaseMat(&temp1);

}
}