#include "updater.h"
#include <iostream>

void on_mouse( int event, int x, int y, int flags, void* param )
{
    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        {

     CvPoint *p;
     p=(CvPoint*) param;
     p->x=x;
     p->y=y;
     }
     break;

     }
}
/**
* constructor
 * @param pMap puntero al mapa que se actualizará
 * @param pDataCam puntero a los parámetros de la cámara
 **/
CUpdater::CUpdater(CMap *pMap_ ,CDataCam *pDataCam_)
{
   ///CUIDADO ESTE CONSTRUCTOR NO SE USA
   pMap=pMap_;
   pDataCam=pDataCam_;
   border=20;
   num_feat_min=10;
   num_feat_max=30;
   point_sep=5;
   calidad_min_punto=2;
//   depth=0.02;
depth=0.2;
}

CUpdater::~CUpdater()
{

}
/**
 * constructor por defecto establece un borde de 30 y un número de características de 30
 **/

CUpdater::CUpdater()
{
   border=20;
   num_feat_min=8;
   num_feat_max=30;
   point_sep=50;//FIXME ESTO PONERLO A 50 OTRA VEZ!!!!!!
   calidad_min_punto=2;
   //   depth=0.02;
depth=0.2;
}

void CUpdater::randsample(vector<point> *data, int n,vector<point> *maybeinliners)
{
  int total= data->size();
  int r;
  if(n>total)
    {
      cout<<"error"<<endl;

    }
  for (int i =1; i<n ; i++)
    {
      r=1+(int) ((total-i)*rand()/(RAND_MAX+1.0));//rand()*(total-i);
      maybeinliners->push_back((*data)[r]);
      (*data)[r]=(*data)[total-i];
      data->pop_back();
    }
}
/**
 * \fn ransac
 * @param orig_data datos originales
 * @param k numero de iteraciones
 * @param n numero de datos que se cogen aleatoriamente
 * @param t umbral para aceptar el punto
 * @param d num minimo de puntos para aceptar el modelo
 *
 */

void CUpdater::ranasac(vector<point> orig_data,int k, int n,double t,int d)
{
 double thiserr;
 double besterr =999999999;
 vector<point> data;
 vector<point> maybeinliners;
 vector<point> bestinliners;
 int iterations = 0;
 cout <<"before while"<<endl;
 while (iterations<k)
   {
     int ss=orig_data.size();
     data.clear();
     ss=orig_data.size();
     // cout <<"befor for " <<endl;
     for (int i =0; i < orig_data.size();i++)
       {
	 data.push_back(orig_data[i]);
       }
     // cout<<"data size: "<<data.size()<<endl;
     //    data= orig_data;//FIXME
     randsample(&data, n, &maybeinliners);
     fit(&maybemodel, maybeinliners);
     for (int i =0 ; i < data.size(); i++)
       {
         //cout <<"ifor"<<i<<endl;
	 if(fits(data[i],maybemodel)<t)
	   {
	     maybeinliners.push_back(data[i]);
	   }
       }
     //     cout<<"maybeinliner.size "<<maybeinliners.size()<<endl;
     if (maybeinliners.size()>d)
       {
	 thiserr=fit(&bettermodel,maybeinliners);
	 //        cout<< "besterr: "<< besterr<< " this_err: "<< thiserr <<" inliners "<<maybeinliners.size()<<endl;

	 if (thiserr<besterr)
	   {
	     bestfit=bettermodel;
	     besterr=thiserr;
	     bestinliners=maybeinliners;
	     //          cout<< "besterr: "<< besterr<< " this_err: "<< thiserr <<" inliners "<<maybeinliners.size()<<endl;
	   }
       }
     maybeinliners.clear();
     iterations++;
     //cout<< "iter " <<iterations++<<endl;;

   }

 cout<<"poly_line:  ["<< bestfit.py[0]<<"(1,0,0) ;"<< bestfit.py[1]<<"(0,1,0) ; "<< bestfit.py[2]<<"(0,0,0); ";
 cout<< bestfit.py[3]<<"(2,0,0) ;"<< bestfit.py[4]<<"(0,2,0) ; "<< bestfit.py[5]<<"(1,1,0) "<<"]"<<endl;
 cout<<"poly_samp:  ["<< bestfit.px[0]<<"(1,0,0) ; "<< bestfit.px[1]<<"(0,1,0) ; "<< bestfit.px[2]<<"(0,0,0); ";
 cout<< bestfit.px[3]<<"(2,0,0) ;"<< bestfit.px[4]<<"(0,2,0) ; "<< bestfit.px[5]<<"(1,1,0) "<<"]"<<endl;

 cout<< "#besterr: "<< besterr<< " this_err: "<< thiserr <<" inliners "<<maybeinliners.size()<<endl;
 cout<<"#data size: "<<data.size()<<endl;
 //     cout<<"bestfit x: "<< bestfit.px[0]<<" "<< bestfit.px[1]<<" "<< bestfit.px[2]<<" "<<endl;
 //     cout<<"bestfit y: "<< bestfit.py[0]<<" "<< bestfit.py[1]<<" "<< bestfit.py[2]<<" "<<endl;
 if (besterr<10){
 int ok=0;
 for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
   {
     if((*It)->state==st_inited){
       ok=0;
       for (int i=0; i <bestinliners.size(); i++)
        {
            if (bestinliners[i].ID==(*It)->ID){
                ok=1;
            }
        }
       if (ok==0){
            (*It)->state=st_no_view;
            cout<<"eleminado ransac "<<(*It)->ID<<endl;
            pMap->visible--;
       }
     }
   }
 }
}
double CUpdater::fits(point p, param model)
{
  double err=0;
  err+=(p.img[0]-(model.px[0]*p.ground[0]+model.px[1]*p.ground[1]+model.px[2]
		  +model.px[3]*p.ground[0]*p.ground[0]
		  +model.px[4]*p.ground[1]*p.ground[1]
		  +model.px[5]*p.ground[0]*p.ground[1]
		  ))*
    (p.img[0]-(model.px[0]*p.ground[0]+model.px[1]*p.ground[1]+model.px[2]
	       +model.px[3]*p.ground[0]*p.ground[0]
	       +model.px[4]*p.ground[1]*p.ground[1]
	       +model.px[5]*p.ground[0]*p.ground[1]
	       ));
  err+=(p.img[1]-(model.py[0]*p.ground[0]+model.py[1]*p.ground[1]+model.py[2]
		  +model.py[3]*p.ground[0]*p.ground[0]
		  +model.py[4]*p.ground[1]*p.ground[1]
		  +model.py[5]*p.ground[0]*p.ground[1]
		  ))*
    (p.img[1]-(model.py[0]*p.ground[0]+model.py[1]*p.ground[1]+model.py[2]
	       +model.py[3]*p.ground[0]*p.ground[0]
	       +model.py[4]*p.ground[1]*p.ground[1]
	       +model.py[5]*p.ground[0]*p.ground[1]
	       ));
  return sqrt(err);
}
double CUpdater::fit(param *model, vector<point> points)
{

  CvMat *Bx,*By,*A,*Px,*Py;
  A=cvCreateMat(points.size(), 6,  CV_32FC1);
  Bx=cvCreateMat( points.size(),1, CV_32FC1);
  By=cvCreateMat( points.size(),1, CV_32FC1);
  Px=cvCreateMat( 6,1, CV_32FC1);
  Py=cvCreateMat( 6,1, CV_32FC1);

  for (int i =0;i<points.size();i++)
    {
      // cout<<"hh"<<endl;
      cvmSet(Bx,i,0,points[i].img[0]);
      cvmSet(By,i,0,points[i].img[1]);
      cvmSet(A,i,0,points[i].ground[0]);
      cvmSet(A,i,1,points[i].ground[1]);
      cvmSet(A,i,2,1.);
      cvmSet(A,i,3,points[i].ground[0]*points[i].ground[0]);
      cvmSet(A,i,4,points[i].ground[1]*points[i].ground[1]);
      cvmSet(A,i,5,points[i].ground[1]*points[i].ground[0]);

    }
  cvSolve(A,Bx,Px,CV_SVD);
  model->px[0]=cvmGet(Px,0,0);
  model->px[1]=cvmGet(Px,1,0);
  model->px[2]=cvmGet(Px,2,0);
  model->px[3]=cvmGet(Px,3,0);
  model->px[4]=cvmGet(Px,4,0);
  model->px[5]=cvmGet(Px,5,0);
  cvSolve(A,By,Py,CV_SVD);
  model->py[0]=cvmGet(Py,0,0);
  model->py[1]=cvmGet(Py,1,0);
  model->py[2]=cvmGet(Py,2,0);
  model->py[3]=cvmGet(Py,3,0);
  model->py[4]=cvmGet(Py,4,0);
  model->py[5]=cvmGet(Py,5,0);
  double err=0;
  for (int i =0;i<points.size();i++)
    {
      /* err+=(points[i].img[0]-(model->px[0]*points[i].ground[0]+model->px[1]*points[i].ground[1]+model->px[2]))*
	 (points[i].img[0]-(model->px[0]*points[i].ground[0]+model->px[1]*points[i].ground[1]+model->px[2]));
	 err+=(points[i].img[1]-(model->py[0]*points[i].ground[0]+model->py[1]*points[i].ground[1]+model->py[2]))*
	 (points[i].img[1]-(model->py[0]*points[i].ground[0]+model->py[1]*points[i].ground[1]+model->py[2]));*/
      err+=(points[i].img[0]-(model->px[0]*points[i].ground[0]+model->px[1]*points[i].ground[1]+model->px[2]
			      +model->px[3]*points[i].ground[0]*points[i].ground[0]
			      +model->px[4]*points[i].ground[1]*points[i].ground[1]
			      +model->px[5]*points[i].ground[0]*points[i].ground[1]
			      ))*
	(points[i].img[0]-(model->px[0]*points[i].ground[0]+model->px[1]*points[i].ground[1]+model->px[2]
			   +model->px[3]*points[i].ground[0]*points[i].ground[0]
			   +model->px[4]*points[i].ground[1]*points[i].ground[1]
                        +model->px[5]*points[i].ground[0]*points[i].ground[1]
			   ));
        err+=(points[i].img[1]-(model->py[0]*points[i].ground[0]+model->py[1]*points[i].ground[1]+model->py[2]
				+model->py[3]*points[i].ground[0]*points[i].ground[0]
				+model->py[4]*points[i].ground[1]*points[i].ground[1]
				+model->py[5]*points[i].ground[0]*points[i].ground[1]
				))*
	  (points[i].img[1]-(model->py[0]*points[i].ground[0]+model->py[1]*points[i].ground[1]+model->py[2]
			     +model->py[3]*points[i].ground[0]*points[i].ground[0]
			     +model->py[4]*points[i].ground[1]*points[i].ground[1]
			     +model->py[5]*points[i].ground[0]*points[i].ground[1]
			     ));
       	cvmSet(Bx,i,0,points[i].img[0]);
	cvmSet(By,i,0,points[i].img[1]);
	cvmSet(A,i,0,points[i].ground[0]);
	cvmSet(A,i,1,points[i].ground[1]);
	cvmSet(A,i,2,1.);
	cvmSet(A,i,3,points[i].ground[0]*points[i].ground[0]);
	cvmSet(A,i,4,points[i].ground[1]*points[i].ground[1]);
	cvmSet(A,i,5,points[i].ground[1]*points[i].ground[0]);

    }
  err/=points.size();
  err=sqrt(err);
  cvReleaseMat(&A);
  cvReleaseMat(&Bx);
  cvReleaseMat(&By);
  cvReleaseMat(&Px);
  cvReleaseMat(&Py);

  return err;
}

void CUpdater::TestRANSAC()
{
  cout<<"in ransac"<<endl;
  point p;
  vector<point> orig_data;
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      if((*It)->state==st_inited){
	p.ground[0]=(*It)->pto.x;
	p.ground[1]=(*It)->pto.y;
	p.img[0]=(*It)->old_pto.x;
	p.img[1]=(*It)->old_pto.y;
	p.ID=(*It)->ID;
	orig_data.push_back(p);
      }//end if state
    }//end iterator
  if (orig_data.size()<10)
    {
      printf("ERROR no hay suficientes puntos\n");
    }else
    {
      cout <<"pre ranasac"<<endl;
      ranasac(orig_data,200,10,35.0,(int)(orig_data.size()/8));
    cout <<"post ranasca"<<endl;
    }
}
void CUpdater::setMap(CMap *p)
{
  pMap=p;
}
void CUpdater::setDataCam(CDataCam *p)
{
  pDataCam=p;
}
void CUpdater::setModelCam(CModelCam *p)
{
  pModelCam=p;
}

void CUpdater::setTracker(CTracker *p)
{
  pTracker=p;
}


/**
 * \fn update
 * actualiza el estado de un punto <br>
 * si el punto esta vacío se pone como st_1st_point si tiene pix distinto de 0 <br>
 * si el punto está como st_1st_point se toma una primera rotación y una primera traslación para luego triangular <br>
 * si el putno está a una distancia suficiente (>dist_triang) entonces se triangula
 **/

int CUpdater::update()
{
  CvMat *h;
  h=cvCreateMat(3,1,CV_32FC1);
  for   (list<CElempunto*>::iterator It=pMap->bbdd.begin();It != pMap->bbdd.end();It++)
    {
      switch((*It)->state)
	{
	case st_empty:
	  if((*It)->pto.x!=0 &&(*It)->pto.y!=0)
	    {
	      (*It)->state=st_inited;
	      pModelCam->cvInverseParam(&h,(*It)->pto);
	      (*It)->wx=cvmGet(pDataCam->translation,0,0);
	      (*It)->wy=cvmGet(pDataCam->translation,1,0);
	      (*It)->wz=cvmGet(pDataCam->translation,2,0);
	      (*It)->theta=atan2(-cvmGet(h,1,0),sqrt(cvmGet(h,0,0)*cvmGet(h,0,0)+cvmGet(h,2,0)*cvmGet(h,2,0)));
	      (*It)->phi=atan2(cvmGet(h,0,0),cvmGet(h,2,0));
	      (*It)->rho=1./depth;
	      cout<<"update_idpth "<<" "<<(*It)->wx<<" "<<(*It)->wy<<" "<<(*It)->wz<<" "<<(*It)->theta<<" "<<(*It)->phi<<" "<<(*It)->rho<<" "<<cvmGet(h,0,0)<<" "<<cvmGet(h,1,0)<<" "<<cvmGet(h,2,0)<<" "<<endl;
	      //cvmCopy(pDataCam->rotation,(*It)->rot_unInit_1);
	      //cvmCopy(pDataCam->translation,(*It)->trans_unInit_1);
	      //(*It)->xpix_1=(*It)->pto.x;
	      //(*It)->ypix_1=(*It)->pto.y;
	    }
	  break;
	case st_inited:
	  (*It)->count++;
	default:
	  break;
	}

    }

  cvReleaseMat(&h);
  //triangulate();

  return 0;
}

/**
 * llama a pTracker para obtener una lista de puntos que se pueden usar<br>
 * los puntos se anaden si cumplen una serie de condiciones
 * @param f frame sobre el que buscar los puntos
 * @param faltan numero de puntos que se necesitan anadir
 * @return el numero de puntos anadidos
 **/

int CUpdater::busca_posibles_para_anadir(IplImage *f,IplImage *f2,int faltan)
{
  //  CvPoint pts[20];
  int count;
  int num_max;
  num_max=20;

  // count=busca_esquinas(f, pts,num_max);
  int *keys=0;
  int n_key;
  CvMat *points;
  int *keys2=0;
  int n_key2;
  CvMat *points2;

  n_key=pTracker->Init(f, &keys, &points);
  n_key2=pTracker->Init(f2, &keys2, &points2);

   int dim = pTracker->getFeatDim();
   count=n_key;//points->rows;

   bool encontrado;
   int j=0;
   int insert=0;
   int surf_encontrado=0;
   unsigned char key[dim];
   for (int i=0; i<1000000; i++)
     {
       if(j<count){
         do {
            encontrado=true;
            /////////////////////////////////////////////////

             float suma = 0;
             float suma_min = 10000000;
             float suma_sec = 10000000;
             float dif;
             unsigned int nearest_neighbour = 0;
                 //para cada muestra
             for (unsigned int ii = 0; ii<n_key2; ii++)
               {
             suma = 0;
             //para cada componente
             for (unsigned int jj = 0; jj<dim;jj++)
               {
                 //distancia en norma2
                 //	    dif=example_pairs[i*64+j]-query[j];
                 //dif=cvmGet(keys2,ii,jj)-cvmGet(keys,j,jj); //FIXME ESTA PARTE ES MUY LENTA
                 dif=keys2[ii*dim+jj]-keys[j*dim+jj];
                 suma += dif*dif;

               }
             suma=sqrt(suma);
             if (suma < suma_min)
               {
                 suma_sec = suma_min;
                 suma_min = suma;
                 nearest_neighbour = ii;
               }else if (suma < suma_sec)
               {
                 suma_sec = suma;
               }
               }
            // cout<<" suma_min: "<<suma_min<<" nearest_neighbour "<<nearest_neighbour<<" suma_sec "<<suma_sec<<" ratio "<<suma_min/suma_sec<<endl;

             //query->data[0] = example_pairs[nearest_neighbour].data[0];
             if ((suma_min <100000)&&((suma_min/suma_sec)<0.8)){

             }else {
               encontrado=false;
               //return -1;
             }
             if (n_key2==0) encontrado = true; /// En el caso de tracker file FIXME
	     if(encontrado==true) surf_encontrado++;
        /////////////////////////////////////////////////
        /////////////////////////////////////////////////

	       //no cerca de un punto existente
	       for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
		     It != pMap->bbdd.end(); It++ )
		 {
		   if((*It)->pto.x<(cvmGet(points,j,0)+point_sep) &&
		      (*It)->pto.y<(cvmGet(points,j,1)+point_sep) &&
		      (*It)->pto.x>(cvmGet(points,j,0)-point_sep) &&
		      (*It)->pto.y>(cvmGet(points,j,1)-point_sep))
		     {
		       encontrado=false;
		     }
		   if((*It)->state>=st_inited){      //FIXME no solo los inited ¿esto evita la revisita?
		     if((*It)->projx<(cvmGet(points,j,0)+point_sep) &&
			(*It)->projy<(cvmGet(points,j,1)+point_sep) &&
			(*It)->projx>(cvmGet(points,j,0)-point_sep) &&
			(*It)->projy>(cvmGet(points,j,1)-point_sep))
		       {
			 encontrado=false;
		       }
		   }
		 }


	       if( (cvmGet(points,j,0)<border) ||
		      (cvmGet(points,j,1)<border) ||
		      (cvmGet(points,j,0)>(pDataCam->frame_width-border)) ||
		      (cvmGet(points,j,1)>(pDataCam->frame_height-border)))
                  {
                      encontrado = false;
                  }

	       //calidad del punto mayor que un tamano FIXME ESTA CALIDAD DEPENDE DEL METODO
	       if(cvmGet(points,j,2)<calidad_min_punto)
		 {
		   encontrado = false;
		 }
	       //inserto si cumple todas las condiciones anteriores
	       if (encontrado==true)
		 {
	//	   cout << "inserto nuevo: "<<cvmGet(points,j,0)<<" "<<cvmGet(points,j,1)<<endl;
		   //pts2[insert]=pts[j];
		   for (int d =0; d<dim;d++)
		     {
		       key[d]=(unsigned char) keys[j*dim+d];//cvmGet(keys,j,d);
		       //cout<<"asignando key_char: "<<(int)key[d]<<" key_mat "<<cvmGet(keys,j,d)<<endl;
		     }
		   pMap->add_key(cvPoint((int)cvmGet(points,j,0),
					 (int)cvmGet(points,j,1)),key,dim);
		   insert++;
		 }
	       j++;
       }while(!encontrado && j<count);
      }//endif
    }//end for
cout<<"surf: "<< "encontrados: "<< surf_encontrado<< " aceptados: "<<insert<<endl;
     if (keys!=0)   delete (keys);
     if (keys2!=0)   delete (keys2);
   //cvReleaseMat(&keys);
   cvReleaseMat(&points);
   cout<<"insertados: "<<insert<<" de "<<j<<endl;
   return insert;
}
/**
 * se anaden puntos al mapa
 * @param f frame sobre el que buscar puntos
 * @param f2 frame anterior tiene que coincidir
 **/
int CUpdater::Add(IplImage *f,IplImage *f2)
{
  int cornercount;

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

  cornercount=num_feat_min-inited_vis;//bbdd.size();
  cout<< "faltan "<<cornercount<<"esquinas"<<num_feat_min<<" "<<pMap->visible<<endl;
  int insert=0;
  if (cornercount>0)
    {
      insert = busca_posibles_para_anadir(f,f2,num_feat_max-pMap->visible);
    }
  return 0;
}
/**
* añadir un punto manual
*
**/
int CUpdater::AddByHand(IplImage *f)
{

  int cornercount;
  cornercount=num_feat_min-pMap->visible;//bbdd.size();
  cout<< "faltan "<<cornercount<<"esquinas"<<endl;
  int insert=0;
  //if (cornercount>0)
    {
    CvPoint point;

      cvNamedWindow( "select" );
      cvSetMouseCallback("select",  on_mouse, (void*)&point);
      int *key;
      key=new int[pTracker->getFeatDim()];
      unsigned char *ckey;
      ckey=new unsigned char[pTracker->getFeatDim()];
      CvPoint oldpoint;
      oldpoint.x = point.x;
      oldpoint.y= point.y;
      while(cvWaitKey(100)!='q'){
      cvShowImage("select",f);
      CvPoint3D32f point2;
      if (point.x!=oldpoint.x || point.y!=oldpoint.y)
      {   float oldist=1000000;
          float dist;
          CvPoint p;
          for( int i = 0; i < pTracker->feat->total; i++ )
          {
             point2 = *(CvPoint3D32f*)cvGetSeqElem( pTracker->feat, i );
             float dx=0, dy=0;
             dx=point2.x-point.x;
             dy=point2.y-point.y;
             dist=dx*dx+dy*dy;
             if (dist< oldist){
                 oldist=dist;
                 p.x=(int)point2.x;
                 p.y=(int)point2.y;
             }
          }
            pTracker->Descriptor(f,&p,(int)point2.z, key);
            for (int kk =0 ; kk<pTracker->getFeatDim();kk++)
                ckey[kk]=key[kk];

 		    pMap->add_key(p,ckey,pTracker->getFeatDim());
		    insert++;
		    oldpoint.x = point.x;
		    oldpoint.y =point.y;
      }
      //insert = busca_posibles_para_anadir(f,f2,num_feat_max-pMap->visible);
      }
}
  return 0;

}
/**
 * funcion obsoleta
 **/
int CUpdater::busca_esquinas(IplImage *f,CvPoint* pts,int count)
{
  int cornercount;
  cornercount=count;
  //cvCvtColor(f,grey,CV_RGB2GRAY);
  CvPoint2D32f corners[100];
  IplImage *grey, *eig_image, *temp;
  grey=cvCreateImage( cvGetSize(f), 8, 1 );
  eig_image = cvCreateImage( cvGetSize(f), 32, 1 );
  temp=cvCreateImage(cvGetSize(f),32,1);
  cvCvtColor(f,grey,CV_RGB2GRAY);

  cvGoodFeaturesToTrack(grey,eig_image,temp,corners,&cornercount,0.1, 40, NULL);
  for (int i=0;i<cornercount;i++){
    pts[i]=cvPointFrom32f( corners[i]);
  }
  cvReleaseImage(&grey);
  cvReleaseImage(&eig_image);
  cvReleaseImage(&temp);
  return cornercount;

}
/**
 * establece condiciones en la que los puntos se deben borrar
 * FIXME es un poco redundante con la funcion match de la clase tracker y debe ser modificada.
 **/
int CUpdater::remove()
{
  int i=0;
  std::cout << "remove ";
  for ( list<CElempunto*>::iterator It=pMap->bbdd.begin();
	It != pMap->bbdd.end(); It++ )
    {
      if ((*It)->pto.x<0 ||(*It)->pto.y<0 ||
	  isnan((*It)->wx)||isnan((*It)->wy)||isnan((*It)->wz))
	{
	  cout<< "ERROR borro punto por not a number. estado:"<<(*It)->state <<"x y:"<<(*It)->pto.x<<" "<<(*It)->pto.y<< endl;
	  //pMap->bbdd.erase(It);
	  i++;
	  pMap->visible--;
	  exit(-1);

	}else if((*It)->pto.x<border ||(*It)->pto.y<border ||
		(*It)->pto.x>pDataCam->frame_width-border ||(*It)->pto.y>pDataCam->frame_height-border )
	      {
		if((*It)->state==st_inited)
		  {
		    i++;
		    std::cout << "pongo punto como st_no_view: "<<(*It)->ID<<" "<<(*It)->pto.x<<" "<<(*It)->pto.y<<std::endl;
		    (*It)->state = st_no_view;
		    pMap->visible--;
		  }else if((*It)->state!=st_no_view)
		  {
		    cout<< "borro punto por no iniciado y fuera devista state: "<<(*It)->state << endl;
		    exit(-1);	///FIXME En el casod de que el punto se deje de ver antes de inicializar quiero borrarlo.
		    //pMap->bbdd.erase(It);
		    //pMap->visible--;
		  }
               }

    }
  return i;
}
/**
 * TODO hace falta implementar esto
 **/
int CUpdater::rematch()
{

  return 0;
}
