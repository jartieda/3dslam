/**
@author Jorge Artieda

**/

#include "surf.h"

using namespace std;
/**
 * constructor. Initializes thresholds
 **/
CSurf::CSurf():det_thres(8),cur_thres(0)
{

}
/**
* Destructor. Do nothing
**/
CSurf::~CSurf()
{

}
/** @bief non maximum suppression
 ** @param det array containing the information of the hessian determinant at several levels
 * @param max not used
 * @param feat sequence of featrues returned
 * @param levels number of piramid levels
 * The maximum is searched in a 3 x 3 x 3 neightbourhood
 **/
void CSurf::non_max_sup(double *det, IplImage **max, CvSeq* feat,int levels)
{
   CvPoint3D32f fp3;
   int suppressed;
   double pix;
   int s=1;
//   uchar* ptr ;
   double old_det;
   for (int level=0; level<levels;level++)
   {
	s=level+1;

        for (int i=3*(2*(levels+1)+1); i<img->width -(3*(2*(levels+1)+1)); i++)
        for (int j=3*(2*(levels+1)+1); j<img->height-(3*(2*(levels+1)+1));j++)
        {

            old_det=0;
            suppressed=255;
            pix=det[level*img->width*img->height+j*img->width+i];
           if (pix<det_thres){
		suppressed=0;
	   }else{
            for (int m=-1; m<2; m++)
                for (int n=-1;n<2;n++)
                    {
                        for (int l=-1; l<2;l++)
                        {
                          if((l+level<levels)&&(l+level>=0)){
                             if((l+level!=level)||(j+n!=j)||(i+m!=i)){
                               if (det[(l+level)*img->width*img->height+(j+n)*img->width+(i+m)]>=pix)
		                       {
                                 suppressed=0;
				                 break;
                               }
                               if(det[(l+level)*img->width*img->height+(j+n)*img->width+(i+m)]>old_det)
                               {
                                  old_det=det[(l+level)*img->width*img->height+(j+n)*img->width+(i+m)];
                               }
                             }
                          }
                        }
                    }
	    }
            if (suppressed==255){
            /** the maximum is accepted if det[] is greater than det_thres and
            * the ratio between the maximum and the second bigger is greater
            * than cur_thres
            **/
//		cout<<"max: "<<cvGetReal2D(det[level],j,i)<<endl;
		       if (det[level*img->width*img->height+j*img->width+i]>det_thres){
                  if(pix/old_det>cur_thres){
                    fp3 =cvPoint3D32f(i,j,level+1);
                    cvSeqPush( feat,(void*) &fp3);
                  }
		       }
            }
           // cvSetReal2D(max[level],j,i,suppressed);
        }
    //cvShowImage("win",max[level]);
    //cvWaitKey(100);
    }
}
/** @brief Calculates bloc filter using integral images
** @param x x coordinate of the upper left pixel of the block
* @param y y coordinate of the upper left pixel of the block
* @param h height of the block
* @param w width of the block
* @param I integral image
*
* This function calculates the sum of the values contained in the the rectangle
* defined by x,y,w and h. The algorithm is based on integral images that can be
* calculated using cvIntegral function.
* @return sum of all the pixels in the block
**/
double CSurf::block(int x, int y, int h, int w,IplImage *I)
{
        int pix_size = 4;//(I->depth & 255) >> 3;
        uchar* ptr = (uchar*)I->imageData;
        long int * ptr1,*ptr2,*ptr3,*ptr4;

        ptr1 = ( long int*)(ptr +  y*I->widthStep + x*pix_size);
        ptr2 = ( long int*)(ptr +  (y+h)*I->widthStep + (x+w)*pix_size);
        ptr3 = ( long int*)(ptr +  y*I->widthStep + (x+w)*pix_size);
        ptr4 = ( long int*)(ptr +  (y+h)*I->widthStep + x*pix_size);

    //return cvGet2D(I,y,x).val[0]+cvGet2D( I,y+h, x+w).val[0]- cvGet2D(I,y,x+w).val[0]- cvGet2D( I,y+h, x).val[0];
	return *ptr1+*ptr2-*ptr3-*ptr4;

}
/** @Estimates a repeteable orientation for a given feature
* @param x x coordinate of the feature
* @param y y cooredinate of the feature
* @param s scale at which the feature was found
*
* This function estimates an orientatio that is repeteable. This orientation is
* used to get orientation invariance of the features. The algorithm first
* estimate vertical and horizontal haar wavelet response using integral images.
* Then the orientation is calculed as the greater sum of orientations in a sliding
* window of angle pi/3
* @return orientation of the feature in radians
**/
float CSurf::orientation(int x, int y, int s)
{
       //IplImage* circle;
       //circle=cvCreateImage(cvSize(6*s,6*s),IPL_DEPTH_8U,1);
       //cvGetRectSubPix( img, circle, cvPoint2D32f(x,y) );
       int hor[36];
       int ver[36];
       float angle[36];
       float orient=0;
       float mod=0;
       float old_mod=0;
       // cout<<"point" <<x<<" "<<y<<" "<<s<<endl;
       int k=0;
       for (int i=-3*s; i<3*s; i+=s)
         for (int j=-3*s; j<3*s; j+=s)
         {
              hor[k]=(int)wl_horz(i+x,j+y,s);
              ver[k]=(int)wl_vert(i+x,j+y,s);
              angle[k]=atan2(ver[k],hor[k]);
              k++;
         }

       float pitercios = 3.14159/3.0;
       float dospi=3.14159*2.0;
       float sumx,sumy;
       for (float i=0; i<dospi;i+=0.1)
       {
           //cout<<"angle "<<i<<endl;
           sumx=0;
           sumy=0;
           for (int j=0; j<36;j++)
           {
               if (((angle[j]<i+pitercios)&&(angle[j]>i))||
               (((angle[j]+dospi)<i+pitercios)&&((angle[j]+dospi)>i)))
               {
                   sumx+=hor[j];
                   sumy+=ver[j];
               }
           }

           mod=sumx*sumx+sumy*sumy;
           if (mod>old_mod)
           {
              old_mod= mod;
              orient=i;
           }
       }

       return orient;
}

double CSurf::wl_vert(int x, int y,int s)
{
    //   cout<<"vert "<<x<<" "<<y<<" "<<s<<endl;
       double r;
        if ((x-2*s>0)&&(y-2*s>0)&&(x+2*s<integral->width)&&(y+2*s<integral->height))
       {
       r=1*block(x-2*s,y+2*s,2*s,4*s,integral);
       r=r-1*block(x-2*s,y,2*s,4*s,integral);
       }else r=0;
       return r;
}

double CSurf::wl_horz(int x, int y,int s)
{
      // cout<<"horz "<<x<<" "<<y<<" "<<s<<endl;
       double r;
       if ((x-2*s>0)&&(y-2*s>0)&&(x+2*s<integral->width)&&(y+2*s<integral->height))
       {
       r=1*block(x-2*s,y-2*s,4*s,2*s,integral);
       r=r-1*block(x,y-2*s,4*s,2*s,integral);
       }else r=0;
       return r;
}
/** @brief this function calculates the surf-64 descriptor for a given feature
* @param x x coordinate
* @param y y coordinate
* @param s scale of the feature
* @param dir orientation of the feature
* This function calcualtes the surf-64 descritpor for a feature at the given position
* scale and orientation. **/
int *CSurf::descriptor(int x, int y, int s,float dir)
{
int *desc;
desc = new int[64];
for (int i =0; i<64;i++) desc[i]=0;
IplImage *region;
region = cvCreateImage(cvSize(20*s,20*s),8,1);//ESTO ESTABA MULTIPLICADO POR S
IplImage *dx,*dy;
dx=cvCreateImage(cvSize(20*s,20*s),8,1);
dy=cvCreateImage(cvSize(20*s,20*s),8,1);
IplImage *gauss;
gauss=cvCreateImage(cvSize(20*s,20*s),8,1);
/*IplImage *cp;
cp=cvCreateImage(cvGetSize(img),8,1);
cvCopy(img,cp);*/
/** First a square of size 20*s x 20*s is extracted with the given orientation **/
gaussian(gauss);
float cd=cos(dir);
float sd=sin(dir);
CvPoint2D32f src[4];
CvPoint2D32f dst[4];
src[0].x=cd*10*s+sd*10*s+x;
src[0].y=-sd*10*s+cd*10*s+y;
src[1].x=cd*10*s+sd*-10*s+x;
src[1].y=-sd*10*s+cd*-10*s+y;
src[2].x=cd*-10*s+sd*-10*s+x;
src[2].y=-sd*-10*s+cd*-10*s+y;
src[3].x=cd*-10*s+sd*10*s+x;
src[3].y=-sd*-10*s+cd*10*s+y;

dst[0].x=10.0*s;
dst[0].y=-10.0*s;
dst[1].x=10.0*s;
dst[1].y=10.0*s;
dst[2].x=-10.0*s;
dst[2].y=10.0*s;
dst[3].x=-10.0*s;
dst[3].y=-10.0*s;

/*CvPoint *src_;
src_=new CvPoint[4];
CvPoint **kk;
kk=&src_;
CvPoint2D32f dst_[4];
src_[0].x=cd*10*s+sd*10*s+x;
src_[0].y=-sd*10*s+cd*10*s+y;
src_[1].x=cd*10*s+sd*-10*s+x;
src_[1].y=-sd*10*s+cd*-10*s+y;
src_[2].x=cd*-10*s+sd*-10*s+x;
src_[2].y=-sd*-10*s+cd*-10*s+y;
src_[3].x=cd*-10*s+sd*10*s+x;
src_[3].y=-sd*-10*s+cd*10*s+y;
for (int i =0 ;i<4;i++)
    cout<<"x "<<src[i].x<<" y "<<src[i].y<<endl;
cout<<endl;
int npts=4;
cvPolyLine( cp, kk, &npts, 1, 1,cvScalar(255));
*/

CvMat* wrap;
wrap=cvCreateMat(2,3,CV_32FC1);
cvGetAffineTransform(dst,src,wrap);

cvGetQuadrangleSubPix(img, region, wrap);
cvReleaseMat (&wrap);
/** Then dx and dy responses are calculated using haar wavelets of size 2*s **/
CvMat *kernel_hor=cvCreateMat(2*s,2*s,CV_32FC1);
CvMat *kernel_ver=cvCreateMat(2*s,2*s,CV_32FC1);
for (int i=0; i<2*s; i++){
	for (int j=0; j<s;j++)
	  {
		cvmSet(kernel_hor,i,j,-1);
		cvmSet(kernel_ver,j,i,-1);
	  }
	for (int j=s; j<2*s;j++)
      {
		cvmSet(kernel_hor,i,j,1);
		cvmSet(kernel_ver,j,i,1);
	  }
  }
cvFilter2D( region, dx, kernel_hor);
cvFilter2D( region, dy, kernel_ver);
cvReleaseMat(&kernel_hor);
cvReleaseMat(&kernel_ver);
cvMul(gauss,dx,dx);
cvMul(gauss,dy,dy);

float tx,ty;
/** Finally the descriptor is calculated as the sum of dx, dy, |dx| and |dy| responses in
* each of the 16 squares of a 4 by 4 grid sampled int 5*s grid. **/
for (int rx=0; rx<4; rx++)
    for (int ry=0; ry<4; ry++)
    {
	for (int sx=0; sx<5; sx++)
	    for (int sy=0; sy<5; sy++)
	    {
		int xp=rx*5*s+sx*s;
		int yp=ry*5*s+sy*s;
//		tx=horiz(xp,yp,s,region);
        tx=cvGetReal2D(dx,xp,yp);
//		ty=vert(xp,yp,s,region);
        ty=cvGetReal2D(dy,xp,yp);
		desc[(rx*4+ry)*4]+=(int)tx;
		desc[(rx*4+ry)*4+1]+=(int)ty;
		desc[(rx*4+ry)*4+2]+=(int)fabs(tx);
		desc[(rx*4+ry)*4+3]+=(int)fabs(ty);
	    }
    }
    double mod=0;
    for (int i=0; i<64; i++)
    {
        mod=mod+((float)desc[i]*(float)desc[i]);
    }
    mod=sqrt(mod);

    for (int i=0; i<64; i++)
    {
        desc[i]=(int)(desc[i]*255.0/mod);
    }
/*    cvShowImage("win",region);
    //cvWaitKey(0);
    cvShowImage("CamShiftDemo",cp);
    cvWaitKey(0);*/
    cvReleaseImage(&region);
    cvReleaseImage(&dx);
    cvReleaseImage(&dy);
    cvReleaseImage(&gauss);
    return desc;
}
float CSurf::horiz(int x, int y,int s, IplImage* region)
{
//     IplImage* kernel;
//     kernel=cvCreateImage(cvSize(2*s,2*s),8,1);
     int *kernel = new int [2*s*2*s];
     float sum=0;
     for (int i=0; i<2*s; i++){
	for (int j=0; j<s;j++)
	  {
		kernel[i*2*s+j]=-1;
	  }
	for (int j=s; j<2*s;j++)
          {
		kernel[i*2*s+j]=1;
	  }
     }
     for (int i=0; i<2*s; i++)
	for (int j=0; j<2*s;j++){
	{
 	    if ((x+i<region->width)&&(y+j<region->height))
     		sum+=cvGetReal2D(region, x+i,y+j)*kernel[i*2*s+j];
	}
     }
	delete(kernel);
//     cvReleaseImage(&kernel);
     return sum ;
}
float CSurf::vert(int x, int y, int s, IplImage* region)
{
     //IplImage* kernel;
     //kernel=cvCreateImage(cvSize(2*s,2*s),8,1);
     int *kernel=new int [2*s*2*s];
     float sum=0;
     for (int j=0; j<2*s; j++){
	for (int i=0; i<s;i++)
	  {
		kernel[i*2*s+j]=-1;
	  }
	for (int i=s; i<2*s;i++)
          {
		kernel[i*2*s+j]=1;
	  }
     }

     for (int i=0; i<2*s; i++){
	for (int j=0; j<2*s;j++)
	{
		if ((x+i<region->width)&&(y+j<region->height))
     			sum+=cvGetReal2D(region, x+i,y+j)*kernel[i*2*s+j];
	}
     }
//     cvReleaseImage(&kernel);
	delete (kernel);
     return sum ;
}
/** @brief simple nearest neightbour classifier
* @param query vector of 64 integer to classify
* @param example_pairs vector of 64*samples in which the query vectors are classified
* @param n_examples number fo samples in example_pairs vector
* This function is a simple classifier based on the euclidean distance. Best
* match is the one with less distance to the samples.
* @return the number of the category or -1 if it's not found.
 **/
int CSurf::nearest_neighbor_classify(int *query, int *example_pairs,int n_examples)
{

// PUT YOUR NEAREST NEIGHBOR CLASSIFIER HERE, ASSIGN CLASS LABEL TO query_point[0]
  float suma = 0;
  float suma_min = 10000000;
  float suma_sec = 10000000;
  float dif;
  unsigned int nearest_neighbour = 0;
//para cada muestra
  for (unsigned int i = 0; i<(unsigned int)n_examples; i++)
  {
    suma = 0;
	//para cada componente
    for (unsigned int j = 0; j<64;j++)
    {
//distancia en norma2
	    dif=example_pairs[i*64+j]-query[j];
        suma += dif*dif;
        //printf("ex_pairs %d query %d\n",example_pairs[i*64+j],query[j]);
    }
    suma=sqrt(suma);
    if (suma < suma_min)
    {
      suma_sec = suma_min;
      suma_min = suma;
      nearest_neighbour = i;
    }else if (suma < suma_sec)
    {
      suma_sec = suma;
    }
  }
    //cout<<" suma_min: "<<suma_min<<" nearest_neighbour "<<nearest_neighbour<<" suma_sec "<<suma_sec<<" ratio "<<suma_min/suma_sec<<endl;
    //for (int j=0; j<64 ; j++)
    //    cout<<example_pairs[nearest_neighbour*64+j]<<" : "<<query[j]<<endl;
    //query->data[0] = example_pairs[nearest_neighbour].data[0];
    /** If a result has a distance less than 15000 and a ratio between the minimum distance
    * and the second best greater than 0.9 the point is rejected as not found **/
	if ((suma_min <150000)&&((suma_min/suma_sec)<0.9)){
  		return nearest_neighbour;
	}else {
  		return -1;
	}

}

/** @brief simple nearest neightbour classifier
* @param query vector of 64 integer to classify
* @param example_pairs vector of 64*samples in which the query vectors are classified
* @param n_examples number fo samples in example_pairs vector
* This function is a simple classifier based on the euclidean distance. Best
* match is the one with less distance to the samples.
* restrics the distance with a previous point in previous image
* @return a list of n number of the categories or -1 if it's not found.
 **/
int CSurf::nearest_neighbor_2(int *query, int *example_pairs,int n_examples,CvPoint p,CvMat *newmat,float dist)
{

// PUT YOUR NEAREST NEIGHBOR CLASSIFIER HERE, ASSIGN CLASS LABEL TO query_point[0]
  float suma = 0;
  float suma_min = 10000000;
  float suma_sec = 10000000;
  float dif;
  unsigned int nearest_neighbour = 0;
//para cada muestra
  for (unsigned int i = 0; i<(unsigned int)n_examples; i++)
  {
    suma = 0;
    float dx= cvmGet(newmat,i,0)-p.x;
    float dy= cvmGet(newmat,i,1)-p.y;
    // cout<<"px "<<p.x<<" py "<<p.y<<" nx "<<cvmGet(newmat,i,0)<<" ny "<<cvmGet(newmat,i,1)<<" dx "<<dx<<" dy "<<dy<<endl;
    if (sqrt (dx*dx+dy*dy)<dist)
    {
        // cout<<"aceptado "<< sqrt (dx*dx+dy*dy)<< endl;
    	//para cada componente
        for (unsigned int j = 0; j<64;j++)
        {
    //distancia en norma2
    	    dif=example_pairs[i*64+j]-query[j];
            suma += dif*dif;
            //printf("ex_pairs %d query %d\n",example_pairs[i*64+j],query[j]);
        }
        suma=sqrt(suma);
        if (suma < suma_min)
        {
          suma_sec = suma_min;
          suma_min = suma;
          nearest_neighbour = i;
        }else if (suma < suma_sec)
        {
          suma_sec = suma;
        }
    }
  }
    cout<<" suma_min: "<<suma_min<<" nearest_neighbour "<<nearest_neighbour<<" suma_sec "<<suma_sec<<" ratio "<<suma_min/suma_sec<<endl;
   /* for (int j=0; j<64 ; j++)
        cout<<example_pairs[nearest_neighbour*64+j]<<" : "<<query[j]<<endl;*/
    //query->data[0] = example_pairs[nearest_neighbour].data[0];
    /** If a result has a distance less than 15000 and a ratio between the minimum distance
     * and the second best greater than 0.9 the point is rejected as not found
     **/
	if ((suma_min <150000)&&((suma_min/suma_sec)<0.9)){
  		return nearest_neighbour;
	}else {
  		return -1;
	}


}
/** @brief simple nearest neightbour classifier
* @param query vector of 64 integer to classify
* @param example_pairs vector of 64*samples in which the query vectors are classified
* @param n_examples number fo samples in example_pairs vector
* This function is a simple classifier based on the euclidean distance. Best
* match is the one with less distance to the samples.
* Obtains multiple best matches
* @return the number of the category or -1 if it's not found.
 **/
int CSurf::nearest_neighbor_classify3(int *query, int *example_pairs,int n_examples,vector<int> &result)
{
        // PUT YOUR NEAREST NEIGHBOR CLASSIFIER HERE, ASSIGN CLASS LABEL TO query_point[0]
    float suma = 0;
    float suma_min = 10000000;
    float suma_sec = 10000000;
    float dif;
    unsigned int listaResult[5]={0,0,0,0,0};
    float listaSuma[5]={10000000,10000000,10000000,10000000,10000000};
    unsigned int nearest_neighbour = 0;
    //para cada muestra
    for (unsigned int i = 0; i<(unsigned int)n_examples; i++)
    {
        suma = 0;
        //para cada componente
        for (unsigned int j = 0; j<64;j++)
        {
            //distancia en norma2
            dif=example_pairs[i*64+j]-query[j];
            suma += dif*dif;
        }
        suma=sqrt(suma);
        for (int j =0; j< 5; j++)
        {
           if (suma < listaSuma[j]){
                for ( int k = 4 ; k>j; k--)
                {
                     listaSuma[k]= listaSuma[k-1];
                     listaResult[k]= listaResult[k-1];
                }
                listaSuma[j]=suma;
                listaResult[j]=i;
                break;
           }
        }
        if (suma < suma_min)
        {
            suma_sec = suma_min;
            suma_min = suma;
            nearest_neighbour = i;
        }else if (suma < suma_sec)
        {
            suma_sec = suma;
        }
    }

    /** If a result has a distance less than 15000 and a ratio between the minimum distance
    * and the second best greater than 0.9 the point is rejected as not found **/
    result.push_back(listaResult[0]);
    for (int j=1;j<5;j++){
        if ((listaSuma[j]<150000)&&((listaSuma[0]/listaSuma[j])>0.5)){
            result.push_back(listaResult[j]);
        }
    }
	if ((suma_min <150000)&&((suma_min/suma_sec)<0.9)){
  		return nearest_neighbour;
	}else {
  		return -1;
	}
}

/** @brief extract surf features from an image
* @param img image
* @param feat sequence of returned features
* @param levels number of levels of the pyramid.
* This funcion extracts surf features from an image. Function is based on Integral
* images to improve the performance of the filters.
**/
void CSurf::find_features(IplImage* _img,CvSeq*feat,int levels)
{
    cout<<"find feat"<<endl;
    img = _img;
    float *dyy,*dxx,*dxy;
    IplImage *max[4];
    if(!integral) cvReleaseImage(&integral);
    integral=cvCreateImage(cvSize(img->width+1,img->height+1),IPL_DEPTH_32S,1);
    dxx=new float[(img->width+1)*(img->height+1)];//cvCreateImage(cvSize(img->width+1,img->height+1),IPL_DEPTH_64F,1);
    dyy=new float[(img->width+1)*(img->height+1)];//cvCreateImage(cvSize(img->width+1,img->height+1),IPL_DEPTH_64F,1);
    dxy=new float[(img->width+1)*(img->height+1)];//cvCreateImage(cvSize(img->width+1,img->height+1),IPL_DEPTH_64F,1);

   /* for(int level=0;level<levels;level++){
       max[level]=cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U,1);
	   cvZero(max[level]);
    }*/
    double *det = new double[levels*img->width*img->height];
    cvIntegral(img,integral);

    cvNamedWindow( "win");
    /*cvShowImage("win",img);
    cvWaitKey(10);
    */
    int s=1;
    double a;
    double b;
    double c;
    for (int level=0; level<levels;level++)
    {
        s=level+1;
/*    for (int i=0; i<img->width-(9*s+1); i++)
        for(int j=0; j<img->height-(9*s+1);j++)
        {
	    dyy[j*(img->width+1)+i]=(-1*block(i,j,3*s,9*s,integral)
                                     +2*block(i,j+3*s,3*s,9*s,integral)
                                     -1*block(i,j+6*s,3*s,9*s,integral))/(s*s);
            dxx[j*(img->width+1)+i]=(-1*block(i,j,9*s,3*s,integral)
                                     +2*block(i+3*s,j,9*s,3*s,integral)
                                     -1*block(i+6*s,j,9*s,3*s,integral))/(s*s);
            dxy[j*(img->width+1)+i]=( 1*block(i+1,j+1,3*s,3*s,integral)
                                     -1*block(i+1,j+3*s+2,3*s,3*s,integral)
                                     -1*block(i+3*s+2,j+1,3*s,3*s,integral)
                                     +1*block(i+3*s+2,j+3*s+2,3*s,3*s,integral))/(s*s);*/
    int size=0;
    size=2*s+1;
    for (int i=0; i<img->width-(3*size+1); i++)
        for(int j=0; j<img->height-(3*size+1);j++)
        {

            dyy[j*(img->width+1)+i]=(-1*block(i,j,size,3*size,integral)
                                     +2*block(i,j+size,size,3*size,integral)
                                     -1*block(i,j+2*size,size,3*size,integral))/(s*s);
            dxx[j*(img->width+1)+i]=(-1*block(i,j,3*size,size,integral)
                                     +2*block(i+size,j,3*size,size,integral)
                                     -1*block(i+2*size,j,3*size,size,integral))/(s*s);
            dxy[j*(img->width+1)+i]=( 1*block(i+1,j+1,size,size,integral)
                                     -1*block(i+1,j+size+2,size,size,integral)
                                     -1*block(i+size+2,j+1,size,size,integral)
                                     +1*block(i+size+2,j+size+2,size,size,integral))/(s*s);

            a=dxx[j*(img->width+1)+i];//cvGetReal2D(dxx,j,i);
            b=dyy[j*(img->width+1)+i];//cvGetReal2D(dyy,j,i);
            c=dxy[j*(img->width+1)+i];//cvGetReal2D(dyy,j,i);
//            cvSet2D(det[level],j+(int)(4.5*s+0.5),i+(int)(4.5*s+0.5),cvScalar((a*b-0.9*cvGetReal2D(dxy,j,i))/65000.0));//*cvGet2D(dxx,j,i).val[0]
            det[level*img->width*img->height+(j+(int)(1.5*size+0.5))*img->width+i+(int)(1.5*size+0.5)]=
                              (a*b-(0.9*c*0.9*c))/(65000.0);

        }
 /*      cvShowImage("win",dyy);
         cvWaitKey(0);
         cvShowImage("win",dxx);
         cvWaitKey(0);
         cvShowImage("win",dxy);
         cvWaitKey(0);
         cvShowImage("win",det[level]);
         cvWaitKey(0);*/

   }//end levels
    non_max_sup(det,max,feat,levels);
    delete[] dxx;//cvReleaseImage(&dxx);
    delete[] dyy;//cvReleaseImage(&dyy);
    delete[] dxy;//cvReleaseImage(&dxy);
    delete[] det;
   /* for(int level=0;level<levels;level++){
       cvReleaseImage(&(max[level]));
    }*/
     cout<<"end find feat"<<endl;
}
void CSurf::gaussian(IplImage* img)
{
int w = img->width;
int h = img->height;
float x,y;
float c;
c=w/2;
for (int i =0; i<w; i++)
    for (int j=0; j<h;j++)
      {
             x=i-w/2.;
             y=j-h/2.;
             cvSetReal2D(img, j,i, exp(-(x*x+y*y)/(2*c*c)));
      }
}
