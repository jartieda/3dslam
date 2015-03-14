#ifndef CSURF_H
#define CSURF_H

#include <cv.h>
#include <highgui.h>
#include <cstdlib>
#include <iostream>
#include <vector>
/**
@author Jorge Artieda

** This class implements a scale and rotation inveriant interest point detector and
* descriptor. The implementation is based on de paper "SURF: Speeded Up Robust Features"
written by Herbert Bay et al. from ETH Zurich and Katholieke Universitet Leuven.
* A it is stated on the abstract of this paper SURF DETECTOR "Approximates or even outperforms previously
* proposed  schemes with respect to repeatability, distinctiveness, and robustness, yet
* can be computed an compared much faster. This is achieved by ralying on integral images for image convolutions;
* by building on th strengths of the leading exiting detectors and descriptors (in casu, using a Hessian
* matrix-based measure for the detector , and a distribution-based descriptor);
* an dby simplifying these methods to the esential. This leads to a combination of
* novel detection description, and matching steps."
* @brief SURF: Speeded Up Robust Features
*/
class CSurf
{
      public:

      CSurf();
      ~CSurf();
double block(int x, int y, int h, int w,IplImage *I);
double wl_vert(int x, int y,int s);
double wl_horz(int x, int y,int s);
float orientation(int x, int y, int s);
int *descriptor(int x, int y, int s,float dir);

float horiz(int x, int y,int s, IplImage* region);
float vert(int x, int y,int s, IplImage* region);

void non_max_sup(double *det, IplImage **max, CvSeq* feat,int levels);
void find_features(IplImage* img,CvSeq*feat,int levels);
int nearest_neighbor_classify(int *query, int *example_pairs,int n_examples) ;
int nearest_neighbor_classify3(int *query, int *example_pairs,int n_examples,std::vector<int> &result);
int nearest_neighbor_2(int *query, int *example_pairs,int n_examples,CvPoint p,CvMat *newmat,float dist) ;
void gaussian(IplImage* img);
IplImage  *img;
IplImage *integral;
int det_thres;
double cur_thres;
};

#endif
