#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include "modelcam.h"
#include "map.h"
#include "model.h"
#include "cv.h"
#include "estimator.h"
namespace SLAM{
/**
@author Jorge Artieda
@brief implementaci&oacute;n del filtro de kalman
*/

class CParticleFilter: public CEstimator {
public:
CParticleFilter();
~CParticleFilter();

void Predict();
void Correct();
void Test();
void initState();
void setModel(CModel *p);///< enlaza la clase calman con el modelo de vehículo
void UpdateMatrixSize();

double **pred_measure;///< predicted measurements. Projection of particles
int num_particles; ///number of particles per variable
int num_variables; ///number of variables
int num_measurements; ///number of measurements
double *weights;
double **particles; ///pointer to a dynamically allocated array of particles

protected:
void ProjectParticle(int n_part);

void initParticle(int part);
int num_max_variables;
int num_max_particles;
int num_max_measurements;
double **A;
/*int n_var;
int n_meas;
int n_feat;*/
int *indx_part_feat;
bool *reject;
void noise(double *v, double *s, int l);
double *rand_num;
double *var_modelo;
double threshold;
double *measure;
double *state;

double Random(double min, double max);
     double ERF[200];
int inited_vis; ///<number of visible points inited
int inited_no_vis; ///<viewed once not visible now
int fdims; ///dimension of one point
  CvMat *rotation;///<rotacion para devolver los valores de la camara
CvMat *trans;///<translacion  para devolver los valores de la camara

};
}

#endif
