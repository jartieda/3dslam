#include "model.h"
namespace SLAM{
CModel::CModel()
{
}
CModel::~CModel()
{

}
CvMat* CModel::getTransitionMatrix()
{
	return TransitionMatrix;
}
CvMat* CModel::getProcessNoiseCov()
{
	return ProcessNoiseCov;
}
CvMat* CModel::getMeasurementMatrix()
{
	return MeasurementMatrix;
}
CvMat* CModel::getMeasurementNoiseCov()
{
	return MeasurementNoiseCov;
}
int CModel::getMeasurementNum()
{
	return MeasurementNum;
}
int CModel::getStateNum()
{
	return StateNum;
}
CvMat* CModel::getInputMatrix()
{
	return InputMatrix;
}
CvMat* CModel::getInputCov()
{
	return InputCov;
}
int CModel::getInputNum()
{
	return InputNum;
}

}
