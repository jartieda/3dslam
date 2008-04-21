#include "model.h"

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
void CModel::setModelCam(CModelCam *p)
{
	pModelCam=p;
}
void CModel::setDataCam(CDataCam *p)
{
	pDataCam=p;
}
void CModel::setMap(CMap *p)
{
	pMap=p;
}
