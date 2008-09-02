#ifndef CROBOTXYTH_H
#define CROBOTXYTH_H

#include "vehicle.h"
namespace SLAM{
class CRobotXYTh : public CVehicle
{
    public:
        CRobotXYTh();
        virtual ~CRobotXYTh();

        virtual void ReadData(CvMat* Rob, CvMat* CamRob,CvMat* T);

    protected:
    private:
};
}
#endif // CROBOTXYTH_H
