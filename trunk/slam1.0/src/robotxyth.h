#ifndef CROBOTXYTH_H
#define CROBOTXYTH_H

#include "vehicle.h"
namespace SLAM{
class CRobotXYTh : public CVehicle
{
    public:
        CRobotXYTh();
        virtual ~CRobotXYTh();
        virtual void ReadData();

    protected:
    private:
};
}

#endif // CROBOTXYTH_H
