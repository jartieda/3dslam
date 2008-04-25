#ifndef CTRACKERFILE_H
#define CTRACKERFILE_H

#include "tracker.h"

class CTrackerFile : public CTracker
{
    public:
        CTrackerFile();
        virtual ~CTrackerFile();
        virtual void Match(IplImage *f);
        virtual int Init(IplImage *img,int **keys,CvMat **points);
        virtual void Descriptor(IplImage *img, CvPoint *point,int s, int *key);
        virtual int getFeatDim();

    protected:
        FILE *f;
        int frame;
        bool first;
    private:
};

#endif // CTRACKERFILE_H
