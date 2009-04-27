#ifndef CVISOR_H
#define CVISOR_H

#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/Point>
#include <osg/MatrixTransform>
#include <osg/ref_ptr>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <iostream>
#include "cslam.h"
#include <string>

class myKeyboardEventHandler : public osgGA::GUIEventHandler
    {
    public:
       virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
       virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };
        CSlam* pSlam;
        char* filein;
        const char *DATA;
        int iter;
        IplImage* frame ;
        osg::Texture2D* FrameTexture ;
        osg::Group* root;
        osg::Node* Camera(std::string filename);
        osg::Geode* Feature();
        osg::Geode* LastFeaturePos();
        osg::Geode* Uncertainty();
        osg::Node* ImageHud(std::string filename);
        osg::Geode* FrameGeode;
        osg::ref_ptr<osg::Geode> UncertGeode;
        osg::ref_ptr<osg::Geode> OldUncertGeode;
        osg::ref_ptr<osg::Geode> UncertCameraGeode;
        osg::ref_ptr<osg::Geode> OldUncertCameraGeode;
        osg::ref_ptr<osg::Geode> UncertCameraPredGeode;
        osg::ref_ptr<osg::Geode> OldUncertCameraPredGeode;
        osg::ref_ptr<osg::Geode> LastFeaturePosGeode;
        osg::ref_ptr<osg::Geode> OldLastFeaturePosGeode;

        osg::Geode* PlotEllipse3D(CvMat *X, CvMat *P, osg::Vec4Array* colors );
        osg::Node *P_Hud(std::string filename);

    };

class cvisor
{
    public:
        cvisor();
        virtual ~cvisor();
        osgViewer::Viewer* viewer;
        void run();
        CSlam* pSlam;
        char* filein;
        const char *DATA;
        void setEventArgs();
        myKeyboardEventHandler* myFirstEventHandler;
        osg::Texture2D* FrameTexture ;
    protected:
    private:
};

#endif // CVISOR_H
