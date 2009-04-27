#include "cvisor.h"

osg::Node* myKeyboardEventHandler::Camera(std::string filename)
    {
        // Declare and initialize a transform node.
       osg::Group* Cam = new osg::Group();

       osg::Geode* FrameGeode = new osg::Geode();
       osg::Geometry* FrameGeometry = new osg::Geometry();
       FrameGeode->addDrawable(FrameGeometry);
       //root->addChild(pyramidGeode);
       osg::Vec3Array* FrameVertices = new osg::Vec3Array;
       FrameVertices->push_back( osg::Vec3( -3, -3, 3) ); // front left
       FrameVertices->push_back( osg::Vec3(3, -3, 3) ); // front right
       FrameVertices->push_back( osg::Vec3(3,3, 3) ); // back right
       FrameVertices->push_back( osg::Vec3(-3,3, 3) ); // back left
       FrameVertices->push_back( osg::Vec3(0,0, 0) ); // back left

       FrameGeometry->setVertexArray( FrameVertices );
       osg::DrawElementsUInt* FrameBase = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
       FrameBase->push_back(3);
       FrameBase->push_back(2);
       FrameBase->push_back(1);
       FrameBase->push_back(0);
       FrameGeometry->addPrimitiveSet(FrameBase);


       osg::Vec2Array* texcoords = new osg::Vec2Array(4);
       (*texcoords)[0].set(0.00f,1.0f); // tex coord for vertex 0
       (*texcoords)[1].set(1.0f,1.0f); // tex coord for vertex 1
       (*texcoords)[2].set(1.0f,0.0f); // ""
       (*texcoords)[3].set(0.0f,0.0f); // ""
       FrameGeometry->setTexCoordArray(0,texcoords);
       FrameTexture = new osg::Texture2D;

       // protect from being optimized away as static state:
       FrameTexture->setDataVariance(osg::Object::DYNAMIC);

         // Create a new StateSet with default settings:
       osg::StateSet* stateOne = new osg::StateSet();

       // Assign texture unit 0 of our new StateSet to the texture
       // we just created and enable the texture.
       stateOne->setTextureAttributeAndModes
          (0,FrameTexture,osg::StateAttribute::ON);

       osg::Image* klnFace = osgDB::readImageFile(filename.c_str());
       if (!klnFace)
       {
          std::cout << " couldn't find texture, quiting." << std::endl;
         // exit (-1);
       }else{

       FrameTexture->setImage(klnFace);
       }
       // Associate this state set with the Geode that contains
       // the pyramid:
       FrameGeode->setStateSet(stateOne);


       /** Cam Body **/
       osg::Geode* BodyGeode = new osg::Geode();
       osg::Geometry* BodyGeometry = new osg::Geometry();
       BodyGeode->addDrawable(BodyGeometry);
       BodyGeometry->setVertexArray( FrameVertices );

       //root->addChild(pyramidGeode);
//       osg::Vec3Array* FrameVertices = new osg::Vec3Array;
//       FrameVertices->push_back( osg::Vec3( -3, -3, 3) ); // front left
//       FrameVertices->push_back( osg::Vec3(3, -3, 3) ); // front right
//       FrameVertices->push_back( osg::Vec3(3,3, 3) ); // back right
//       FrameVertices->push_back( osg::Vec3(-3,3, 3) ); // back left
//       FrameVertices->push_back( osg::Vec3(0,0, 0) ); // back left

       osg::DrawElementsUInt* FaceOne =
       new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
       FaceOne->push_back(0);
       FaceOne->push_back(1);
       FaceOne->push_back(4);
       BodyGeometry->addPrimitiveSet(FaceOne);
       osg::DrawElementsUInt* Face2 =
       new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
       Face2->push_back(1);
       Face2->push_back(2);
       Face2->push_back(4);
       BodyGeometry->addPrimitiveSet(Face2);
       osg::DrawElementsUInt* Face3 =
       new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
       Face3->push_back(2);
       Face3->push_back(3);
       Face3->push_back(4);
       BodyGeometry->addPrimitiveSet(Face3);
       osg::DrawElementsUInt* Face4 =
       new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
       Face4->push_back(3);
       Face4->push_back(0);
       Face4->push_back(4);
       BodyGeometry->addPrimitiveSet(Face4);

         // Create a new StateSet with default settings:
       osg::StateSet* stateOne2 = new osg::StateSet();
        osg::PolygonMode *pm = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
        stateOne2->setAttributeAndModes(pm);
        osg::LineWidth* lw = new osg::LineWidth(2.f);
        stateOne2->setAttribute(lw);
       // Assign texture unit 0 of our new StateSet to the texture
       // we just created and enable the texture.
       //stateOne->setTextureAttributeAndModes
        //  (0,FrameTexture,osg::StateAttribute::ON);

       // Associate this state set with the Geode that contains
       // the pyramid:
       BodyGeode->setStateSet(stateOne2);


        Cam->addChild(FrameGeode);
        Cam->addChild(BodyGeode);

        return Cam;
    }
 osg::Node *myKeyboardEventHandler::ImageHud(std::string filename)
{
	    osg::Geode * geode = new osg::Geode;
        osg::Geometry* FrameGeometry = new osg::Geometry();
       geode->addDrawable(FrameGeometry);
       //root->addChild(pyramidGeode);
       osg::Vec3Array* FrameVertices = new osg::Vec3Array;
       FrameVertices->push_back( osg::Vec3( 0, 0, 0) ); // front left
       FrameVertices->push_back( osg::Vec3(10, 0, 0) ); // front right
       FrameVertices->push_back( osg::Vec3(10,10, 0) ); // back right
       FrameVertices->push_back( osg::Vec3( 0,10, 0) ); // back left

       FrameGeometry->setVertexArray( FrameVertices );
       osg::DrawElementsUInt* FrameBase = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
       FrameBase->push_back(3);
       FrameBase->push_back(2);
       FrameBase->push_back(1);
       FrameBase->push_back(0);
       FrameGeometry->addPrimitiveSet(FrameBase);
       osg::Vec2Array* texcoords = new osg::Vec2Array(4);
       (*texcoords)[0].set(0.00f,0.0f); // tex coord for vertex 0
       (*texcoords)[1].set(1.0f,0.0f); // tex coord for vertex 1
       (*texcoords)[2].set(1.0f,1.0f); // ""
       (*texcoords)[3].set(0.0f,1.0f); // ""
       FrameGeometry->setTexCoordArray(0,texcoords);
       FrameTexture = new osg::Texture2D;

       // protect from being optimized away as static state:
       FrameTexture->setDataVariance(osg::Object::DYNAMIC);

// create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::Vec4Array* colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    FrameGeometry->setColorArray(colors);
    FrameGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

         // Create a new StateSet with default settings:
       osg::StateSet* stateOne = new osg::StateSet();

       // Assign texture unit 0 of our new StateSet to the texture
       // we just created and enable the texture.
       stateOne->setTextureAttributeAndModes
          (0,FrameTexture,osg::StateAttribute::ON);

       osg::Image* klnFace = osgDB::readImageFile(filename.c_str());
       if (!klnFace)
       {
          std::cout << " couldn't find texture, quiting." << std::endl;
          //exit (-1);
       }
        else
       FrameTexture->setImage(klnFace);

       // Associate this state set with the Geode that contains
       // the pyramid:
       geode->setStateSet(stateOne);

	    osg::StateSet * stateset = geode->getOrCreateStateSet();
	    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	    stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	    stateset->setRenderBinDetails(11, "RenderBin");

	    osg::MatrixTransform * modelview = new osg::MatrixTransform;
	    modelview->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	    osg::Matrixd matrix(osg::Matrixd::scale(50,50,50) * osg::Matrixd::translate(10,10,0)); // I've played with these values a lot and it seems to work, but I have no idea why
	    modelview->setMatrix(matrix);
	    modelview->addChild(geode);

	    osg::Projection * projection = new osg::Projection;
	    projection->setMatrix(osg::Matrix::ortho2D(0,1280,0,1024)); // or whatever the OSG window res is
	    projection->addChild(modelview);

	    return projection; //make sure you delete the return sb line

}
 osg::Node *myKeyboardEventHandler::P_Hud(std::string filename)
{
	    osg::Geode * geode = new osg::Geode;
        osg::Geometry* FrameGeometry = new osg::Geometry();
       geode->addDrawable(FrameGeometry);
       //root->addChild(pyramidGeode);
       osg::Vec3Array* FrameVertices = new osg::Vec3Array;
       FrameVertices->push_back( osg::Vec3( 0, 0, 0) ); // front left
       FrameVertices->push_back( osg::Vec3(10, 0, 0) ); // front right
       FrameVertices->push_back( osg::Vec3(10,10, 0) ); // back right
       FrameVertices->push_back( osg::Vec3( 0,10, 0) ); // back left

       FrameGeometry->setVertexArray( FrameVertices );
       osg::DrawElementsUInt* FrameBase = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
       FrameBase->push_back(3);
       FrameBase->push_back(2);
       FrameBase->push_back(1);
       FrameBase->push_back(0);
       FrameGeometry->addPrimitiveSet(FrameBase);
       osg::Vec2Array* texcoords = new osg::Vec2Array(4);
       (*texcoords)[0].set(0.00f,0.0f); // tex coord for vertex 0
       (*texcoords)[1].set(1.0f,0.0f); // tex coord for vertex 1
       (*texcoords)[2].set(1.0f,1.0f); // ""
       (*texcoords)[3].set(0.0f,1.0f); // ""
       FrameGeometry->setTexCoordArray(0,texcoords);
       FrameTexture = new osg::Texture2D;

       // protect from being optimized away as static state:
       FrameTexture->setDataVariance(osg::Object::DYNAMIC);

// create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::Vec4Array* colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    FrameGeometry->setColorArray(colors);
    FrameGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

         // Create a new StateSet with default settings:
       osg::StateSet* stateOne = new osg::StateSet();

       // Assign texture unit 0 of our new StateSet to the texture
       // we just created and enable the texture.
       stateOne->setTextureAttributeAndModes
          (0,FrameTexture,osg::StateAttribute::ON);

       osg::Image* klnFace = osgDB::readImageFile(filename.c_str());
       if (!klnFace)
       {
          std::cout << " couldn't find texture, quiting." << std::endl;
          //exit (-1);
       }
        else
       FrameTexture->setImage(klnFace);

       // Associate this state set with the Geode that contains
       // the pyramid:
       geode->setStateSet(stateOne);

	    osg::StateSet * stateset = geode->getOrCreateStateSet();
	    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	    stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	    stateset->setRenderBinDetails(11, "RenderBin");

	    osg::MatrixTransform * modelview = new osg::MatrixTransform;
	    modelview->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	    osg::Matrixd matrix(osg::Matrixd::scale(40,40,40) * osg::Matrixd::translate(600,10,0)); // I've played with these values a lot and it seems to work, but I have no idea why
	    modelview->setMatrix(matrix);
	    modelview->addChild(geode);

	    osg::Projection * projection = new osg::Projection;
	    projection->setMatrix(osg::Matrix::ortho2D(0,1280,0,1024)); // or whatever the OSG window res is
	    projection->addChild(modelview);

	    return projection; //make sure you delete the return sb line

}
osg::Geode* myKeyboardEventHandler::PlotEllipse3D(CvMat *X, CvMat *P, osg::Vec4Array* colors )
{
    osg::Geode* geode = new osg::Geode();
    osg::Geometry* pointsGeom = new osg::Geometry();
    osg::Vec3Array* vertices = new osg::Vec3Array;
    CvMat *temp;
    temp=cvCreateMatHeader(3,3,CV_32FC1);
    CvMat *_temp = cvCreateMat (3,3,CV_32FC1);
    CvMat* vect=cvCreateMat (3,1,CV_32FC1);
    CvMat* res6=cvCreateMat (3,1,CV_32FC1);
    //CvMat* vect2=cvCreateMat(1,6,CV_32FC1);
    //CvPoint2D64f proj ;
    //CvMat* m = cvCreateMat(3,1,CV_32FC1);
    float *fstate;
    fstate= X->data.fl;
    float xc,yc,zc;
    xc=fstate[0];
    yc=fstate[1];
    zc=fstate[2];

    cvGetSubRect( P,temp,cvRect(0,0,3,3) );

//    for (int part=0; part<100; part++){
    for(float t = 0; t<2*3.14159; t+=0.1)
        for (float p = -3.14159/2; p<3.14159;p+=0.1)
        {
        cvmSet(vect,0,0,7.8*cos(p)*cos(t));
        cvmSet(vect,1,0,7.8*cos(p)*sin(t));
        cvmSet(vect,2,0,7.8*sin(p));
        cvZero(_temp);
        pSlam->Cholesky(temp,_temp);
        cvMatMul(_temp,vect,res6);

        double ox=cvmGet(res6,0,0)+xc;
        double oy=cvmGet(res6,1,0)+yc;
        double oz=cvmGet(res6,2,0)+zc;

        if (ox> -100000 && ox<10000 &&
            oy> -100000 && oy<10000 &&
            oz> -100000 && oz<10000)
        {
            CvPoint3D64f point = cvPoint3D64f(ox,oy,oz);
            vertices->push_back(osg::Vec3(ox,oy,oz));
        }
    }

    // pass the created vertex array to the points geometry object.
    pointsGeom->setVertexArray(vertices);

    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    pointsGeom->setNormalArray(normals);
    pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);


    // create and add a DrawArray Primitive (see include/osg/Primitive).  The first
    // parameter passed to the DrawArrays constructor is the Primitive::Mode which
    // in this case is POINTS (which has the same value GL_POINTS), the second
    // parameter is the index position into the vertex array of the first point
    // to draw, and the third parameter is the number of points to draw.
    pointsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));


    // add the points geometry to the geode.
    geode->addDrawable(pointsGeom);
    // Create a new StateSet with default settings:
    osg::StateSet* stateOne = new osg::StateSet();
    osg::Point *  point = new osg::Point();
    point->setSize(1.0);
    stateOne->setAttributeAndModes(point,osg::StateAttribute::ON);
    geode->setStateSet(stateOne);
    return geode;

}
osg::Geode* myKeyboardEventHandler::Uncertainty()
{
    osg::Geode* geode = new osg::Geode();
    osg::Geometry* pointsGeom = new osg::Geometry();

    osg::Vec3Array* vertices = new osg::Vec3Array;

    CvMat *temp,*temp2,*temp3;
    temp=cvCreateMatHeader(6,6,CV_32FC1);
    CvMat *_temp = cvCreateMat (6,6,CV_32FC1);
    temp2=cvCreateMat(3,6,CV_32FC1);
    temp3=cvCreateMat(3,3,CV_32FC1);
    CvMat* vect=cvCreateMat (6,1,CV_32FC1);
    CvMat* res6=cvCreateMat (6,1,CV_32FC1);
    //CvMat* vect2=cvCreateMat(1,6,CV_32FC1);
    //CvPoint2D64f proj ;
    //CvMat* m = cvCreateMat(3,1,CV_32FC1);
    float *fstate;
    fstate= pSlam->X->data.fl;
    float xc,yc,zc,theta,phi,rho;

    for (unsigned int i = 0 ; i < pSlam->visible.size(); i++)
    {
        if(pSlam->visible[i]==true)
        {
            xc=fstate[pSlam->modelSD+6*i];
            yc=fstate[pSlam->modelSD+6*i+1];
            zc=fstate[pSlam->modelSD+6*i+2];
            theta=fstate[pSlam->modelSD+6*i+3];
            phi=fstate[pSlam->modelSD+6*i+4];
            rho=fstate[pSlam->modelSD+6*i+5];


            cvGetSubRect( pSlam->P,temp,cvRect(pSlam->modelSD+i*6,pSlam->modelSD+i*6,6,6) );

            for (int part=0; part<40; part++){
//                cvmSet(vect,0,0,pSlam->randomVector(-.005,.005));
//                cvmSet(vect,1,0,pSlam->randomVector(-.005,.005));
//                cvmSet(vect,2,0,pSlam->randomVector(-.005,.005));
//                cvmSet(vect,3,0,pSlam->randomVector(-.05,0.05));
//                cvmSet(vect,4,0,pSlam->randomVector(-.05,0.05));
//                cvmSet(vect,5,0,pSlam->randomVector(-1,1));
                cvmSet(vect,0,0,pSlam->randomVector(-12.6,12.6));
                cvmSet(vect,1,0,pSlam->randomVector(-12.6,12.6));
                cvmSet(vect,2,0,pSlam->randomVector(-12.6,12.6));
                cvmSet(vect,3,0,pSlam->randomVector(-12.6,12.6));
                cvmSet(vect,4,0,pSlam->randomVector(-12.6,12.6));
                cvmSet(vect,5,0,pSlam->randomVector(-12.6,12.6));
                cvZero(_temp);
                pSlam->Cholesky(temp,_temp);
                cvMatMul(_temp,vect,res6);

                double ox, oy, oz;
                pSlam->InverseDepth2Depth(cvmGet(res6,0,0)+xc,
                                          cvmGet(res6,1,0)+yc,
                                          cvmGet(res6,2,0)+zc,
                                          cvmGet(res6,3,0)+theta,
                                          cvmGet(res6,4,0)+phi,
                                          cvmGet(res6,5,0)+rho,
                                          &ox,&oy,&oz );

                //CvMat* nullmat = 0;

                if (ox> -100000 && ox<10000 &&
                    oy> -100000 && oy<10000 &&
                    oz> -100000 && oz<10000)
                {
                    CvPoint3D64f point = cvPoint3D64f(ox,oy,oz);
                    vertices->push_back(osg::Vec3(ox,oy,oz));
                }
            }
        }
    }

    // pass the created vertex array to the points geometry object.
    pointsGeom->setVertexArray(vertices);
    // create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::Vec4Array* colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    colors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    pointsGeom->setNormalArray(normals);
    pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);


    // create and add a DrawArray Primitive (see include/osg/Primitive).  The first
    // parameter passed to the DrawArrays constructor is the Primitive::Mode which
    // in this case is POINTS (which has the same value GL_POINTS), the second
    // parameter is the index position into the vertex array of the first point
    // to draw, and the third parameter is the number of points to draw.
    pointsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));


    // add the points geometry to the geode.
    geode->addDrawable(pointsGeom);
    // Create a new StateSet with default settings:
    osg::StateSet* stateOne = new osg::StateSet();
    osg::Point *  point = new osg::Point();
    point->setSize(1.0);
    stateOne->setAttributeAndModes(point,osg::StateAttribute::ON);
    geode->setStateSet(stateOne);
    return geode;
}
osg::Geode* myKeyboardEventHandler::Feature()
{
    osg::Geode* geode = new osg::Geode();
    osg::Geometry* pointsGeom = new osg::Geometry();
    float *fstate;
    fstate = pSlam->X->data.fl;
    int step = pSlam->X->step/sizeof(fstate[0]);
    fstate+=(pSlam->modelSD*step);

    // create a Vec3Array and add to it all my coordinates.
    // Like all the *Array variants (see include/osg/Array) , Vec3Array is derived from both osg::Array
    // and std::vector<>.  osg::Array's are reference counted and hence sharable,
    // which std::vector<> provides all the convenience, flexibility and robustness
    // of the most popular of all STL containers.
    osg::Vec3Array* vertices = new osg::Vec3Array;
    double x,y,z,theta,phi,rho;

    double ox,oy,oz;
    for (unsigned int i = 0 ; i < pSlam->visible.size(); i++)
    {
        x=*fstate;
        fstate++;
        y=*fstate;
        fstate++;
        z=*fstate;
        fstate++;
        theta=*fstate;
        fstate++;
        phi=*fstate;
        fstate++;
        rho=*fstate;
        fstate++;
        pSlam->InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
        vertices->push_back(osg::Vec3(ox,oy,oz));
    }

    // pass the created vertex array to the points geometry object.
    pointsGeom->setVertexArray(vertices);

    // create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::Vec4Array* colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    pointsGeom->setNormalArray(normals);
    pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // create and add a DrawArray Primitive (see include/osg/Primitive).  The first
    // parameter passed to the DrawArrays constructor is the Primitive::Mode which
    // in this case is POINTS (which has the same value GL_POINTS), the second
    // parameter is the index position into the vertex array of the first point
    // to draw, and the third parameter is the number of points to draw.
    pointsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    // add the points geometry to the geode.
    geode->addDrawable(pointsGeom);
    // Create a new StateSet with default settings:
    osg::StateSet* stateOne = new osg::StateSet();
    osg::Point *  point = new osg::Point();
    point->setSize(2.0);
    stateOne->setAttributeAndModes(point,osg::StateAttribute::ON);
    geode->setStateSet(stateOne);
    return geode;

}

osg::Geode* myKeyboardEventHandler::LastFeaturePos()
{
    osg::Geode* geode = new osg::Geode();
    osg::Geometry* pointsGeom = new osg::Geometry();
    float *fstate;
    fstate = pSlam->X->data.fl;
    int step = pSlam->X->step/sizeof(fstate[0]);
    fstate+=(pSlam->modelSD*step);

    // create a Vec3Array and add to it all my coordinates.
    // Like all the *Array variants (see include/osg/Array) , Vec3Array is derived from both osg::Array
    // and std::vector<>.  osg::Array's are reference counted and hence sharable,
    // which std::vector<> provides all the convenience, flexibility and robustness
    // of the most popular of all STL containers.
    osg::Vec3Array* vertices = new osg::Vec3Array;
    double x,y,z,theta,phi,rho;

    double ox,oy,oz;
    for (unsigned int i = 0 ; i < pSlam->visible.size(); i++)
    {
        x=*fstate;
        fstate++;
        y=*fstate;
        fstate++;
        z=*fstate;
        fstate++;
        theta=*fstate;
        fstate++;
        phi=*fstate;
        fstate++;
        rho=*fstate;
        fstate++;
        pSlam->InverseDepth2Depth(x,y,z,theta,phi,rho,&ox,&oy,&oz);
        vertices->push_back(osg::Vec3(ox,oy,oz));
    }

    // pass the created vertex array to the points geometry object.
    pointsGeom->setVertexArray(vertices);

    // create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::Vec4Array* colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    colors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f));

    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    pointsGeom->setNormalArray(normals);
    pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // create and add a DrawArray Primitive (see include/osg/Primitive).  The first
    // parameter passed to the DrawArrays constructor is the Primitive::Mode which
    // in this case is POINTS (which has the same value GL_POINTS), the second
    // parameter is the index position into the vertex array of the first point
    // to draw, and the third parameter is the number of points to draw.
    pointsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    // add the points geometry to the geode.
    geode->addDrawable(pointsGeom);
    // Create a new StateSet with default settings:
    osg::StateSet* stateOne = new osg::StateSet();
    osg::Point *  point = new osg::Point();
    point->setSize(4.0);
    stateOne->setAttributeAndModes(point,osg::StateAttribute::ON);
    geode->setStateSet(stateOne);
    return geode;

}

    bool myKeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
     {
       switch(ea.getEventType())
       {
       case(osgGA::GUIEventAdapter::KEYDOWN):
          {
            char c = ea.getKey();
            if (c=='l')
            {
               osg::StateSet * stateset = root->getOrCreateStateSet();
                 stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            }
            if (c=='k')
            {
               osg::StateSet * stateset = root->getOrCreateStateSet();
                 stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
            }
            if (c=='c')
            {

                std::cout << " c key pressed "<< std::endl;
                printf("%s\n",DATA);
                sprintf(filein,DATA,iter);
                printf("%s\n",filein);
                frame=cvLoadImage(filein );
                while (frame == NULL){
                    printf("error abriendo fichero\n");
                     printf("%s\n",DATA);
                    sprintf(filein,DATA,++iter);
                    printf("%s\n",filein);
                    frame=cvLoadImage(filein );
                }
               // Assign the texture to the image we read from file:
               pSlam->run(frame);
               osg::PositionAttitudeTransform* FrameXForm = new osg::PositionAttitudeTransform();
               root->addChild(FrameXForm);
               FrameXForm->addChild(Camera(std::string(filein)));
               double xcam= 0;
               double ycam = 0;
               double zcam = 0;
                CvMat * r = cvCreateMat(4,1,CV_32FC1);
                CvMat * t = cvCreateMat(3,1,CV_32FC1);
                CvMat * R = cvCreateMat(3,3,CV_32FC1);

               if (pSlam->X != NULL){
                   pSlam->getTransRot (pSlam->X,t,r,R);

                    xcam = cvmGet(t,0,0);
                    ycam = cvmGet(t,1,0);
                    zcam = cvmGet(t,2,0);
                    root->addChild(Feature());
                    UncertGeode = Uncertainty();
                    if (root->containsNode(OldUncertGeode)){
                        root->replaceChild(OldUncertGeode,UncertGeode);
                    }else{
                        root->addChild(UncertGeode);
                    }
                    OldUncertGeode = UncertGeode;

                    osg::Vec4Array* blue = new osg::Vec4Array;
                    blue->push_back(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
                    UncertCameraGeode = PlotEllipse3D(pSlam->X,pSlam->P,blue);
                    if (root->containsNode(OldUncertCameraGeode)){
                        root->replaceChild(OldUncertCameraGeode,UncertCameraGeode);
                    }else{
                        root->addChild(UncertCameraGeode);
                    }
                    OldUncertCameraGeode = UncertCameraGeode;

                    osg::Vec4Array* green = new osg::Vec4Array;
                    green->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
                    UncertCameraPredGeode = PlotEllipse3D(pSlam->Xp,pSlam->Pp,green);
                    if (root->containsNode(OldUncertCameraPredGeode)){
                        root->replaceChild(OldUncertCameraPredGeode,UncertCameraPredGeode);
                    }else{
                        root->addChild(UncertCameraPredGeode);
                    }
                    OldUncertCameraPredGeode = UncertCameraPredGeode;
                    LastFeaturePosGeode=LastFeaturePos();
                    if (root->containsNode(OldLastFeaturePosGeode)){
                        root->replaceChild(OldLastFeaturePosGeode,LastFeaturePosGeode);
                    }else{
                        root->addChild(LastFeaturePosGeode);
                    }
                    OldLastFeaturePosGeode = LastFeaturePosGeode;

               }
               root->addChild(ImageHud(std::string("slam.jpg")));
               //root->addChild(P_Hud(std::string("slam_p.tif")));
               osg::Vec3 FramePosition(xcam,ycam,zcam);
               FrameXForm->setPosition( FramePosition );
               osg::Quat FrameRotation(-cvmGet(r,1,0),-cvmGet(r,2,0),-cvmGet(r,3,0),cvmGet(r,0,0));
               FrameXForm->setAttitude( FrameRotation);

                std::cout<<"Visible: "<<pSlam->visNum<<std::endl;
                iter+=1;
                cvWaitKey(100);//esto probablemente se pueda quitar
                return false;
            }else{
                return false;
            }
          }
       default:
          return false;
       }
    }

cvisor::cvisor()
{
    //ctor
	//Creating the root node
	osg::ref_ptr<osg::Group> root (new osg::Group);

	//The geode containing our shpae
   	osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);

    viewer = new osgViewer::Viewer();
    viewer->setSceneData( root.get() );
    myFirstEventHandler = new myKeyboardEventHandler();
    myFirstEventHandler->FrameTexture=FrameTexture ;
    myFirstEventHandler->root=root.get();
//  myFirstEventHandler->FrameGeode= FrameGeode;
    viewer->addEventHandler(myFirstEventHandler);
}
void cvisor:: setEventArgs()
{
    myFirstEventHandler->pSlam=pSlam;
    myFirstEventHandler->filein=filein;
    myFirstEventHandler->DATA=DATA;
    myFirstEventHandler->iter=1;
}
void cvisor::run()
{
    viewer->run();
}
cvisor::~cvisor()
{
    //dtor
}
