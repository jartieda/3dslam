#include <iostream>
#include "cslam.h"
#include <cv.h>
#include <highgui.h>
#include "xmlParser.h"
#include "cvisor.h"

using namespace std;

int main(int argc,char *argv[])
{
    cvNamedWindow( "SLAM", 1 );
    //IplImage* frame = 0;
    //int iter =1;

    XMLNode xMainNode;
    if (argc>1)
       xMainNode=XMLNode::openFileHelper(argv[1],"mainslam");
    else
        xMainNode=XMLNode::openFileHelper("config.xml","mainslam");
    /** Search for data dir in config.xml **/
    const char *DATA;
    DATA=xMainNode.getChildNode("data").getText();
    cout <<"DATA IS : " <<DATA<<endl;

    CSlam *pSlam;
    XMLNode slamNode;
    slamNode = xMainNode.getChildNode("slam");
    pSlam = new CSlam(&slamNode);
    char filein[400];
    cvisor mvisor;
    mvisor.pSlam = pSlam;
    mvisor.filein= filein;
    mvisor.DATA=DATA;
    mvisor.setEventArgs();
    mvisor.run();
    return 0;
}
