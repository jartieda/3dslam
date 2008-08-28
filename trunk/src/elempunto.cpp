#include "elempunto.h"
#include <iostream>
namespace SLAM{
/** Constructor <br>
 * Crea las matrices: dpdr, dpdt, dpdw, rot_unInit_1, rot_unInit_2, trans_unInit_1, trans_unInit_2
 **/
CElempunto::CElempunto():count(0)
{
   //visible=true;
   dpdr= cvCreateMat(2,3,CV_32FC1);
   dpdt= cvCreateMat(2,3,CV_32FC1);
   dpdw= cvCreateMat(2,6,CV_32FC1);
}

/**
 * Destructor <br>
 * Destruye las matrices: dpdr, dpdt, dpdw <br>
 **/
CElempunto::~CElempunto()
{
   cvReleaseMat(&dpdr);
   cvReleaseMat(&dpdt);
   cvReleaseMat(&dpdw);
}
}