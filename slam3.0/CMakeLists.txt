
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

FIND_PACKAGE(OpenCV REQUIRED )
FIND_PACKAGE(OSG REQUIRED) 

include_directories( ${OpenCV_INCLUDE_DIRS} ${OSG_INCLUDE_DIR})
add_executable (slam3 BTLocalMatcherT.h           OvImageAdapter.cpp          cslam.h
OpenCVImageAdapter.cpp      OvImageAdapter.h            cvisor.cpp
OpenCVImageAdapter.h        OvImagePairPreprocessorT.h  cvisor.h
Openvis3d.h                 OvImageT.h                  main.cpp
OvDisparityPostprocessor.h  OvLocalMatcherT.h           mainpage.h
OvFlowDiffuseMatcherT.h     OvStereoDiffuseMatcherT.h   surf.cpp
OvFlowGlobalMatcherT.h      OvStereoGlobalMatcherT.h    surf.h
OvFlowPostprocessor.h       OvStereoT.h                 xmlParser.cpp
OvFlowT.h                   cslam.cpp                   xmlParser.h )


TARGET_LINK_LIBRARIES( slam3 ${OpenCV_LIBS}  ${OSG_LIBRARY} ${OSGGA_LIBRARY} 
${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGWIDGET_LIBRARY} 
${OSGTERRAIN_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGFX_LIBRARY} ${OSGVOLUME_LIBRARY} ${OPENTHREADS_LIBRARY} )