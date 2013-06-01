
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable (slam3 BTLocalMatcherT.h           OvImageAdapter.cpp          cslam.h
OpenCVImageAdapter.cpp      OvImageAdapter.h            cvisor.cpp
OpenCVImageAdapter.h        OvImagePairPreprocessorT.h  cvisor.h
Openvis3d.h                 OvImageT.h                  main.cpp
OvDisparityPostprocessor.h  OvLocalMatcherT.h           mainpage.h
OvFlowDiffuseMatcherT.h     OvStereoDiffuseMatcherT.h   surf.cpp
OvFlowGlobalMatcherT.h      OvStereoGlobalMatcherT.h    surf.h
OvFlowPostprocessor.h       OvStereoT.h                 xmlParser.cpp
OvFlowT.h                   cslam.cpp                   xmlParser.h )

