#pragma once

#include "FeatureReader.h"
#include "ImagePyramid.h"

struct FeatureLevel: public Level
{
  FeatureImageContainerType featureImageVec;
  FeatureImageContainerType featureImageGradXVec;
  FeatureImageContainerType featureImageGradYVec;
};

class FeaturePyramid
{
public:

  // has to given the path to the feature image database
  FeaturePyramid();
  ~FeaturePyramid();

  void create(int nW, int nH, int nChannels, int numLevels);
  void setupCameraPyramid(int numLevels, CameraInfo& camInfo);
  // void setupPyramid(IntensityImageType& grayImageBYTE,std::string key);
  void setupPyramid(unsigned char* pCurrentGrayImage, string key);

  void updateData();
  void updatePrev();

  CameraInfo& getCameraInfo(int nLevel);
  FeatureLevel& getPrevFeatureLevel(int nLevel);
  FeatureLevel& getCurrFeatureLevel(int nLevel);

private:

  // whether previous frame has been initialized
  bool m_bInitialized;

  FeatureLevel* prevLevels;
  FeatureLevel* currLevels;
  FeatureLevel* currLevelsBuffer;

  FeatureImageType featureBufferImage;
  FeatureImageType blurBufferImage;

  vector<CameraInfo> camInfoLevels;

  int m_nWidth;
  int m_nHeight;
  int m_nNumChannels;

  int m_nNumLevels;

  FeatureReader* pFeatureReader;

};
