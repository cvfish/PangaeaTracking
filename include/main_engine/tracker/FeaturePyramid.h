#pragma once

#include "../utils/global.h"
#include "ImagePyramid.h"

#include <lmdb.h>

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
  void InitializeDB(const char* db_path);

  void setupCameraPyramid(int numLevels, CameraInfo& camInfo);
  void setupPyramid(std::string key);

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

  // lmdb
  MDB_env *mdb_env;
  MDB_dbi mdb_dbi;
  MDB_val mdb_key, mdb_data;
  MDB_txn *mdb_txn;

  //MDB_cursor* mdb_cursor;
};
