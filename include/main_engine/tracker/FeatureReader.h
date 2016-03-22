#ifndef _FEATURE_READER_H
#define _FEATURE_READER_H

#include "../utils/global.h"

class FeatureReader
{

public:

  FeatureReader(){};
  FeatureReader(string featureFile){};
  virtual ~FeatureReader(){};

  virtual void InitializeDB(int height, int width, int numChannels) = 0;
  virtual void ShutDownDB() = 0;

  virtual void getFeatureLevel(string key, int channel,
                               FeatureImageType& featureBufferImage) = 0;

  int m_nWidth;
  int m_nHeight;
  int m_nNumChannels;

};

class LMDBReader : public FeatureReader
{

public:

  // for lmdb, we only need the folder of the lock files
  LMDBReader(string folder);
  ~LMDBReader();

  void InitializeDB(int height, int width, int numChannels);
  void ShutDownDB();

  void getFeatureLevel(string key, int channel,
                       FeatureImageType& featureBufferImage);

private:

  string db_path;

  // lmdb
  MDB_env *mdb_env;
  MDB_dbi mdb_dbi;
  MDB_val mdb_key, mdb_data;
  MDB_txn *mdb_txn;

  //MDB_cursor* mdb_cursor;

};

class HDF5Reader : public FeatureReader
{

public:

  HDF5Reader(string file);
  ~HDF5Reader();

  void InitializeDB(int height, int width, int numChannels);
  void ShutDownDB();

  void getFeatureLevel(string key, int channel,
                       FeatureImageType& featureBufferImage);

private:

  string hdf5file;

  hid_t   file_id, dataset_id;  /* identifiers */
  hid_t   attr;
  herr_t  ret;                /* Return value */

  double* pFeatureImages;
  string currentKey;

};

#endif
