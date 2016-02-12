#ifndef _TRACKING_ENGINE_H
#define _TRACKING_ENGINE_H

#include "./Mesh.h"
#include "main_engine/utils/settings.h"

class TrackingEngine
{

public:

  TrackingEngine(){};
  virtual ~TrackingEngine(){};

  virtual bool setCurrentFrame(int curFrame) = 0; // only useful when loading stuff
  virtual void setIntrinsicMatrix(double K[3][3]) = 0; // set camera parameters

  virtual void setInitialMesh(PangaeaMeshData& mesh) = 0;
  virtual void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid) = 0;
  virtual bool trackFrame(int nFrame, unsigned char* pColorImageRGB,
                          TrackerOutputInfo** pOutputInfo) = 0;

  virtual void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                                    int nRenderLevel, bool renderType = false) = 0;

};

class ShapesBufferReader : public TrackingEngine
{

public:

  ShapesBufferReader(ShapeLoadingSettings& settings, int width, int height, double K[3][3],
                     int startFrame, int numTrackingFrames);
  ~ShapesBufferReader();

  void Initialization();
  void readModelInfo();
  void loadShapesBuffer(bool isGT, std::string& shapePath, std::string& shapeFormat,
                        int startFrame, int numTrackingFrames);
  bool setCurrentFrame(int curFrame);
  void setIntrinsicMatrix(double K[3][3]);
  void setTextureColors(unsigned char* pColorImageRGB);
  void setInitialMesh(PangaeaMeshData& mesh){};
  void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid){};

  void trackerInitSetup(TrackerOutputInfo& outputInfo);
  void trackerUpdate(TrackerOutputInfo& outputInfo);
  bool trackFrame(int nFrame, unsigned char* pColorImageRGB, TrackerOutputInfo** pOutputInfo);

  void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                            int nRenderLevel, bool renderType = false){};

  void projShapes(int numTrackingFrames);
  void getGTCenter(double* pGTCenter);
  void getESTCenter(double* pESTCenter);

  CoordinateType* getTextureColors();
  CoordinateType* getTrackingResult();
  CoordinateType* getGTTrackingResult();

  CoordinateType* getProjResult();
  CoordinateType* getGTProjResult();

  CoordinateType* getNormals();
  CoordinateType* getGTNormals();

  void computeNormals(bool isGT, unsigned int radius);

  CoordinateType* m_pTrackingResultsBuffer;
  CoordinateType* m_pTrackingResultsBufferGT;

  CoordinateType* m_pTrackingResultsNormal;
  CoordinateType* m_pTrackingResultsNormalGT;

  CoordinateType* m_pProjResultsBuffer;
  CoordinateType* m_pProjResultsBufferGT;

  CoordinateType* m_pTextureColors;

  int startFrameNo;
  int currentFrameNo;
  int totalFrameNo;

  bool m_hasGT;
  bool m_colMajor;

  int m_nHeight;
  int m_nWidth;

  std::string shapePath;
  std::string shapeFormat;
  std::string shapeFormatGT;

  std::string gtModelFile;
  std::string solModelFile;
  std::string labelColorFile;

  bool useMask;
  std::string shapeMaskFile;
  std::vector<double> maskImage;

  double centerGT[3];
  double centerEST[3];

  double KK[3][3];

  int m_nModelNum;
  vector<CoordinateType> m_vModelColors; // normalized to [0,1]
  vector<unsigned int> m_vModelGT;
  vector<unsigned int> m_vModelEST;

  double samplingScale;
  bool isTextured;

  bool trackerInitialized;

  TrackerOutputInfo outputInfo;

  int nFrameStep;

  int m_nGoodFrames;  //the number of frames successfully loaded in buffer

};

class ShapeSequenceReader : public TrackingEngine
{

public:

  ShapeSequenceReader(ShapeLoadingSettings& settings, int width, int height, double K[3][3],
                      int startFrame, int numTrackingFrames);
  ~ShapeSequenceReader();

  void Initialization();
  void readModelInfo();
  bool loadShape(bool isGT, std::string& shapePath, std::string& shapeFormat, int curFrame);
  bool setCurrentFrame(int curFrame);
  void setIntrinsicMatrix(double K[3][3]);
  void setTextureColors(unsigned char* pColorImageRGB);
  void setInitialMesh(PangaeaMeshData& mesh){};
  void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid){};

  void trackerInitSetup(TrackerOutputInfo& outInfo);
  void trackerUpdate(TrackerOutputInfo& outputInfo);
  bool trackFrame(int nFrame, unsigned char* pColorImageRGB, TrackerOutputInfo** pOutputInfo);
  void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                            int nRenderLevel, bool renderType = false){};

  void projShape();

  void computeNormal(bool isGT, unsigned int radius);
  void getGTCenter(double* pGTCenter);
  void getESTCenter(double* pESTCenter);

  CoordinateType* getTextureColors();
  CoordinateType* getTrackingResult();
  CoordinateType* getGTTrackingResult();

  CoordinateType* getProjResult();
  CoordinateType* getGTProjResult();

  CoordinateType* getNormals();
  CoordinateType* getGTNormals();

  CoordinateType* m_pCurTrackingResult;
  CoordinateType* m_pCurTrackingResultGT;

  CoordinateType* m_pCurTrackingResultNormal;
  CoordinateType* m_pCurTrackingResultNormalGT;

  CoordinateType* m_pCurProjResult;
  CoordinateType* m_pCurProjResultGT;

  CoordinateType* m_pTextureColors;

  int startFrameNo;
  int currentFrameNo;
  int totalFrameNo;

  bool m_hasGT;
  bool m_colMajor;

  int m_nHeight;
  int m_nWidth;

  std::string shapePath;
  std::string shapeFormat;
  std::string shapeFormatGT;

  std::string gtModelFile;
  std::string solModelFile;
  std::string labelColorFile;

  bool useMask;
  std::string shapeMaskFile;
  std::vector<double> maskImage;

  double centerGT[3];
  double centerEST[3];

  double KK[3][3];

  int m_nModelNum;
  vector<CoordinateType> m_vModelColors; // normalized to [0,1]
  vector<unsigned int> m_vModelGT;
  vector<unsigned int> m_vModelEST;

  double samplingScale;
  bool isTextured;

  bool trackerInitialized;

  TrackerOutputInfo outputInfo;

};

class MeshSequenceReader : public TrackingEngine
{
public:

  MeshSequenceReader(MeshLoadingSettings& settings, int width, int height, double K[3][3],
                     int startFrame, int numTrackingFrames);
  virtual ~MeshSequenceReader();

  bool setCurrentFrame(int curFrame);
  void setIntrinsicMatrix(double K[3][3]);

  void setInitialMesh(PangaeaMeshData& mesh){};
  void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid){};
  bool trackFrame(int nFrame, unsigned char* pColorImageRGB,
                  TrackerOutputInfo** pOutputInfo);
  void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                            int nRenderLevel, bool renderType = false){};

  bool loadMesh(std::string& meshPath, std::string& meshFormat, int curFrame);
  void trackerInitSetup(TrackerOutputInfo& outputInfo);
  void trackerUpdate(TrackerOutputInfo& outputInfo);

private:

  bool trackerInitialized;
  CoordinateType KK[3][3];

  int startFrameNo;
  int currentFrameNo;

  int m_nWidth;
  int m_nHeight;

  // current loaded mesh,
  // already transformed
  vector< vector<unsigned int> > visibilityFacesTest;
  vector<bool> visibilityMask;
  CoordinateType camPose[6];
  PangaeaMeshData currentMesh;

  unsigned char* pCurrentColorImageRGB;
  InternalColorImageType colorImage; // 0-1, RGB order
  InternalIntensityImageType colorImageSplit[3];

  bool useVisibilityMask;

  TrackerOutputInfo outputInfo;

};

class MeshPyramidReader : public TrackingEngine
{
public:

  MeshPyramidReader(MeshLoadingSettings& settings,int width, int height, double K[3][3],
                    int startFrame, int numTrackingFrames);
  virtual ~MeshPyramidReader();

  bool setCurrentFrame(int curFrame);
  void setIntrinsicMatrix(double K[3][3]);

  void setInitialMesh(PangaeaMeshData& mesh){};
  void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid){};
  bool trackFrame(int nFrame, unsigned char* pColorImageRGB,
                  TrackerOutputInfo** pOutputInfo);
  bool loadMeshPyramid(string meshPath, string meshLevelFormat,
                       int frame, IntegerContainerType& meshLevelList);
  void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                            int nRenderLevel, bool renderType = false);

  void setMeshPyramid();

private:

  bool trackerInitialized;
  CoordinateType KK[3][3];

  int startFrameNo;
  int currentFrameNo;

  int m_nWidth;
  int m_nHeight;


  CoordinateType camPose[6];

  unsigned char* pCurrentColorImageRGB;
  InternalColorImageType colorImage; // 0-1, RGB order
  InternalIntensityImageType colorImageSplit[3];

  bool useVisibilityMask;

  int m_nNumMeshLevels;
  PangaeaMeshPyramid currentMeshPyramid;
  PangaeaMeshPyramid propMeshPyramid;
  vector<vector<bool> > visibilityMaskPyramid;
  vector<TrackerOutputInfo> outputInfoPyramid;
  vector<TrackerOutputInfo> outputPropPyramid;

  // ground truth
  PangaeaMeshPyramid currentMeshPyramidGT;

};

class MeshBufferReader: public TrackingEngine
{
public:

  MeshBufferReader(MeshLoadingSettings& settings, int width, int height, double K[3][3],
                   int startFrame, int numTrackingFrames);
  virtual ~MeshBufferReader();

  bool setCurrentFrame(int curFrame);
  void setIntrinsicMatrix(double K[3][3]);

  void setInitialMesh(PangaeaMeshData& mesh){};
  void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid){};
  bool trackFrame(int nFrame, unsigned char* pColorImageRGB,
                  TrackerOutputInfo** pOutputInfo);
  void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                            int nRenderLevel, bool renderType = false);

  void setMeshPyramid();

private:

  bool trackerInitialized;
  CoordinateType KK[3][3];

  int startFrameNo;
  int currentFrameNo;

  int m_nWidth;
  int m_nHeight;

  CoordinateType camPose[6];

  unsigned char* pCurrentColorImageRGB;
  InternalColorImageType colorImage; // 0-1, RGB order
  InternalIntensityImageType colorImageSplit[3];

  bool useVisibilityMask;

  int m_nNumMeshLevels;
  PangaeaMeshPyramid currentMeshPyramid;
  PangaeaMeshPyramid propMeshPyramid;
  vector<vector<bool> > visibilityMaskPyramid;
  vector<TrackerOutputInfo> outputInfoPyramid;
  vector<TrackerOutputInfo> outputPropPyramid;

  //buffer
  //only mesh buffer first
  vector<PangaeaMeshPyramid> currentMeshPyramidBuffer;
  vector<PangaeaMeshPyramid> propMeshPyramidBuffer;
  vector<vector<TrackerOutputInfo> > outputInfoPyramidBuffer;
  vector<vector<TrackerOutputInfo> > outputPropPyramidBuffer;

  int nRenderingLevel;
  int nFrameStep;

  int m_nGoodFrames;
};

#endif
