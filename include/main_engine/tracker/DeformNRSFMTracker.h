#ifndef _DEFORMNRSFM_TRACKER_H
#define _DEFORMNRSFM_TRACKER_H

#include "./TrackingEngine.h"
#include "./ImagePyramid.h"
#include "./FeaturePyramid.h"
#include "./OptimizationStrategy.h"
#include "./residual.h"
#include "./ProblemWrapper.h"

#include "ceres/ceres.h"

// baType mapBA(std::string const& inString);

// dataTermErrorType mapErrorType(std::string const& inString);

// // Linear Solver mapping
// ceres::LinearSolverType mapLinearSolver(std::string const& inString);

class DeformNRSFMTracker : public TrackingEngine
{

public:

  DeformNRSFMTracker(TrackerSettings& settings, int width, int height, double K[3][3],
                     int startFrame, int numTrackingFrames);

  virtual ~DeformNRSFMTracker();

  bool setCurrentFrame(int curFrame);  // only useful when loading stuff
  void setIntrinsicMatrix(double K[3][3]); // set camera parameters
  void initializeCamera();

  void setInitialMesh(PangaeaMeshData& mesh){};
  void setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid);

  //void trackerInitSetup(TrackerOutputInfo& outputInfo);
  bool trackFrame(int nFrame, unsigned char* pColorImageRGB,
                  TrackerOutputInfo** pOutputInfo);
  void updateRenderingLevel(TrackerOutputInfo** pOutputInfo,
                            int nRenderLevel, bool renderType = false);

  void AddPhotometricCost(ceres::Problem& problem,
                          ceres::LossFunction* loss_function,
                          dataTermErrorType errorType);

  void AddPhotometricCostNew(ceres::Problem& problem,
                          ceres::LossFunction* loss_function,
                          dataTermErrorType errorType);

  void AddFeatureCost(ceres::Problem& problem,
                      ceres::LossFunction* loss_function);

  void AddTotalVariationCost(ceres::Problem& problem,
                             ceres::LossFunction* loss_function);
  void AddRotTotalVariationCost(ceres::Problem& problem,
                                ceres::LossFunction* loss_function);

  void AddARAPCost(ceres::Problem& problem,
                   ceres::LossFunction* loss_function);
  void AddInextentCost(ceres::Problem& problem,
                       ceres::LossFunction* loss_function);
  void AddDeformationCost(ceres::Problem& problem,
                          ceres::LossFunction* loss_function);
  void AddTemporalMotionCost(ceres::Problem& problem,
                             double rotWeight, double transWeight);

  void AddVariableMask(ceres::Problem& problem, baType BA);
  void AddConstantMask(ceres::Problem& problem, baType BA);

  //
  void EnergySetup(ceres::Problem& problem);
  void EnergyMinimization(ceres::Problem& problem);
  void RegTermsSetup(ceres::Problem& problem, WeightPara& weightParaLevel);

  //
  bool SaveData();
  bool SaveMeshToFile(TrackerOutputInfo& outputInfo);
  bool SaveMeshPyramid();
  void SaveThread(TrackerOutputInfo** pOutputInfoRendering);

  void UpdateResults();
  void UpdateResultsLevel(int level);
  void PropagateMesh();
  void PropagateMeshCoarseToFine(int coarse_level, int fine_level);

  // attach features to the meshes
  void AttachFeaturesToMeshPyramid();
  void AttachFeatureToMesh(PangaeaMeshData* pMesh,
                           FeatureLevel* pFeatureLevel,
                           CameraInfo* pCamera,
                           vector<bool>& visibilityMask,
                           PangaeaMeshData* pOutputMesh);

  // helper functions for data term
  void AddCostImageProjection(ceres::Problem& problem,
                              ceres::LossFunction* loss_function,
                              dataTermErrorType errorType,
                              PangaeaMeshData& templateMesh,
                              MeshDeformation& meshTrans,
                              vector<bool>& visibilityMask,
                              CameraInfo* pCamera,
                              Level* pFrame);

  void AddCostImageProjectionPatch(ceres::Problem& problem,
                                   ceres::LossFunction* loss_function,
                                   dataTermErrorType errorType,
                                   PangaeaMeshData& templateMesh,
                                   MeshDeformation& meshTrans,
                                   vector<bool>& visibilityMask,
                                   MeshNeighbors& patchNeighbors,
                                   MeshNeighbors& patchRadii,
                                   MeshWeights& patchWeights,
                                   CameraInfo* pCamera,
                                   Level* pFrame);

  void AddCostImageProjectionCoarse(ceres::Problem& problem,
                                    ceres::LossFunction* loss_function,
                                    dataTermErrorType errorType,
                                    PangaeaMeshData& templateMesh,
                                    vector<bool>& visibilityMask,
                                    PangaeaMeshData& templateNeighborMesh,
                                    MeshDeformation& neighborMeshTrans,
                                    MeshDeformation& neighborMeshRot,
                                    MeshNeighbors& neighbors,
                                    MeshWeights& weights,
                                    CameraInfo* pCamera,
                                    Level* pFrame);

  void AddCostImageProjectionPatchCoarse(ceres::Problem& problem,
                                         ceres::LossFunction* loss_function,
                                         dataTermErrorType& errorType,
                                         PangaeaMeshData& templateMesh,
                                         vector<bool>& visibilityMask,
                                         MeshNeighbors& patchNeighbors,
                                         MeshNeighbors& patchRadii,
                                         MeshWeights& patchWeights,
                                         PangaeaMeshData& templateNeighborMesh,
                                         MeshDeformation& neighborMeshTrans,
                                         MeshDeformation& neighborMeshRot,
                                         MeshNeighbors& neighbors,
                                         MeshWeights& weights,
                                         CameraInfo* pCamera,
                                         Level* pFrame);

private:

  // Siggraph14 or DynamicFusion
  OptimizationStrategy* pStrategy;
  int currLevel;  // current optimization level

  int startFrameNo;
  int currentFrameNo;

  int m_nWidth;
  int m_nHeight;
  int m_nMeshLevels;

  baType BAType;
  dataTermErrorType PEType;

  double prevCamPose[6];
  double camPose[6];
  double KK[3][3];

  //
  CameraInfo camInfo;
  bool trackerInitialized;

  PangaeaMeshPyramid templateMeshPyramid;

  // containes the neighbor & weight information between pairs of meshes
  MeshPropagation meshPropagation;

  // ImagePyramid
  ImagePyramid* pImagePyramid;

  bool dataInBuffer;

  // FeaturePyramid
  FeaturePyramid* pFeaturePyramid;

  // visibilityMask Pyramid
  vector<vector<bool> > visibilityMaskPyramid;

  // tracker output Pyramid
  vector< TrackerOutputInfo > outputInfoPyramid;
  // keep the mesh just after propagation
  // should think a way to save memory
  vector< TrackerOutputInfo > outputPropPyramid;

  // transformation Pyramid
  vector< MeshDeformation > meshTransPyramid;
  vector< MeshDeformation > meshRotPyramid;

  vector< MeshDeformation > prevMeshTransPyramid;
  vector< MeshDeformation > prevMeshRotPyramid;

  // what about if we use dual quarternion representation?
  // In that case, we will need a dual quarternion
  // field transformation

  // ceres output
  std::ofstream ceresOutput;

  boost::thread* preProcessingThread;
  boost::thread* savingThread;

  boost::thread* featureThread;

  // attach features to the mesh
  boost::thread* attachFeatureThread;

  //problem wrapper
  ProblemWrapper problemWrapper;
  bool useProblemWrapper;  // for debug

  vector<ceres::ResidualBlockId> dataTermResidualBlocks;
  vector<ceres::ResidualBlockId> featureTermResidualBlocks;

};

#endif
