#include "../utils/global.h"
#include "ceres/ceres.h"

class ProblemWrapper
{
public:
  ProblemWrapper();
  ProblemWrapper(int numLevels);
  ~ProblemWrapper();

  void Initialize(int numLevels);
  ceres::Problem& getProblem(int nLevel);
  bool getLevelFlag(int nLevel);
  void setLevelFlag(int nLevel);

  void addDataTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addFeatureTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addTVTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addRotTVTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addARAPTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addINEXTENTTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addDeformTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);
  void addTemporalTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);

  void clearDataTerm(int nLevel);
  void clearFeatureTerm(int nLevel);
  int getDataTermNum(int nLevel);

  void getTotalEnergy(int nLevel, double* cost);
  void getDataTermCost(int nLevel, double* cost);
  void getFeatureTermCost(int nLevel, double* cost);
  void getTVTermCost(int nLevel, double* cost);
  void getRotTVTermCost(int nLevel, double* cost);
  void getARAPTermCost(int nLevel, double* cost);
  void getINEXTENTTermCost(int nLevel, double* cost);
  void getDeformTermCost(int nLevel, double* cost);
  void getTemporalTermCost(int nLevel, double* cost);

  void getAllCost(int nLevel, double cost[7], double* total_cost, double* sum_cost);

  // double* getRigidTransformation();
  // PangaeaMeshData& getMeshData(int nLevel);
  // MeshDeformation& getMeshRotation(int nLevel);
  // MeshDeformation& getMeshTranslation(int nLevel);
  // MeshDeformation& getPrevMeshRotation(int nLevel);
  // MeshDeformation& getPrevMeshTranslation(int nLevel);

  // void setRigidTransformation(double* pRigidTransform);
  // void setMeshData(PangaeaMeshPyramid* pMeshPyr);
  // void setMeshRotation(vector<MeshDeformation>* pMeshRotPyr);
  // void setMeshTranslation(vector<MeshDeformation>* pMeshTransPyr);

  // void setOptimizationVariables(double* pRigidTransform,
  //                               PangaeaMeshPyramid* pMeshPyr,
  //                               vector<MeshDeformation>* pMeshRotPyr,
  //                               vector<MeshDeformation>* pMeshTransPyr,
  //                               vector<MeshDeformation>* pPrevMeshRotPyr,
  //                               vector<MeshDeformation>* pPrevMeshTransPyr);

private:

  vector<bool> setupFlag;
  vector<ceres::Problem* > problems;

  // keep the pointer to all possible optimization variables
  // double* pRigidTransform_;
  // PangaeaMeshPyramid* pMeshPyr_;
  // vector<MeshDeformation>* pMeshRotPyr_;
  // vector<MeshDeformation>* pMeshTransPyr_;
  // vector<MeshDeformation>* pPrevMeshRotPyr_;
  // vector<MeshDeformation>* pPrevMeshTransPyr_;


  vector<vector<ceres::ResidualBlockId> > dataTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > featureTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > tvTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > rotTVTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > arapTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > inextentTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > deformTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > temporalTermResidualBlocks;

  int numLevels;

};
