#include "../utils/global.h"
#include "ceres/ceres.h"

class ProblemWrapper
{
public:
  ProblemWrapper();
  ProblemWrapper(int numLevels);
  ~ProblemWrapper();

  void Initialize(int numLevels);
  int getLevelsNum();

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

  void addDataTermCost(int nLevel, ceres::CostFunction* pCostFunction);
  void addFeatureTermCost(int nLevel, ceres::CostFunction* pCostFunction);
  void addRegTermCost(int nLevel, ceres::CostFunction* pCostFunction);

  void addDataTermLoss(int nLevel, ceres::LossFunction* pLossFunction);
  void addFeatureTermLoss(int nLevel, ceres::LossFunction* pLossFunction);
  void addRegTermLoss(int nLevel, ceres::LossFunction* pLossFunction);

  void clearDataTerm(int nLevel);
  void clearFeatureTerm(int nLevel);
  int getDataTermNum(int nLevel);
  int getFeatureTermNum(int nLevel);

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

  void clearDataTermCost(int nLevel);
  void clearFeatureTermCost(int nLevel);
  void clearCostFunctions(int nLevel);
  void clearCostFunctionsHelper(vector<ceres::CostFunction*>& costFunctions);

  void clearDataTermLoss(int nLevel);
  void clearFeatureTermLoss(int nLevel);
  void clearLossFunctions(int nLevel);
  void clearLossFunctionsHelper(vector<ceres::LossFunction*>& lossFunctions);

private:

  vector<bool> setupFlag;
  vector<ceres::Problem* > problems;

  vector<vector<ceres::ResidualBlockId> > dataTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > featureTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > tvTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > rotTVTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > arapTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > inextentTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > deformTermResidualBlocks;
  vector<vector<ceres::ResidualBlockId> > temporalTermResidualBlocks;


  vector<vector<ceres::CostFunction*> > dataTermCostFunctions;
  vector<vector<ceres::CostFunction*> > featureTermCostFunctions;
  vector<vector<ceres::CostFunction*> > regTermCostFunctions;

  vector<vector<ceres::LossFunction*> > dataTermLossFunctions;
  vector<vector<ceres::LossFunction*> > featureTermLossFunctions;
  vector<vector<ceres::LossFunction*> > regTermLossFunctions;

  int numLevels;
};
