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
  void addTermporalTerm(int nLevel, ceres::ResidualBlockId& residualBlockId);

  void clearDataTerm(int nLevel);
  void clearFeatureTerm(int nLevel);

  void getTotalEnergy(int nLevel, double* cost);
  void getDataTermCost(int nLevel, double* cost);

private:

  vector<ceres::Problem* > problems;
  vector<bool> setupFlag;

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
