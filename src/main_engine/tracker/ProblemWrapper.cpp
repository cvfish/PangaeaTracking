#include "main_engine/tracker/ProblemWrapper.h"

ProblemWrapper::ProblemWrapper()
{
  numLevels = 0;
}

ProblemWrapper::ProblemWrapper(int num)
{
  Initialize(num);
}

ProblemWrapper::~ProblemWrapper()
{
  for(int i = 0; i < numLevels; ++i)
    delete problems[i];
}

void ProblemWrapper::Initialize(int num)
{
  numLevels = num;

  problems.resize(numLevels);
  setupFlag.resize(numLevels, false);

  for(int i = 0; i < numLevels; ++i)
    problems[i] = new ceres::Problem;

  dataTermResidualBlocks.resize(num);
  featureTermResidualBlocks.resize(num);
  tvTermResidualBlocks.resize(num);
  rotTVTermResidualBlocks.resize(num);
  arapTermResidualBlocks.resize(num);
  inextentTermResidualBlocks.resize(num);
  deformTermResidualBlocks.resize(num);
  temporalTermResidualBlocks.resize(num);
}

ceres::Problem& ProblemWrapper::getProblem(int nLevel)
{
  return *(problems[nLevel]);
}

bool ProblemWrapper::getLevelFlag(int nLevel)
{
  return setupFlag[nLevel];
}

void ProblemWrapper::setLevelFlag(int nLevel)
{
  setupFlag[nLevel] = true;
}

// add terms

void ProblemWrapper::addDataTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  dataTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addFeatureTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  featureTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addTVTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  tvTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addRotTVTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  rotTVTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addARAPTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  arapTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addINEXTENTTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  inextentTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addDeformTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  deformTermResidualBlocks[nLevel].push_back(residualBlockId);
}

void ProblemWrapper::addTermporalTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  temporalTermResidualBlocks[nLevel].push_back(residualBlockId);
}

// clear terms
void ProblemWrapper::clearDataTerm(int nLevel)
{
  dataTermResidualBlocks[nLevel].clear();
}

void ProblemWrapper::clearFeatureTerm(int nLevel)
{
  featureTermResidualBlocks[nLevel].clear();
}


// get energy
void ProblemWrapper::getTotalEnergy(int nLevel, double* cost)
{
  problems[nLevel]->Evaluate(ceres::Problem::EvaluateOptions(),
                        cost, NULL, NULL, NULL);
}


void ProblemWrapper::getDataTermCost(int nLevel, double* cost)
{

  ceres::Problem::EvaluateOptions evaluateOptions;

  evaluateOptions.residual_blocks = std::move(dataTermResidualBlocks[nLevel]);

  problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

  dataTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);

}
