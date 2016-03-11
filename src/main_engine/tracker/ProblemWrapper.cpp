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

int ProblemWrapper::getLevelsNum()
{
  return numLevels;
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

void ProblemWrapper::addTemporalTerm(int nLevel, ceres::ResidualBlockId& residualBlockId)
{
  temporalTermResidualBlocks[nLevel].push_back(residualBlockId);
}

// clear terms
void ProblemWrapper::clearDataTerm(int nLevel)
{

  // also need to remove correponding residualBlocks from problem
  for(int i = 0; i < dataTermResidualBlocks[nLevel].size(); ++i)
    problems[nLevel]->RemoveResidualBlock(dataTermResidualBlocks[nLevel][ i ]);

  dataTermResidualBlocks[nLevel].clear();
}

void ProblemWrapper::clearFeatureTerm(int nLevel)
{
  // also need to remove correponding residualBlocks from problem
  for(int i = 0; i < featureTermResidualBlocks[nLevel].size(); ++i)
    problems[nLevel]->RemoveResidualBlock(featureTermResidualBlocks[nLevel][ i ]);

  featureTermResidualBlocks[nLevel].clear();
}

// get terms number
int ProblemWrapper::getDataTermNum(int nLevel)
{
  return dataTermResidualBlocks[nLevel].size();
}

int ProblemWrapper::getFeatureTermNum(int nLevel)
{
  return featureTermResidualBlocks[nLevel].size();
}
// get energy
void ProblemWrapper::getTotalEnergy(int nLevel, double* cost)
{
  problems[nLevel]->Evaluate(ceres::Problem::EvaluateOptions(),
                        cost, NULL, NULL, NULL);
}

void ProblemWrapper::getDataTermCost(int nLevel, double* cost)
{

  if(dataTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(dataTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      dataTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}

void ProblemWrapper::getFeatureTermCost(int nLevel, double* cost)
{
  if(featureTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(featureTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      featureTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}

void ProblemWrapper::getTVTermCost(int nLevel, double* cost)
{
  if(tvTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
  {
    ceres::Problem::EvaluateOptions evaluateOptions;

    evaluateOptions.residual_blocks = std::move(tvTermResidualBlocks[nLevel]);

    problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

    tvTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
  }

}

void ProblemWrapper::getRotTVTermCost(int nLevel, double* cost)
{
  if(rotTVTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(rotTVTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      rotTVTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}

void ProblemWrapper::getARAPTermCost(int nLevel, double* cost)
{
  if(arapTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(arapTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      arapTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}

void ProblemWrapper::getINEXTENTTermCost(int nLevel, double* cost)
{

  if(inextentTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(inextentTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      inextentTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}


void ProblemWrapper::getDeformTermCost(int nLevel, double* cost)
{

  if(deformTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(deformTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      deformTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}

void ProblemWrapper::getTemporalTermCost(int nLevel, double* cost)
{

  if(temporalTermResidualBlocks[nLevel].empty())
    cost[0] = 0;
  else
    {
      ceres::Problem::EvaluateOptions evaluateOptions;

      evaluateOptions.residual_blocks = std::move(temporalTermResidualBlocks[nLevel]);

      problems[nLevel]->Evaluate(evaluateOptions, cost, NULL, NULL, NULL);

      temporalTermResidualBlocks[nLevel] = std::move(evaluateOptions.residual_blocks);
    }

}

void ProblemWrapper::getAllCost(int nLevel, double cost[7], double* total_cost, double* sum_cost)
{
  sum_cost[0] = 0;
  total_cost[0] = 0;

  getDataTermCost(nLevel, &cost[0]);
  getFeatureTermCost(nLevel, &cost[1]);
  getTVTermCost(nLevel, &cost[2]);
  getRotTVTermCost(nLevel, &cost[3]);

  getARAPTermCost(nLevel, &cost[4]);
  getINEXTENTTermCost(nLevel, &cost[5]);
  getDeformTermCost(nLevel, &cost[6]);
  getTemporalTermCost(nLevel, &cost[7]);

  for(int i = 0; i < 8; ++i)
    sum_cost[0] += cost[i];

  getTotalEnergy(nLevel, total_cost);

}
