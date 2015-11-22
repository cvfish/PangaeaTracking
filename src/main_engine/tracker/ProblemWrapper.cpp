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