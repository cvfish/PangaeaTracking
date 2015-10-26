// implementation of OptimizationStrategy file
#include "main_engine/tracker/OptimizationStrategy.h"

OptimizationStrategy::OptimizationStrategy(int numMeshLevels):
    numMeshLevels(numMeshLevels)
{}

void OptimizationStrategy::setWeightParameters(WeightPara& inputWeightPara)
{
    weightPara = inputWeightPara;
}

void OptimizationStrategy::setWeightScale(IntegerContainerType& meshVertexNum)
{

    if(numOptimizationLevels > 1)
    {
        // setting up the proper scale according to the number of vertices
        // of each level, starting from the 0th level
        weightScale.dataTermScale.resize(numOptimizationLevels, 1);
        weightScale.tvTermScale.resize(numOptimizationLevels, 1);
        weightScale.arapTermScale.resize(numOptimizationLevels, 1);
        weightScale.inextentTermScale.resize(numOptimizationLevels, 1);
        weightScale.deformTermScale.resize(numOptimizationLevels, 1);

        weightScale.rotScale.resize(numOptimizationLevels, 1);
        weightScale.transScale.resize(numOptimizationLevels, 1);

        // // all the 0th level have scale 1
        // weightScale.dataTermScale[0] = 1;
        // weightScale.tvTermScale[0] = 1;
        // weightScale.arapTermScale[0] = 1;
        // weightScale.inextentTermScale[0] = 1;
        // weightScale.deformTermScale[0] = 1;

        // weightScale.rotScale[0] = 1;
        // weightScale.transScale[0] = 1;

        for(int i = 1; i < numOptimizationLevels; ++i)
        {
            double decreaseFactor = double(meshVertexNum[i]) / meshVertexNum[0];
            weightScale.dataTermScale[i] = 1;
            weightScale.tvTermScale[i] = decreaseFactor * weightScale.tvTermScale[0];
            weightScale.arapTermScale[i] =
                pow(decreaseFactor,2) * weightScale.arapTermScale[0];
            weightScale.inextentTermScale[i] =
                pow(decreaseFactor,2) * weightScale.inextentTermScale[0];
            weightScale.deformTermScale[i] = 1;

            weightScale.rotScale[i] = decreaseFactor *  weightScale.rotScale[0];
            weightScale.transScale[i] = decreaseFactor * weightScale.transScale[0];
        }

    }

}

void OptimizationStrategy::setWeightScale(WeightScale& inputWeightScale)
{
    weightScale = inputWeightScale;
}

Siggraph14Strategy::Siggraph14Strategy(int numMeshLevels):
    OptimizationStrategy(numMeshLevels)
{
    numOptimizationLevels = numMeshLevels;
}

void Siggraph14Strategy::Initialize()
{
    optimizationSettings.resize(numOptimizationLevels);

    for(int i = 0; i < numOptimizationLevels; ++i)
    {
        std::pair<int, int> levelPairs(i,i);
        optimizationSettings[i].dataTermPairs.push_back(levelPairs);
        optimizationSettings[i].tvTermPairs.push_back(levelPairs);
        optimizationSettings[i].arapTermPairs.push_back(levelPairs);
        optimizationSettings[i].inextentTermPairs.push_back(levelPairs);

        // deformation penalty
        optimizationSettings[i].deformTermLevelIDVec.push_back(i);

        if(i > 0)
        {
            std::pair<int, int> propPairs(i,i-1);  // level i -> i-1
            optimizationSettings[i].propPairs.push_back(propPairs);
        }
    }

    // weight initialization
}

// there should be at least three levels for this to work
DynamicFusionStrategy::DynamicFusionStrategy(int numMeshLevels):
    OptimizationStrategy(numMeshLevels)
{
    // by default we think only the 0th level is used for data term,
    // based on the interpolation from 1th level, so the optimization
    // variables in the data term is the motion of the first level mesh

    // there is one optimization for DynamicFusion
    numOptimizationLevels = 1;
}

void DynamicFusionStrategy::Initialize()
{
    optimizationSettings.resize(numOptimizationLevels);

    // data term
    std::pair<int, int> levelPairs(0,1);
    optimizationSettings[0].dataTermPairs.push_back(levelPairs);

    // arap term, tv term, inextent term
    for(int i = 1; i < numMeshLevels; ++i)
    {
        std::pair<int, int> levelPairs(i,i+1);
        optimizationSettings[0].arapTermPairs.push_back(levelPairs);
        optimizationSettings[0].tvTermPairs.push_back(levelPairs);
        optimizationSettings[0].inextentTermPairs.push_back(levelPairs);
    }

    // propagation from level 1 -> level 0
    std::pair<int, int> propPairs(1,0);
    optimizationSettings[0].propPairs.push_back(propPairs);
}

// write you own optimization strategy here if you want do something new
CoarseNodeStrategy::CoarseNodeStrategy(int numMeshLevels):
    OptimizationStrategy(numMeshLevels)
{
    numOptimizationLevels = numMeshLevels-1;
}

void CoarseNodeStrategy::Initialize()
{
    optimizationSettings.resize(numOptimizationLevels);

    for(int i = 0; i < numOptimizationLevels; ++i)
    {
        std::pair<int, int> levelPairs(i,i+1);
        optimizationSettings[i].dataTermPairs.push_back(levelPairs);
        optimizationSettings[i].tvTermPairs.push_back(levelPairs);
        optimizationSettings[i].arapTermPairs.push_back(levelPairs);
        optimizationSettings[i].inextentTermPairs.push_back(levelPairs);

        // deformation penalty
        optimizationSettings[i].deformTermLevelIDVec.push_back(i);

        if(i > 0)
        {
            std::pair<int, int> propPairs(i,i-1);  // level i -> i-1
            optimizationSettings[i].propPairs.push_back(propPairs);
        }
    }
}
