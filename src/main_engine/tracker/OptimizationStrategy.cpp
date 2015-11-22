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

        weightScale.rotTVTermScale.resize(numOptimizationLevels,1);

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

            weightScale.rotTVTermScale[i] = decreaseFactor * weightScale.rotTVTermScale[0];

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

void OptimizationStrategy::setWeightParametersVec()
{

    WeightPara weightParaLevel;
    
    for(int currLevel = 0; currLevel < numOptimizationLevels; ++currLevel)
    {
        weightParaLevel.dataTermWeight = weightPara.dataTermWeight * weightScale.dataTermScale[currLevel];
        weightParaLevel.tvTermWeight =
            weightPara.tvTermWeight * weightScale.tvTermScale[currLevel];
        weightParaLevel.tvRotTermWeight = weightPara.tvRotTermWeight *
            weightScale.tvTermScale[currLevel];
        weightParaLevel.arapTermWeight = weightPara.arapTermWeight *
            weightScale.arapTermScale[currLevel];
        weightParaLevel.inextentTermWeight = weightPara.inextentTermWeight *
            weightScale.inextentTermScale[currLevel];
        weightParaLevel.deformWeight = weightPara.deformWeight *
            weightScale.deformTermScale[currLevel];

        // always the same dataHuberWidth and tvHuberWidth
        weightParaLevel.dataHuberWidth = weightPara.dataHuberWidth;
        weightParaLevel.tvHuberWidth = weightPara.tvHuberWidth;
        weightParaLevel.tvRotHuberWidth = weightPara.tvRotHuberWidth;

        // rotWeight and transWeight
        weightParaLevel.rotWeight = weightPara.rotWeight *
            weightScale.rotScale[currLevel];
        weightParaLevel.transWeight = weightPara.transWeight *
            weightScale.transScale[currLevel];
        
        weightParaVec.push_back( weightParaLevel );
    }
    
}

// free neighbor
FreeNeighborStrategy::FreeNeighborStrategy(int numMeshLevels):
    OptimizationStrategy(numMeshLevels)
{
}

void FreeNeighborStrategy::Initialize()
{
    // get number of optimization levels
    if(trackerSettings.dataTermPair.size() > 0)
    {
        numOptimizationLevels = min<int>(numMeshLevels, trackerSettings.dataTermPair.size());
        optimizationSettings.resize( numOptimizationLevels );
        for(int i = 0; i < numOptimizationLevels; ++i)
        optimizationSettings[i].dataTermPairs = trackerSettings.dataTermPair[i];
    }
    else
    {
        // siggraph14 strategy
        numOptimizationLevels = numMeshLevels;
        optimizationSettings.resize( numOptimizationLevels );
        for(int i = 0; i < numOptimizationLevels; ++i)
        {
            std::pair<int, int> levelPairs(i,i);
            optimizationSettings[i].dataTermPairs.push_back(levelPairs);
        }
    }


    for(int i = 0; i < numOptimizationLevels; ++i)
    {
        vector<std::pair<int, int> >& dataTermPairs = optimizationSettings[i].dataTermPairs;

        // tv term
        if(trackerSettings.regTermPair.size() > i)
        optimizationSettings[i].regTermPairs = trackerSettings.regTermPair[i];
        else
        optimizationSettings[i].regTermPairs = dataTermPairs;
    }
        
    for(int i = 0; i < numOptimizationLevels; ++i)
    {
        // // what should we propagate, we should do the pairs whose first item
        // // have been updated, while second not. In other words,

        vector<int> updatedLevels;
        vector<int> nextOptimizationLevels;

        vector<std::pair<int, int> >& dataTermPairs = optimizationSettings[i].dataTermPairs;
        vector<std::pair<int, int> >& regTermPairs = optimizationSettings[i].regTermPairs;

        for(int j = 0; j < dataTermPairs.size(); ++j)
        updatedLevels.push_back( dataTermPairs[j].second );

        for(int j = 0; j < regTermPairs.size(); ++j)
        {
            updatedLevels.push_back( regTermPairs[j].first );
            updatedLevels.push_back( regTermPairs[j].second );
        }

        // remove duplicate items
        sort( updatedLevels.begin(), updatedLevels.end() );  // from small to large
        updatedLevels.erase( unique( updatedLevels.begin(), updatedLevels.end() ), updatedLevels.end() );

        //
        optimizationSettings[ i ].deformTermLevelIDVec = updatedLevels;

        int nextLevel = i ? i - 1 : numOptimizationLevels - 1;

        vector<std::pair<int, int> >& nextDataTermPairs = optimizationSettings[ nextLevel ].dataTermPairs;
        vector<std::pair<int, int> >& nextRegTermPairs  = optimizationSettings[ nextLevel ].regTermPairs;

        for(int j = 0; j < dataTermPairs.size(); ++j)
        nextOptimizationLevels.push_back( nextDataTermPairs[j].second );

        for(int j = 0; j < regTermPairs.size(); ++j)
        {
            nextOptimizationLevels.push_back( nextRegTermPairs[j].first );
            nextOptimizationLevels.push_back( nextRegTermPairs[j].second );
        }
        
        // remove duplicate items
        sort( nextOptimizationLevels.begin(), nextOptimizationLevels.end() );
        nextOptimizationLevels.erase( unique( nextOptimizationLevels.begin(), nextOptimizationLevels.end() ),
            nextOptimizationLevels.end() );

        // AddPropPairs( optimizationSettings[ i ].propPairs, nextDataTermPairs, updatedLevels );
        // AddPropPairs( optimizationSettings[ i ].propPairs, nextRegTermPairs, updatedLevels );

        // add up all the nextOptimizationLevels which is not in updatedLevels and also below updatedLevels
        for(int j = 0; j < nextOptimizationLevels.size(); ++j)
        {
            std::vector<int>::iterator first_position = std::find( updatedLevels.begin(), updatedLevels.end(), nextOptimizationLevels[j] );
            if( first_position == updatedLevels.end() )
            {
                std::vector<int>::iterator up;
                up = std::upper_bound( updatedLevels.begin(), updatedLevels.end(), nextOptimizationLevels[j] );
                if(up == updatedLevels.end())  // all elements are bigger
                up = updatedLevels.begin();

                for(int k = *up; k > nextOptimizationLevels[j]; --k)
                optimizationSettings[ i ].propPairs.push_back( pair<int, int>(k, k-1) );
                
            }
        }
        
        if(i == 0)
        {
            for(int j = updatedLevels[0]; j > 0 ; --j)
            propPairsFinal.push_back( pair<int, int>(j, j-1) );
        }
        
    }
    
}

void FreeNeighborStrategy::AddPropPairs(vector<std::pair<int, int> >& propPairs,
    vector<std::pair<int, int> >& nextPairs, vector<int>& updatedLevels)
{
    for(int j = 0; j < nextPairs.size();  ++j)
    {
        std::vector<int>::iterator first_position = std::find(
            updatedLevels.begin(), updatedLevels.end(), nextPairs[ j ].first );

        std::vector<int>::iterator second_position = std::find(
            updatedLevels.begin(), updatedLevels.end(), nextPairs[ j ].second );
        
        vector<pair<int, int> >::iterator position = std::find(
            propPairs.begin(), propPairs.end(), nextPairs[ j ]);

        if( nextPairs[ j ].first != nextPairs[ j ].second &&
            first_position == updatedLevels.end() &&
            second_position != updatedLevels.end() &&
            position == propPairs.end() )
        propPairs.push_back( nextPairs[j] );
    }
}