//
// OptimizationStrategy file

#pragma once

#include "../utils/global.h"
#include "../utils/settings.h"

struct OptimizationLevel
{
    // data term mesh levels for each level
    // the first one gives the projection point
    // the second one gives its neighbors,
    // if these two are the same level, do not need to do any interpolation
    // otherwise we interpolate the position based on its neighbors
    vector<std::pair<int, int> > dataTermPairs;
    vector<std::pair<int, int> > regTermPairs;

    // propagation to be done after optimization
    vector<std::pair<int, int> > propPairs;

    //
    vector<int> deformTermLevelIDVec;
};

struct WeightPara
{
    // weighting parameters of each energy term
    double dataTermWeight; // photometric term
    double tvTermWeight;
    double tvRotTermWeight;

    double arapTermWeight;
    double inextentTermWeight;
    double deformWeight;

    // rigid transformation weight
    double rotWeight;
    double transWeight;

    // Huber width
    double dataHuberWidth;
    double tvHuberWidth;
    double tvRotHuberWidth;

  // feature weighting parameters
  double featureTermWeight;
  double featureHuberWidth;

};

struct WeightScale
{
    vector<double> dataTermScale;
    vector<double> tvTermScale;
    vector<double> rotTVTermScale;
    vector<double> arapTermScale;
    vector<double> inextentTermScale;
    vector<double> deformTermScale;

    // rigid transformation weighting scale
    // change over different levels of the pyramid
    vector<double> rotScale;
    vector<double> transScale;

  vector<double> featureTermScale;
};

class OptimizationStrategy
{
public:


    vector<OptimizationLevel> optimizationSettings;
    int numOptimizationLevels;
    int numMeshLevels;

    OptimizationStrategy(){};
    OptimizationStrategy(int numMeshLevels);
    ~OptimizationStrategy(){};

    virtual void Initialize(){};

    virtual void setWeightParameters(WeightPara& inputWeightPara);
    virtual void setWeightScale(IntegerContainerType& meshVertexNum);
    virtual void setWeightScale(WeightScale& inputWeightScale);
    virtual void setWeightParametersVec();

    WeightPara weightPara;
    WeightScale weightScale;

    // weightPara
    vector< WeightPara > weightParaVec;

    vector< pair<int, int> > propPairsFinal;
};

// arbitary neighbors support for all terms goes here
class FreeNeighborStrategy:public OptimizationStrategy
{
public:

    FreeNeighborStrategy(int numMeshLevels);
    ~FreeNeighborStrategy(){};

    void Initialize();

    void AddPropPairs(vector<std::pair<int, int> >& propPairs,
        vector<std::pair<int, int> >& nextPairs,
        vector<int>& updatedLevels);

};
