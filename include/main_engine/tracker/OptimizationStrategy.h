// 
// OptimizationStrategy file

#pragma once

#include "../utils/global.h"

// should use proper factory to avoid using switching case in Tracker
enum OptimizationType{
    Siggraph14 = 0,
    DynamicFusion,
};

struct OptimizationLevel
{
    // data term mesh levels for each level
    // the first one gives the projection point
    // the second one gives its neighbors,
    // if these two are the same level, do not need to do any interpolation
    // otherwise we interpolate the position based on its neighbors
    vector<std::pair<int, int> > dataTermPairs;
    vector<std::pair<int, int> > tvTermPairs;
    vector<std::pair<int, int> > arapTermPairs;
    vector<std::pair<int, int> > inextentTermPairs;

    vector<int> deformTermLevelIDVec;

    // propagation to be done after optimization
    vector<std::pair<int, int> > propPairs;
};

struct WeightPara
{    
    // weighting parameters of each energy term 
    double dataTermWeight; // photometric term
    double tvTermWeight;
    double arapTermWeight;
    double inextentTermWeight;
    double deformWeight;

    // rigid transformation weight
    double rotWeight;
    double transWeight;

    // Huber width
    double dataHuberWidth;
    double tvHuberWidth;
};

struct WeightScale
{
    vector<double> dataTermScale;
    vector<double> tvTermScale;
    vector<double> arapTermScale;
    vector<double> inextentTermScale;
    vector<double> deformTermScale;

    // rigid transformation weighting scale
    // change over different levels of the pyramid
    vector<double> rotScale;
    vector<double> transScale;
};

class OptimizationStrategy
{
public:
    vector<OptimizationLevel> optimizationSettings;
    int numOptimizationLevels;

    virtual void setWeightParameters(WeightPara& inputWeightPara);
    virtual void setWeightScale(IntegerContainerType& meshVertexNum);
    virtual void setWeightScale(WeightScale& inputWeightScale);
    
    WeightPara weightPara;
    WeightScale weightScale;
};

class Siggraph14Strategy:public OptimizationStrategy
{
public:
    
    Siggraph14Strategy(int numMeshLevels);
    ~Siggraph14Strategy(){};

    void setWeightScale(IntegerContainerType& meshVertexNum);
    
public: 
       
};

class DynamicDusionStrategy:public OptimizationStrategy
{
public:
    DynamicDusionStrategy(int numMeshLevels);
    ~DynamicDusionStrategy(){};

    // never worry about scale weights
    // for the only level optimization,
    // we use the weights with no scale
};