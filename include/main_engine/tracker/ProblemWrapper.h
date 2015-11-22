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
    
private:
    
    vector<ceres::Problem* > problems;
    vector<bool> setupFlag;
    
    // vector<bool> arapFlag;
    // vector<bool> tvFlag;
    // vector<bool> rotTVFlag;
    // vector<bool> inextentFlag;
    // vector<bool> deformFlag;
    // vector<bool> tempFlag;

    int numLevels;

};