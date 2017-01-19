#pragma once

#include "./Mesh.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

// ICP known correspondences residual
class ResidualKnownICP
{
public:

  ResidualKnownICP(double weight, double* pTemplate, double* pTarget):
    weight(weight),pTemplate(pTemplate),pTarget(pTarget){}

  template<typename T>
  bool operator()(const T* const pRot,
                  const T* const pTrans,
                  T* residuals) const
  {
    T pRotTemplate[3];

    T pTemp[3];
    for(int i = 0;  i < 3; ++i)
      pTemp[i] = T(pTemplate[i]);

    ceres::AngleAxisRotatePoint(pRot, pTemp, pRotTemplate);

    for(int i = 0; i < 3; ++i)
      residuals[i] = T(weight)*(pRotTemplate[i] + pTrans[i]- T(pTarget[i]));

    return true;

  }

private:

  double weight;
  double* pTemplate;
  double* pTarget;

};

void KnownCorrespondencesICP(PangaeaMeshData& templateMesh, PangaeaMeshData& currentMesh, double pose[6]);
