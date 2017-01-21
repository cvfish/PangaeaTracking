#include "main_engine/tracker/residual_util.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

void KnownCorrespondencesICP(PangaeaMeshData& templateMesh,
                             PangaeaMeshData& currentMesh,
                             double pose[6])
{

  int numVertices = templateMesh.numVertices;

  ceres::Problem problemICP;

  // ceresOutput << "initial pose " << endl;
  // for(int i = 0; i < 6; ++i)
  //   ceresOutput << pose[i] << " ";
  // ceresOutput << endl;

  for(int i = 0; i < numVertices; ++i)
    {
      ResidualKnownICP* pResidual = new ResidualKnownICP(1,
                                                         &templateMesh.vertices[i][0],
                                                         &currentMesh.vertices[i][0]);

      ceres::AutoDiffCostFunction<ResidualKnownICP, 3, 3, 3>* cost_function =
        new ceres::AutoDiffCostFunction<ResidualKnownICP, 3, 3, 3>(pResidual);

      problemICP.AddResidualBlock(
                                  cost_function,
                                  NULL,
                                  &pose[0],
                                  &pose[3]);

      // ceresOutput << " point " << i << " difference " <<
      //   templateMesh.vertices[i][0] - currentMesh.vertices[i][0] << " " <<
      //   templateMesh.vertices[i][1] - currentMesh.vertices[i][1] << " " <<
      //   templateMesh.vertices[i][2] - currentMesh.vertices[i][2] << endl;

    }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  options.num_threads = 2;
  options.max_num_iterations = 10;

  ceres::Solve(options, &problemICP, &summary);

}
