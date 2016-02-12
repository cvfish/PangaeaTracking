#pragma once

#include "../utils/global.h"

template<typename FloatType>
class MeshData
{

public:

  MeshData(){};

  void clear();

  void updateVertices(MeshData& d);

  void updateNormals(MeshData& d);

  void computeNormals();

  void computeNormalsNeil();

  // MeshData(const MeshData& d) = default;
  // MeshData& operator=(const MeshData& d) = default;
  // the two functions in the above doesn't work on the visual4,
  // so have to write it explicitly
  MeshData(const MeshData& d)
  {
    vertices = d.vertices;
    normals = d.normals;
    colors = d.colors;
    grays = d.grays;

    facesVerticesInd = d.facesVerticesInd;
    adjVerticesInd = d.adjVerticesInd;
    modelColors = d.modelColors;
    modelLabels = d.modelLabels;

    numVertices = d.numVertices;
    numFaces = d.numFaces;
    for(int i = 0; i < 3; ++i)
      center[i] = d.center[i];

		clockwise = d.clockwise;
    features = d.features;

  }

  MeshData& operator=(const MeshData& d)
  {
    vertices = d.vertices;
    normals = d.normals;
    colors = d.colors;
    grays = d.grays;

    facesVerticesInd = d.facesVerticesInd;
    adjVerticesInd = d.adjVerticesInd;
    modelColors = d.modelColors;
    modelLabels = d.modelLabels;

    numVertices = d.numVertices;
    numFaces = d.numFaces;
    for(int i = 0; i < 3; ++i)
      center[i] = d.center[i];

		clockwise = d.clockwise;

    features = d.features;

    return *this;
  }


  MeshData(MeshData&& d) {
		vertices = std::move(d.vertices);
    normals = std::move(d.normals);
    colors = std::move(d.colors);
    grays = std::move(d.grays);

    facesVerticesInd = std::move(d.facesVerticesInd);
    adjVerticesInd = std::move(d.adjVerticesInd);
    modelColors = std::move(d.modelColors);
    modelLabels = std::move(d.modelLabels);

    numVertices = d.numVertices;
    numFaces = d.numFaces;
    for(int i = 0; i < 3; ++i)
      center[i] = d.center[i];

		clockwise = d.clockwise;

    features = std::move(d.features);

	};
	MeshData& operator=(MeshData&& d) {
    vertices = std::move(d.vertices);
    normals = std::move(d.normals);
    colors = std::move(d.colors);
    grays = std::move(d.grays);
    facesVerticesInd = std::move(d.facesVerticesInd);
    adjVerticesInd = std::move(d.adjVerticesInd);
    modelColors = std::move(d.modelColors);
    modelLabels = std::move(d.modelLabels);

    numVertices = d.numVertices;
    numFaces = d.numFaces;
    for(int i = 0; i < 3; ++i)
      center[i] = d.center[i];

		clockwise = d.clockwise;

    features = std::move(d.features);

    return *this;
	};


  ~MeshData(){};

  vector<vector<FloatType> > vertices;
  vector<vector<FloatType> > normals;
  vector<vector<FloatType> > colors;
  vector<FloatType> grays;

  vector<vector<unsigned int> > facesVerticesInd;
  vector<vector<unsigned int> > adjVerticesInd;

  vector<vector<FloatType> > modelColors;
  vector<unsigned int> modelLabels;

  int numVertices;
  int numFaces;
  double center[3];

	bool clockwise;

  // attach features to the mesh
  vector<vector<FloatType> > features;
  vector<vector<FloatType> > featuresBuffer;
};

template<typename FloatType>
void MeshData<FloatType>::clear()
{
  vertices.clear();
  normals.clear();
  colors.clear();
  grays.clear();
  facesVerticesInd.clear();
  adjVerticesInd.clear();
  modelColors.clear();
  modelLabels.clear();

  features.clear();
}

template<typename FloatType>
void MeshData<FloatType>::updateVertices(MeshData& d)
{
  // check if the two meshes have the same number of vertices
  assert(numVertices == d.numVertices);
  for(int i = 0; i < d.numVertices; ++i)
    {
      for(int j = 0; j < 3; ++j)
        {
          vertices[i][j] = d.vertices[i][j];
        }
    }
}

template<typename FloatType>
void MeshData<FloatType>::updateNormals(MeshData& d)
{
  assert(numVertices == d.numVertices);
  for(int i = 0; i < d.numVertices; ++i)
    {
      for(int j = 0; j < 3; ++j)
        {
          normals[i][j] = d.normals[i][j];
        }
    }
}
// compute normals for mesh created form depth map input
// edges order is upper, right, lower, left,
// so we need to flip the normals in the end
template<typename FloatType>
void MeshData<FloatType>::computeNormals()
{
  // normals for each vertex
  FloatType normal[3];
  unsigned int ind1, ind2;

  for(int i = 0; i < numVertices; ++i)
    {
      int neighborsNum = adjVerticesInd[i].size();
      if(neighborsNum >= 2)
        {
          ind1 = adjVerticesInd[i][0];
          normal[0] = 0; normal[1] = 0; normal[2] = 0;
          FloatType testNorm[3], temp;
          for(int j = 0; j < neighborsNum; ++j)
            {
              // second edge
              if(j < neighborsNum - 1)
                ind2 = adjVerticesInd[i][ j+1 ];
              else
                ind2 = adjVerticesInd[i][0];

              compnorm(&vertices[i][0],
                       &vertices[ind2][0],
                       &vertices[ind1][0],
                       testNorm,
                       clockwise);

              normal[0] += testNorm[0];
              normal[1] += testNorm[1];
              normal[2] += testNorm[2];

              ind1 = ind2;
            }

          temp = 1.0/sqrt(normal[0]*normal[0]
                          + normal[1]*normal[1]
                          + normal[2]*normal[2]);

          normals[i][0] = -normal[0] * temp;
          normals[i][1] = -normal[1] * temp;
          normals[i][2] = -normal[2] * temp;
        }
    }
}

// compute the normals for Neil's mesh
template<typename FloatType>
void MeshData<FloatType>::computeNormalsNeil()
{
  // loop over all the faces
  for(int i = 0; i < numVertices; ++i)
    normals[i].resize(3,0);
  for(int i = 0; i < numFaces; ++i)
    {
      for(int j = 0; j < 3; ++j)
        {
          int m = (j+1)%3;
          int k = (j+2)%3;

          //
          FloatType normal[3];
          // compnorm computes (p1-p3) x (p1-p2)
          // or (p3-p1) x (p2-p1)
          compnorm(&vertices[ facesVerticesInd[i][j] ][0],
                   &vertices[ facesVerticesInd[i][m] ][0],
                   &vertices[ facesVerticesInd[i][k] ][0],
                   normal, clockwise);

          normals[ facesVerticesInd[i][j] ][0] += normal[0];
          normals[ facesVerticesInd[i][j] ][1] += normal[1];
          normals[ facesVerticesInd[i][j] ][2] += normal[2];

        }

    }
  // make normals have unit norm
  for(int i = 0; i < numVertices; ++i)
    {
      FloatType magn = mynorm(&normals[i][0]);
      for(int j = 0; j < 3; ++j)
        normals[i][j] = normals[i][j] / magn;
    }

}


typedef MeshData<float> MeshDataf;
typedef MeshData<double> MeshDatad;

typedef MeshData<CoordinateType> PangaeaMeshData;


// output interface
struct TrackerOutputInfo
{

  // TrackerOutputInfo();
  // ~TrackerOutputInfo();

  // PangaeaMeshData& getMeshData();
  // PangaeaMeshData& getMeshDataGT();

  // vector<vector<CoordinateType> >& getMeshProj();
  // vector<vector<CoordinateType> >& getMeshProjGT();

  // vector<bool>& getVisibilityMask();
  // vector<bool>& getVisibilityMaskGT();

  // PangaeaMeshData& getColorDiff();
  // PangaeaMeshData& getColorDiffGT();

  double camPose[6];

  PangaeaMeshData meshData;
  PangaeaMeshData meshDataGT;

  // color difference mesh
  PangaeaMeshData meshDataColorDiff;
  PangaeaMeshData meshDataColorDiffGT;

  // 2d projections
  vector<vector<CoordinateType> > meshProj;
  vector<vector<CoordinateType> > meshProjGT;

  // visibility mask
  vector<bool> visibilityMask;
  vector<bool> visibilityMaskGT;

  int nRenderLevel;

};
