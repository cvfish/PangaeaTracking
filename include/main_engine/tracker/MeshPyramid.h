#pragma once

#include "./MeshData.h"
#include "./MeshIO.h"

#include "third_party/KDTreeAdaptor.hpp"

typedef vector<vector<size_t> > MeshNeighbors;

template<class FloatType>
struct MeshPropagation
{
    typedef vector<vector<FloatType> > MeshWeights;
    typedef vector<vector<FloatType> > MeshDistances;
    typedef vector<FloatType> MeshSigmas;
    typedef vector<vector<FloatType> > MeshDeform;

    bool useRadius;
    IntegerContainerType KNN;
    CoordinateContainerType radius;
    vector<MeshNeighbors> neighborsPyramid;
    vector<MeshWeights> weightsPyramid;
    vector<MeshDistances> distancesPyramid;
    vector<MeshSigmas> sigmasPyramid;

    vector<vector<vector<unsigned int> > > neighborsPyramidUINT;
};

// mesh pyramid
template<class FloatType>
class MeshPyramid
{

public:

    typedef MeshData<FloatType> Mesh;

    MeshPyramid(){};
    MeshPyramid(Mesh& mesh);
    MeshPyramid(string meshLevelFormat,
        IntegerContainerType& meshVertexNum);
    MeshPyramid(string meshPath, string meshLevelFormat, int frame,
        IntegerContainerType& meshLevelList);

    MeshPyramid& operator=(const MeshPyramid& d) = default;
    MeshPyramid& operator=(MeshPyramid&& d)
    {
        levels = std::move(d.levels);
        numLevels = d.numLevels;
        meshPyramidVertexNum = std::move(d.meshPyramidVertexNum);
    }

    void updatePyramid(string meshPath, string meshLevelFormat, int frame,
        IntegerContainerType& meshLevelList);

    // pyramid data, the finest level is level 0
    // and level number increases as we go up the pyramid
    int numLevels;
    vector<Mesh > levels;
    IntegerContainerType meshPyramidVertexNum;

    // put this outside levels as we would need multiple copies of meshes,
    // but only need one copy of neighbors and weights
    // // these correspond to the weights and neighbors
    // // from fine mesh to coarse mesh, for each vertex
    // // in the fine mesh, we search for its neighbors
    // // in the coarse mesh and calculate corresponding weights.
    // // vector<int> kNN; //number of nearest neighbors
    // // vector<FloatType> radius; //the radius range for searching neighbors

    // IntegerContainerType kNN;
    // CoordinateContainerType radius;

    // vector<MeshNeighbors> neighborsPyramid;
    // vector<MeshWeights> weightsPyramid;
    // vector<MeshDistances> distancesPyramid;
    // vector<MeshSigmas> sigmasPyramid;
    // bool useRadius;

    // should we keep the neighbors from coarse level to fine level,
    // not at the moment

    // member methods
    // setup the neighbors and weights for all the levels,
    // we will have n-1 levels of neighbors and weights for n level pyramid
    // void prepareMeshPyramid(
    //     IntegerContainerType& meshNeighborNum,
    //     CoordinateContainerType& meshNeighborRadius,
    //     bool meshPyramidUseRadius,
    //     MeshPropagation& meshPropagation);

};

template<class FloatType>
MeshPyramid<FloatType>::MeshPyramid(MeshData<FloatType>& mesh)
{
    // creare a MeshPyramid from a single mesh
    // in this case, we just got one level,
    // mesh subsampling to be added if necessary

    numLevels = 1;
    levels.resize(1);
    meshPyramidVertexNum.resize(1);
    meshPyramidVertexNum.push_back(mesh.numVertices);
    levels[0] = std::move(mesh);

}

template<class FloatType>
MeshPyramid<FloatType>::MeshPyramid(string meshLevelFormat,
    IntegerContainerType& meshVertexNum)
{
    numLevels = meshVertexNum.size();
    levels.resize(numLevels);
    meshPyramidVertexNum.resize(numLevels);

    // load meshes
    char buffer[BUFFER_SIZE];
    Mesh tempMesh;
    for(int i = 0; i < numLevels; ++i)
    {
        std::stringstream meshFile;
        sprintf(buffer, meshLevelFormat.c_str(), meshVertexNum[i]);
        meshFile << buffer;
        PangaeaMeshIO::loadfromFile(meshFile.str(), tempMesh);

        levels[i] = std::move(tempMesh);
        meshPyramidVertexNum[i] = levels[i].numVertices;

    }

}

template<class FloatType>
MeshPyramid<FloatType>::MeshPyramid(string meshPath,
    string meshLevelFormat, int frame, IntegerContainerType& meshLevelList)
{
    numLevels = meshLevelList.size();
    levels.resize(numLevels);
    meshPyramidVertexNum.resize(numLevels);

    // load meshes
    char buffer[BUFFER_SIZE];
    Mesh tempMesh;
    for(int i = 0; i < numLevels; ++i)
    {
        std::stringstream meshFile;
        sprintf(buffer, meshLevelFormat.c_str(), frame, meshLevelList[i]);
        meshFile << meshPath << buffer;
        PangaeaMeshIO::loadfromFile(meshFile.str(), tempMesh);
        levels[i] = std::move(tempMesh);

        meshPyramidVertexNum[i] = levels[i].numVertices;
    }
}


template<class FloatType>
void MeshPyramid<FloatType>::updatePyramid(string meshPath,
    string meshLevelFormat, int frame, IntegerContainerType& meshLevelList)
{
    // just do update, no memory allocation and much faster
    // load meshes
    char buffer[BUFFER_SIZE];
    for(int i = 0; i < numLevels; ++i)
    {
        std::stringstream meshFile;
        sprintf(buffer, meshLevelFormat.c_str(), frame, meshLevelList[i]);
        meshFile << meshPath << buffer;
        PangaeaMeshIO::updateFromFile(meshFile.str(), levels[i]);
    }
}

typedef MeshPyramid<CoordinateType> PangaeaMeshPyramid;
typedef MeshPropagation<CoordinateType> PangaeaMeshPropagation;