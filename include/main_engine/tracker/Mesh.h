#pragma once

#include "./MeshData.h"
#include "./MeshIO.h"
#include "./MeshPyramid.h"

// get Mesh Bounding Box
template<class FloatType>
void getMeshBoundingBox(MeshData<FloatType>& meshData,
    vector<vector<FloatType> >& bbox);

template<class FloatType>
void scaleMeshUp(MeshData<FloatType>& meshData, double factor);

template<class FloatType>
void calcNeighborsAndWeights(vector<vector<FloatType> >&  fineMesh,
    vector<vector<FloatType> >& coarseMesh,
    MeshNeighbors& neighbors,
    vector<vector<FloatType> >& distances,
    int knn);

template<class FloatType>
void calcNeighborsAndWeights(vector<vector<FloatType> >&  fineMesh,
    vector<vector<FloatType> >& coarseMesh,
    MeshNeighbors& neighbors,
    vector<vector<FloatType> >& distances,
    FloatType radius);

template<class FloatType>
void distToWeights(vector<vector<FloatType> >& distances,
    vector<vector<FloatType> >& weights);

template<class FloatType>
void setupPropagation(MeshPyramid<FloatType>& meshPyramid,
    MeshPropagation& meshPropagation,
    IntegerContainerType& meshNeighborNum,
    CoordinateContainerType& meshNeighborRadius,
    bool meshPyramidUseRadius);

// get Mesh Bounding Box
template<class FloatType>
void getMeshBoundingBox(MeshData<FloatType>& meshData,
    vector<vector<FloatType> >& bbox)
{
    double xmin, xmax, ymin, ymax, zmin, zmax;
    xmin = ymin = zmin = numeric_limits<double>::max();
    xmax = ymax = zmax = numeric_limits<double>::min();

    for(int i = 0; i < meshData.numVertices; ++i)
    {
        xmin = xmin < meshData.vertices[i][0] ? xmin : meshData.vertices[i][0];
        ymin = ymin < meshData.vertices[i][1] ? ymin : meshData.vertices[i][1];
        zmin = zmin < meshData.vertices[i][2] ? zmin : meshData.vertices[i][2];

        xmax = xmax > meshData.vertices[i][0] ? xmax : meshData.vertices[i][0];
        ymax = ymax > meshData.vertices[i][1] ? ymax : meshData.vertices[i][1];
        zmax = zmax > meshData.vertices[i][2] ? zmax : meshData.vertices[i][2];
    }

    bbox.resize(3);
    bbox[0].push_back(xmin); bbox[0].push_back(xmax);
    bbox[1].push_back(ymin); bbox[1].push_back(ymax);
    bbox[2].push_back(zmin); bbox[2].push_back(zmax);
}

template<class FloatType>
void scaleMeshUp(MeshData<FloatType>& meshData, double factor)
{
    // just scale the vertices up
    for(int i = 0; i < meshData.numVertices; ++i)
    {
        meshData.vertices[i][0] *= factor;
        meshData.vertices[i][1] *= factor;
        meshData.vertices[i][2] *= factor;
    }
}

// Mesh Pyramid
template<class FloatType>
void calcNeighborsAndWeights(vector<vector<FloatType> >&  fineMesh,
    vector<vector<FloatType> >& coarseMesh,
    MeshNeighborsNano& neighbors,
    vector<vector<FloatType> >& distances,
    int knn)
{
    my_kd_tree_t mat_index(coarseMesh);
    mat_index.index->buildIndex();

    int numPoints = fineMesh.size();
    neighbors.resize(numPoints);
    distances.resize(numPoints);

    for(int j = 0; j < numPoints; ++j)
    {
        // do a knn search for each point in the fine mesh
        neighbors[j].resize(knn);
        distances[j].resize(knn);
        mat_index.index->knnSearch(
            &fineMesh[j][0], knn,
            &neighbors[j][0], &distances[j][0]);
    }
}

template<class FloatType>
void calcNeighborsAndWeights(vector<vector<FloatType> >&  fineMesh,
    vector<vector<FloatType> >& coarseMesh,
    MeshNeighborsNano& neighbors,
    vector<vector<FloatType> >& distances,
    FloatType radius)
{
    my_kd_tree_t mat_index(coarseMesh);
    mat_index.index->buildIndex();

    int numPoints = fineMesh.size();
    neighbors.resize(numPoints);
    distances.resize(numPoints);

    for(int j = 0; j < numPoints; ++j)
    {
        // do a radius search for each point in the fine mesh
        std::vector<std::pair<size_t, FloatType> > ret_matches;
        nanoflann::SearchParams params;
        int nMatches = mat_index.index->radiusSearch(
            &fineMesh[j][0], radius,
            ret_matches, params);
        // copy over the matches
        neighbors[j].resize(nMatches);
        distances[j].resize(nMatches);
        for(int k = 0; k < nMatches; ++k)
        {
            neighbors[j][k] = ret_matches[k].first;
            distances[j][k] = ret_matches[k].second;
        }

    }
}

template<class FloatType>
void distToWeights(vector<vector<FloatType> >& distances,
    vector<vector<FloatType> >& weights,
    vector<FloatType>& sigmas)
{
    int numVertices = distances.size();
    weights.resize(numVertices);

    for(int i = 0; i < numVertices; ++i)
    {
        int numNeighbors = distances[i].size();
        weights[i].resize(numNeighbors);
        FloatType weightsSum = 0;
        for(int j = 0; j < numNeighbors; ++j)
        {
            weights[i][j] = exp(-distances[i][j] / (2*sigmas[i]));
            weightsSum += weights[i][j];
        }
        for(int j = 0; j < numNeighbors; ++j)
        {
            weights[i][j] = weights[i][j] / weightsSum;
        }
    }

}

template<class FloatType>
void setupPropagation(MeshPyramid<FloatType>& meshPyramid,
    MeshPropagation& meshPropagation,
    IntegerContainerType& meshNeighborNum,
    CoordinateContainerType& meshNeighborRadius,
    bool meshPyramidUseRadius)
{
    // // setup mesh propagation within a pyramid
    // typedef vector<FloatType> MeshSigmas;
    // typedef vector<vector<FloatType> > MeshDistances;
    // typedef vector<vector<size_t> > MeshNeighborsNano;

    // vector<MeshWeights>& weightsPyramid = meshPropagation.weightsVec;
    // vector<MeshNeighbors>& neighborsPyramid = meshPropagation.neighborsVec;

    // bool useRadius = meshPyramidUseRadius;

    // vector<MeshSigmas> sigmasPyramid;
    // vector<MeshDistances> distancesPyramid;
    // vector<MeshNeighborsNano> neighborsPyramidNano;

    // int numLevels = meshPyramid.levels.size();

    // weightsPyramid.resize(numLevels);
    // neighborsPyramid.resize(numLevels);

    // sigmasPyramid.resize(numLevels);
    // distancesPyramid.resize(numLevels);
    // neighborsPyramidNano.resize(numLevels);

    // for(int i = 1; i < numLevels; ++i)
    // {

    //     if(!useRadius)
    //     calcNeighborsAndWeights(meshPyramid.levels[i-1].vertices,
    //         meshPyramid.levels[i].vertices,
    //         neighborsPyramidNano[i-1],
    //         distancesPyramid[i-1],
    //         meshNeighborNum[i-1]);
    //     else
    //     calcNeighborsAndWeights(meshPyramid.levels[i-1].vertices,
    //         meshPyramid.levels[i].vertices,
    //         neighborsPyramidNano[i-1],
    //         distancesPyramid[i-1],
    //         meshNeighborRadius[i-1]);

    //     // determine the sigma(controling the coverage of weights) of each vertex,
    //     // setting to the maximum distance of each point at the moment
    //     int numVertices = meshPyramid.levels[i-1].vertices.size();
    //     sigmasPyramid[i-1].resize(numVertices);
    //     for(int j = 0; j < numVertices; ++j)
    //         sigmasPyramid[i-1][j] = distancesPyramid[i-1][j].back();

    //     distToWeights(distancesPyramid[i-1],
    //         weightsPyramid[i-1],
    //         sigmasPyramid[i-1]);

    // }

    // // copy over the neighbors from neighborsPyramid to neighborsPyramidUINT
    // for(int i = 0; i < numLevels-1; ++i)
    // {
    //     neighborsPyramid[i].resize(neighborsPyramidNano[i].size());
    //     for(int j = 0 ; j < neighborsPyramid[i].size(); ++j)
    //     {
    //         neighborsPyramid[i][j].resize(neighborsPyramidNano[i][j].size());
    //         for(int k = 0; k < neighborsPyramid[i][j].size(); ++k)
    //         neighborsPyramid[i][j][k] = neighborsPyramidNano[i][j][k];
    //     }

    //     // setup the neighborMap as well
    //     MeshPair meshPair(i, i);
    //     neighborMap.insert( pair<MeshPair, int >(meshPair, i) );
    // }

    int numLevels = meshPyramid.levels.size();
    
    for(int i = 1; i < numLevels; ++i)
    AddMeshToMeshPropagation(
        meshPyramid,
        i-1,    // current level
        i,      // neighbor level
        meshPropagation,
        meshNeighborNum[ i-1 ],
        meshNeighborRadius[ i-1 ],
        meshPyramidUseRadius);

    // using the same mesh for as neighbors
    // take care, a point will be a neighbor of itself
    for(int i = 0; i < numLevels; ++i)
    AddMeshToMeshPropagation(
        meshPyramid,
        i,    // current level
        i,      // neighbor level
        meshPropagation,
        meshNeighborNum[ i ],
        meshNeighborRadius[ i ],
        meshPyramidUseRadius);
}

// general setup propagation
template<class FloatType>
void AddMeshToMeshPropagation(MeshPyramid<FloatType>& meshPyramid, int meshLevel, int neighborLevel,
    MeshPropagation& meshPropagation, int kNN, double radius, bool useRadius = false)
{
    typedef vector<FloatType> MeshSigmas;
    typedef vector<vector<FloatType> > MeshDistances;

    MeshWeights weights;
    MeshNeighbors neighbors;

    MeshSigmas sigmas;
    MeshDistances distances;
    MeshNeighborsNano neighborsNano;

    MeshData<FloatType >& firstMesh = meshPyramid.levels[meshLevel];
    MeshData<FloatType >& secondMesh = meshPyramid.levels[neighborLevel];

    if(!useRadius)
    calcNeighborsAndWeights(firstMesh.vertices, secondMesh.vertices,
        neighborsNano, distances, kNN);
    else
    calcNeighborsAndWeights(firstMesh.vertices, secondMesh.vertices,
        neighborsNano, distances, radius);


    // determine the sigma(controling the coverage of weights) of each vertex,
    // setting to the maximum distance of each point at the moment
    int numVertices = firstMesh.vertices.size();
    sigmas.resize(numVertices);
    for(int j = 0; j < numVertices; ++j)
    sigmas[j] = distances[j].back();

    distToWeights(distances, weights, sigmas);

    neighbors.resize( neighborsNano.size() );
    for(int j = 0 ; j < neighborsNano.size(); ++j)
    {
        neighbors[j].resize(neighborsNano[j].size());
        for(int k = 0; k < neighborsNano[j].size(); ++k)
        neighbors[j][k] = neighborsNano[j][k];
    }

    meshPropagation.addNeighborsAndWeights(
        pair<int,int>(meshLevel, neighborLevel),
        neighbors, weights);

}

template<class FloatType>
void AddMeshToMeshPropagation(MeshPyramid<FloatType>& meshPyramid, vector<pair<int, int> >& termPairs,
    MeshPropagation& meshPropagation, vector<int> kNN, vector<double> radius, bool useRadius = false)
{
    for(int i = 0; i < termPairs.size(); ++i)
    {
        if( !meshPropagation.isAdded( termPairs[i] ) )
        AddMeshToMeshPropagation(
            meshPyramid,
            termPairs[i].first,
            termPairs[i].second,
            meshPropagation,
            kNN[i],
            radius[i]);
    }
}

// update output info, need to be updated
void UpdateRenderingData(TrackerOutputInfo& outputInfo, double KK[3][3],
    CoordinateType camPose[6], PangaeaMeshData& currentMesh);

void UpdateRenderingData(TrackerOutputInfo& outputInfo, double KK[3][3],
    CoordinateType camPose[6], PangaeaMeshData& templateMesh, MeshDeformation& meshTrans);

void UpdateRenderingDataFast(TrackerOutputInfo& outputInfo, double KK[3][3],
    PangaeaMeshData& currentMesh);

void UpdateVisibilityMask(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
    int width, int height);

void UpdateVisibilityMaskGL(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
    double KK[3][3], CoordinateType camPose[6], int width, int height, double toleranceRatio = 0.01);

void UpdateColorDiff(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
    InternalIntensityImageType colorImageSplit[3]);
