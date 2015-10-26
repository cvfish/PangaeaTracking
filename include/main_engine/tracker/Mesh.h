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
    MeshPropagation<FloatType>& meshPropagation,
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
    MeshNeighbors& neighbors,
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
    MeshNeighbors& neighbors,
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
    MeshPropagation<FloatType>& meshPropagation,
    IntegerContainerType& meshNeighborNum,
    CoordinateContainerType& meshNeighborRadius,
    bool meshPyramidUseRadius)

{

    typedef vector<vector<FloatType> > MeshWeights;
    typedef vector<vector<FloatType> > MeshDistances;
    typedef vector<FloatType> MeshSigmas;

    meshPropagation.useRadius = meshPyramidUseRadius;

    bool useRadius = meshPropagation.useRadius;
    IntegerContainerType& kNN = meshPropagation.KNN;
    CoordinateContainerType& radius = meshPropagation.radius;

    vector<MeshNeighbors>& neighborsPyramid = meshPropagation.neighborsPyramid;
    vector<MeshWeights>& weightsPyramid = meshPropagation.weightsPyramid;
    vector<MeshDistances>& distancesPyramid = meshPropagation.distancesPyramid;
    vector<MeshSigmas>& sigmasPyramid = meshPropagation.sigmasPyramid;
    vector<vector<vector<unsigned int> > >& neighborsPyramidUINT =
        meshPropagation.neighborsPyramidUINT;

    int numLevels = meshPyramid.levels.size();

    neighborsPyramid.resize(numLevels);
    distancesPyramid.resize(numLevels);
    sigmasPyramid.resize(numLevels);
    weightsPyramid.resize(numLevels);
    kNN.resize(numLevels);
    radius.resize(numLevels);

    for(int i = 0; i < numLevels-1; ++i)
    {
        kNN[i] = meshNeighborNum[i];
        radius[i] = meshNeighborRadius[i];
    }

    for(int i = 1; i < numLevels; ++i)
    {

        if(!useRadius)
        calcNeighborsAndWeights(meshPyramid.levels[i-1].vertices,
            meshPyramid.levels[i].vertices,
            neighborsPyramid[i-1],
            distancesPyramid[i-1],
            kNN[i-1]);
        else
        calcNeighborsAndWeights(meshPyramid.levels[i-1].vertices,
            meshPyramid.levels[i].vertices,
            neighborsPyramid[i-1],
            distancesPyramid[i-1],
            radius[i-1]);

        // determine the sigma(controling the coverage of weights) of each vertex,
        // setting to the maximum distance of each point at the moment
        int numVertices = meshPyramid.levels[i-1].vertices.size();
        sigmasPyramid[i-1].resize(numVertices);
        for(int j = 0; j < numVertices; ++j)
            sigmasPyramid[i-1][j] = distancesPyramid[i-1][j].back();

        distToWeights(distancesPyramid[i-1],
            weightsPyramid[i-1],
            sigmasPyramid[i-1]);

    }

    // copy over the neighbors from neighborsPyramid to neighborsPyramidUINT
    neighborsPyramidUINT.resize(numLevels);
    for(int i = 0; i < numLevels-1; ++i)
    {
        neighborsPyramidUINT[i].resize(neighborsPyramid[i].size());
        for(int j = 0 ; j < neighborsPyramidUINT[i].size(); ++j)
        {
            neighborsPyramidUINT[i][j].resize(neighborsPyramid[i][j].size());
            for(int k = 0; k < neighborsPyramid[i][j].size(); ++k)
            neighborsPyramidUINT[i][j][k] = neighborsPyramid[i][j][k];
        }

    }

}

// update output info, need to be updated
void UpdateRenderingData(TrackerOutputInfo& outputInfo, double KK[3][3],
    CoordinateType camPose[6], PangaeaMeshData& currentMesh);

void UpdateRenderingDataFast(TrackerOutputInfo& outputInfo, double KK[3][3],
    PangaeaMeshData& currentMesh);

void UpdateVisibilityMask(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
    int width, int height);

void UpdateVisibilityMaskGL(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
    double KK[3][3], CoordinateType camPose[6], int width, int height, double toleranceRatio = 0.01);

void UpdateColorDiff(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
    InternalIntensityImageType colorImageSplit[3]);
