#pragma once

#include "./Mesh.h"
#include "./ImagePyramid.h"

#include "sample.h"
#include "jet_extras.h"
#include "ceres/rotation.h"

enum baType{
    BA_MOT,
    BA_STR,
    BA_MOTSTR
};

enum dataTermErrorType{
    PE_INTENSITY = 0,
    PE_COLOR,
    PE_DEPTH,
    PE_DEPTH_PLANE,
    PE_NCC,
    PE_COLOR_NCC,
    NUM_DATA_TERM_ERROR
};

static int PE_RESIDUAL_NUM_ARRAY[NUM_DATA_TERM_ERROR] = {1,3,3,1,1,3};

//
template<typename T>
void IntrinsicProjection(const CameraInfo* pCamera, T* p, T* u, T* v)
{
    if(pCamera->isOrthoCamera)
    {
        u[0] = p[0]; //transformed x (2D)
        v[0] = p[1]; //transformed y (2D)
    }
    else
    {
        u[0] = ( ( p[0] * T( pCamera->KK[0][0] ) ) / p[2] ) + T( pCamera->KK[0][2] ); //transformed x (2D)
        v[0] = ( ( p[1] * T( pCamera->KK[1][1] ) ) / p[2] ) + T( pCamera->KK[1][2] ); //transformed y (2D)
    }
}

// backprojection
template<typename T>
void BackProjection(const CameraInfo* pCamera, const ImageLevel* pFrame, T* u, T* v, T* backProj)
{
    T currentValue;

    currentValue = SampleWithDerivative< T, InternalIntensityImageType > (pFrame->depthImage,
        pFrame->depthGradXImage, pFrame->depthGradYImage, u[0], v[0]);

    backProj[2] = currentValue;

    if(pCamera->isOrthoCamera)
    {
        backProj[0] = u[0]; backProj[1] = v[0];
    }else
    {
        backProj[0] = backProj[2] * (pCamera->invKK[0][0]*u[0] + pCamera->invKK[0][2]);
        backProj[1] = backProj[2] * (pCamera->invKK[1][1]*v[0] + pCamera->invKK[1][2]);
    }

}

// patch based score estimation
template<typename T>
void nccScore(vector<T>& neighborValues, vector<T>& projValues, int k1, int k2, T* pScore)
{
    T neighborMean, projMean, neighborSTD, projSTD;
    neighborMean = T(0.0); projMean = T(0.0);
    neighborSTD = T(0.0); projSTD = T(0.0);

    pScore[0] = T(0.0);

    int num = neighborValues.size();
    for(int i = 0; i < num; ++i)
    {
        neighborMean += neighborValues[ k1*i + k2 ];
        projMean += projValues[ k1*i+k2 ];
    }

    neighborMean = neighborMean / T(num);
    projMean = projMean / T(num);

    for(int i = 0; i < num; ++i)
    {
        neighborSTD += (neighborValues[k1*i + k2] - neighborMean) *
            (neighborValues[k1*i + k2] - neighborMean);
        projSTD += (projValues[k1*i + k2] - projMean) *
            (projValues[k1*i + k2] - projMean);
    }

    neighborSTD = sqrt(neighborSTD);
    projSTD = sqrt(projSTD);

    for(int i = 0; i < num; ++i)
    {
        pScore[0] += (neighborValues[k1*i + k2] - neighborMean) / neighborSTD *
            (projValues[k1*i + k2] - projMean) / projSTD;
    }
}

template<typename T>
void getPatchScore(vector<T>& neighborValues, vector<T>& projValues,
    T* pScore, const dataTermErrorType& PE_TYPE=PE_NCC)
{
    switch(PE_TYPE)
    {
        case PE_NCC:

            nccScore(neighborValues, projValues, 1, 0, pScore);
            break;

        case PE_COLOR_NCC:

            for(int i = 0; i < 3; ++i)
            nccScore(neighborValues, projValues, 3, i, pScore+i);

            break;

        default:

            cerr << "Not supported data term type in getPatchScore" << endl;
            break;
    }
}

template<typename T>
void getValue(const CameraInfo* pCamera, const ImageLevel* pFrame,
    T* p, T* value, const dataTermErrorType& PE_TYPE)
{
    T transformed_r, transformed_c;

    IntrinsicProjection(pCamera, p, &transformed_c, &transformed_r);

    T templateValue, currentValue;

    if( transformed_r >= T(0.0) && transformed_r < T(pCamera->height) &&
        transformed_c >= T(0.0) && transformed_c < T(pCamera->width))
    {
        switch(PE_TYPE)
        {
            case PE_INTENSITY:
            {
                value[0] = SampleWithDerivative< T, InternalIntensityImageType > (pFrame->grayImage,
                    pFrame->gradXImage, pFrame->gradYImage, transformed_c, transformed_r );
                break;
            }
            case PE_COLOR:
            {
                for(int i = 0; i < 3; ++i)
                {
                    value[i] = SampleWithDerivative< T, InternalIntensityImageType >( pFrame->colorImageSplit[i],
                        pFrame->colorImageGradXSplit[i], pFrame->colorImageGradYSplit[i], transformed_c, transformed_r );
                }
                break;
            }
            case PE_DEPTH:   // point-to-point error
            {
                // depth value of the point
                BackProjection(pCamera, pFrame, &transformed_c, &transformed_r, value);

                break;
            }
            case PE_DEPTH_PLANE: // point-to-plane error
            {
                //
                BackProjection(pCamera, pFrame, &transformed_c, &transformed_r, value);

                // normals at back projection point
                T normals_at_bp[3];
                for(int  i = 0; i < 3; ++i)
                {
                    normals_at_bp[i] = SampleWithDerivative< T, InternalIntensityImageType > (pFrame->depthNormalImageSplit[i],
                        pFrame->depthNormalImageGradXSplit[i], pFrame->depthNormalImageGradYSplit[i], transformed_c, transformed_r );

                    p[i] = normals_at_bp[i] * p[i];
                    value[i] = normals_at_bp[i] * value[i];
                }

            }
        }
    }

}

template<typename T>
void getResidual(double weight, const CameraInfo* pCamera, const ImageLevel* pFrame,
    double* pValue, T* p, T* residuals, const dataTermErrorType& PE_TYPE)
{
    T transformed_r, transformed_c;

    IntrinsicProjection(pCamera, p, &transformed_c, &transformed_r);

    T templateValue, currentValue;

    if( transformed_r >= T(0.0) && transformed_r < T(pCamera->height) &&
        transformed_c >= T(0.0) && transformed_c < T(pCamera->width))
    {
        switch(PE_TYPE)
        {
            case PE_INTENSITY:
            {
                templateValue = T(pValue[0]);
                currentValue = SampleWithDerivative< T, InternalIntensityImageType > (pFrame->grayImage,
                    pFrame->gradXImage, pFrame->gradYImage, transformed_c, transformed_r );
                residuals[0] = T(weight) * (currentValue - templateValue);
                break;
            }
            case PE_COLOR:
            {
                for(int i = 0; i < 3; ++i)
                {
                    templateValue = T(pValue[i]);
                    currentValue = SampleWithDerivative< T, InternalIntensityImageType >( pFrame->colorImageSplit[i],
                        pFrame->colorImageGradXSplit[i], pFrame->colorImageGradYSplit[i], transformed_c, transformed_r );
                    residuals[i] = T(weight) * ( currentValue - templateValue );
                }
                break;
            }
            case PE_DEPTH:   // point-to-point error
            {
                // depth value of the point
                T back_projection[3];
                BackProjection(pCamera, pFrame, &transformed_c, &transformed_r, back_projection);

                residuals[0] = T(weight) * (p[0] - back_projection[0]);
                residuals[1] = T(weight) * (p[1] - back_projection[1]);
                residuals[2] = T(weight) * (p[2] - back_projection[2]);

                break;
            }
            case PE_DEPTH_PLANE: // point-to-plane error
            {
                //
                T back_projection[3];
                BackProjection(pCamera, pFrame, &transformed_c, &transformed_r, back_projection);

                // normals at back projection point
                T normals_at_bp[3];
                for(int  i = 0; i < 3; ++i)
                {
                    normals_at_bp[i] = SampleWithDerivative< T, InternalIntensityImageType > (pFrame->depthNormalImageSplit[i],
                        pFrame->depthNormalImageGradXSplit[i], pFrame->depthNormalImageGradYSplit[i], transformed_c, transformed_r );
                }

                // should we do normalization on the normals, probably doesn't make much difference
                // as the normals has already been normalized and we just do
                // interpolation

                // get the point-to-plane error
                residuals[0] = T(weight)*(
                    normals_at_bp[0]*(p[0] - back_projection[0]) +
                    normals_at_bp[1]*(p[1] - back_projection[1]) +
                    normals_at_bp[2]*(p[2] - back_projection[2]) );
            }
        }
    }

}

template<typename T>
void getPatchResidual(double weight, const CameraInfo* pCamera, const ImageLevel* pFrame,
    const PangaeaMeshData* pMesh, vector<T>& neighborVertices,
    int numNeighbors, const vector<unsigned int>& neighbors, T* residuals,
    const dataTermErrorType& PE_TYPE=PE_NCC)
{
    vector<T> neighborValues;
    vector<T> projValues;

    switch(PE_TYPE)
    {
        case PE_NCC:
        {
            neighborValues.resize(numNeighbors);
            projValues.resize(numNeighbors);

            for(int i = 0; i < numNeighbors; ++i)
            {
                getValue(pCamera, pFrame, &neighborVertices[3*i], &projValues[i], PE_INTENSITY);
                neighborValues.push_back( T(pMesh->grays[ neighbors[i] ]) );
            }

            T gray_score;
            gray_score = T(0.0);
            getPatchScore(neighborValues, projValues, &gray_score, PE_TYPE);

            residuals[0] = T(weight) * (T(1.0) - gray_score);
        }
        break;

        case PE_COLOR_NCC:
        {

            neighborValues.resize( 3*numNeighbors );
            projValues.resize( 3*numNeighbors );

            for(int i = 0; i < numNeighbors; ++i)
            {
                getValue(pCamera, pFrame, &neighborVertices[3*i], &projValues[i], PE_COLOR);
                for(int k = 0; k < 3; ++k)
                neighborValues.push_back( T(pMesh->colors[ neighbors[i] ][k]) );
            }

            T color_score[3];
            getPatchScore(neighborValues, projValues, color_score, PE_TYPE);

            for(int i = 0; i < 3; ++i)
            residuals[i] = T(weight) * (T(1.0) - color_score[i]);

        }
        break;
    }


}

// need to write a new ResidualImageProjection(ResidualImageInterpolationProjection)
// for a data term similar to dynamicFusion
// optional parameters: vertex value(will be needed if we are using photometric)
// paramters: vertex position, neighboring weights, pCamera and pFrame
// optimization: rotation, translation and neighboring transformations
// (split up rotation and translation)
// several things we could try here:
// fix the number of neighbors, if we use knn nearest neighbors
// or use variable number of neighbors, if we use neighbors in a certain radius range
// arap term of dynamicFusion can be implemented as usual

// Image Projection residual covers all the possible projection cases
// Including gray, rgb, point-to-point and point-to-plane error
// For different pyramid level, just change the pCamera and pFrame accordingly,
// make sure that pCamera and pFrame are consistent

class ResidualImageProjection
{
public:

ResidualImageProjection(double weight, double* pValue, double* pVertex,
    const CameraInfo* pCamera, const ImageLevel* pFrame,
    dataTermErrorType PE_TYPE=PE_INTENSITY):
    weight(weight),
        pValue(pValue),
        pVertex(pVertex),
        pCamera(pCamera),
        pFrame(pFrame),
        PE_TYPE(PE_TYPE),
        optimizeDeformation(true)
        {
            // check the consistency between camera and images
            assert(pCamera->width == pFrame->grayImage.cols);
            assert(pCamera->height == pFrame->grayImage.rows);
        }

ResidualImageProjection(double weight, double* pTemplateVertex,
    const CameraInfo* pCamera, const ImageLevel* pFrame,
    dataTermErrorType PE_TYPE=PE_INTENSITY):
    weight(weight),
        pVertex(pVertex),
        pCamera(pCamera),
        pFrame(pFrame),
        PE_TYPE(PE_TYPE),
        optimizeDeformation(true)
        {
            // check the consistency between camera and images
            assert(pCamera->width == pFrame->grayImage.cols);
            assert(pCamera->height == pFrame->grayImage.rows);
        }

    template<typename T>
        bool operator()(const T* const rotation,
            const T* const translation,
            const T* const xyz,
            T* residuals) const
    {
        int residual_num = PE_RESIDUAL_NUM_ARRAY[PE_TYPE];
        for(int i = 0; i < residual_num; ++i)
        residuals[i] = T(0.0);

        T p[3], afterTrans[3];

        // if we are doing optimization on the transformation,
        // we need to add up the template position first
        if(optimizeDeformation)
        {
            afterTrans[0] = xyz[0] + pVertex[0];
            afterTrans[1] = xyz[1] + pVertex[1];
            afterTrans[2] = xyz[2] + pVertex[2];
        }
        else
        {
            afterTrans[0] = xyz[0];
            afterTrans[1] = xyz[1];
            afterTrans[2] = xyz[2];
        }

        ceres::AngleAxisRotatePoint( rotation, afterTrans, p);
        p[0] += translation[0];
        p[1] += translation[1];
        p[2] += translation[2];

        //debug
        // double xyz_[3],rotation_[3],translation_[3];
        // double p_[3];
        // for(int i = 0; i < 3; ++i)
        // {
        //     xyz_[i] = ceres::JetOps<T>::GetScalar(xyz[i]);
        //     rotation_[i] = ceres::JetOps<T>::GetScalar(rotation[i]);
        //     translation_[i] = ceres::JetOps<T>::GetScalar(translation[i]);
        //     p_[i] = ceres::JetOps<T>::GetScalar(p[i]);
        // }

        getResidual(weight, pCamera, pFrame, pValue, p, residuals, PE_TYPE);

        return true;
    }

private:
    bool optimizeDeformation;  // whether optimize deformation directly
    double* pVertex;
    double weight;
    // this will only be useful if we are using gray or rgb value
    // give a dummy value in other cases
    double* pValue;
    const CameraInfo* pCamera;
    const ImageLevel* pFrame;
    dataTermErrorType PE_TYPE;
};


// ResidualImageProjection from coarse level deformation,
class ResidualImageProjectionCoarse
{
public:
ResidualImageProjectionCoarse(double weight, double* pValue, double* pVertex,
    const CameraInfo* pCamera, const ImageLevel* pFrame, int numNeighbors,
    vector<double> neighborWeights, vector<double*> neighborVertices,
    dataTermErrorType PE_TYPE=PE_INTENSITY):
    weight(weight),
        pValue(pValue),
        pVertex(pVertex),
        pCamera(pCamera),
        pFrame(pFrame),
        numNeighbors(numNeighbors),
        neighborWeights(neighborWeights),
        neighborVertices(neighborVertices),
        PE_TYPE(PE_TYPE)
        {
            // check the consistency between camera and images
            assert(pCamera->width == pFrame->grayImage.cols);
            assert(pCamera->height == pFrame->grayImage.rows);
        }

    template<typename T>
        bool operator()(const T* const* const parameters, T* residuals) const
    {
        int residual_num = PE_RESIDUAL_NUM_ARRAY[PE_TYPE];
        for(int i = 0; i < residual_num; ++i)
        residuals[i] = T(0.0);

        // v is the position after applying deformation only
        // p is the position after rigid transformation on v
        T v[3], p[3], diff_vertex[3], rot_diff_vertex[3];
        v[0] = T(0.0); v[1] = T(0.0); v[2] = T(0.0);
        p[0] = T(0.0); p[1] = T(0.0); p[2] = T(0.0);
        T transformed_r, transformed_c;

        const T* const* const trans = parameters;
        const T* const* const rotations = &(parameters[ numNeighbors ]);

        const T* const rigid_rot = parameters[ 2*numNeighbors ];
        const T* const rigid_trans = parameters[ 2*numNeighbors+1 ];

        // compute the position from coarse neighbors nodes first
        for(int i = 0; i < numNeighbors; ++i)
        {
            // get template difference
            for(int index = 0; index < 3; ++index)
            diff_vertex[index] = T(pVertex[index]) - neighborVertices[ i ][ index ];

            ceres::AngleAxisRotatePoint( &(rotations[ i ][ 0 ]), diff_vertex, rot_diff_vertex );

            for(int index = 0; index < 3; ++index)
            v[index] += neighborWeights[i] * (rot_diff_vertex[ index ]
                + neighborVertices[ i ][ index ] + trans[ i ][ index] );
        }

        ceres::AngleAxisRotatePoint( rigid_rot, v , p);
        p[0] += rigid_trans[0];
        p[1] += rigid_trans[1];
        p[2] += rigid_trans[2];

        getResidual(weight, pCamera, pFrame, pValue, p, residuals, PE_TYPE);

        return true;

    }

private:

    double* pVertex;
    double weight;
    double* pValue;
    const CameraInfo* pCamera;
    const ImageLevel* pFrame;
    dataTermErrorType PE_TYPE;

    int numNeighbors;
    vector<double> neighborWeights;
    vector<double*> neighborVertices;

};

// ResidualImageProjectionPatch
class ResidualImageProjectionPatch
{
public:

ResidualImageProjectionPatch(double weight, const PangaeaMeshData* pMesh,
    const CameraInfo* pCamera, const ImageLevel* pFrame, int numNeighbors,
    vector<double> neighborWeights, vector<unsigned int> neighborRadii,
    vector<unsigned int> neighbors, dataTermErrorType PE_TYPE=PE_NCC):
    weight(weight),
        pMesh(pMesh),
        pCamera(pCamera),
        pFrame(pFrame),
        numNeighbors(numNeighbors),
        neighborWeights(neighborWeights),
        neighborRadii(neighborRadii),
        neighbors(neighbors),
        PE_TYPE(PE_TYPE)
        {
            // check the consistency between camera and images
            assert(pCamera->width == pFrame->grayImage.cols);
            assert(pCamera->height == pFrame->grayImage.rows);
        }

    template<typename T>
        bool operator()(const T* const* const parameters, T* residuals) const
    {
        int residual_num = PE_RESIDUAL_NUM_ARRAY[PE_TYPE];
        for(int i = 0; i < residual_num; ++i)
        residuals[i] = T(0.0);

        const T* const* const trans = parameters;

        const T* const rigid_rot = parameters[ numNeighbors ];
        const T* const rigid_trans = parameters[ numNeighbors + 1 ];

        vector<T> neighborVertices;
        neighborVertices.resize( 3*numNeighbors );

        T p[3];
        for(int i = 0; i < numNeighbors; ++i)
        {
            for( int k = 0; k < 3; ++k)
            p[k] = trans[i][k] + T(pMesh->vertices[ neighbors[i] ][k]);

            ceres::AngleAxisRotatePoint( rigid_rot, p, &neighborVertices[3*i] );

            for( int k = 0; k < 3; ++k)
            neighborVertices[3*i+k] += rigid_trans[k];
        }

        getPatchResidual(weight, pCamera, pFrame, pMesh, neighborVertices, numNeighbors, neighbors, residuals, PE_TYPE);

        return true;

    }


private:

    double weight;

    const PangaeaMeshData* pMesh;
    const CameraInfo* pCamera;
    const ImageLevel* pFrame;

    int numNeighbors;

    vector<unsigned int> neighborRadii;
    vector<unsigned int> neighbors;
    vector<double> neighborWeights;

    vector<double> neighborVertexPositions;
    
    dataTermErrorType PE_TYPE;

};

// ResidualImageProjectionPatch from coarse level deformation
class ResidualImageProjectionPatchCoarse
{
public:

    ResidualImageProjectionPatchCoarse(double weight, const PangaeaMeshData* pMesh,
        const PangaeaMeshData* pNeighborMesh, const CameraInfo* pCamera,
        const ImageLevel* pFrame, int numNeighbors, int numCoarseNeighbors,
        vector<double> neighborWeights, vector<unsigned int> neighborRadii,
        vector<unsigned int> neighbors, vector<unsigned int> parameterIndices,
        vector<unsigned int> coarseNeighborIndices, vector<unsigned int> coarseNeighborBiases,
        vector<double> coarseNeighborWeights, dataTermErrorType PE_TYPE=PE_NCC):
        weight(weight),
        pMesh(pMesh),
        pNeighborMesh(pNeighborMesh),
        pCamera(pCamera),
        pFrame(pFrame),
        numNeighbors(numNeighbors),
        numCoarseNeighbors(numCoarseNeighbors),
        neighborWeights(neighborWeights),
        neighborRadii(neighborRadii),
        neighbors(neighbors),
        parameterIndices(parameterIndices),
        coarseNeighborIndices(coarseNeighborIndices),
        coarseNeighborBiases(coarseNeighborBiases),
        coarseNeighborWeights(coarseNeighborWeights),
        PE_TYPE(PE_TYPE)
    {
        // check the consistency between camera and images
        assert(pCamera->width == pFrame->grayImage.cols);
        assert(pCamera->height == pFrame->grayImage.rows);
    }

    template<typename T>
    bool operator()(const T* const* const parameters, T* residuals) const
    {

        int residual_num = PE_RESIDUAL_NUM_ARRAY[PE_TYPE];
        for(int i = 0; i < residual_num; ++i)
        residuals[i] = T(0.0);

        const T* const* const trans = parameters;
        const T* const* const rotations = &(parameters[ numCoarseNeighbors ]  );

        const T* const rigid_rot = parameters[ 2*numCoarseNeighbors ];
        const T* const rigid_trans = parameters[ 2*numCoarseNeighbors + 1 ];

        vector<T> neighborVertices;
        neighborVertices.resize( 3*numNeighbors );

        // computer the position from coarse neighbors nodes first

        T p[3], diff_vertex[3], rot_diff_vertex[3];

        int startPos = 0; // starting position for neighbors of i
        for(int i = 0; i < numNeighbors; ++i)
        {

            p[0] = T(0.0); p[1] = T(0.0); p[2] = T(0.0);

            int endPos = coarseNeighborBiases[i];  // ending position for neighbors of i

            for(int j = startPos; j < endPos; ++j)
            {
                for(int k = 0; k < 3; ++k)
                diff_vertex[k] = pMesh->vertices[ neighbors[i] ][k] - T(pNeighborMesh->vertices[ coarseNeighborIndices[ j ]  ][k] );

                ceres::AngleAxisRotatePoint( &(rotations[ parameterIndices[j] ][ 0 ]), diff_vertex, rot_diff_vertex );

                for(int index = 0; index < 3; ++index)
                p[index] += coarseNeighborWeights[j] * (rot_diff_vertex[ index ]
                    + pNeighborMesh->vertices[ coarseNeighborIndices[j] ][ index ] + trans[ parameterIndices[j] ][ index] );
            }

            startPos = endPos;

            ceres::AngleAxisRotatePoint( rigid_rot, p, &neighborVertices[3*i] );

            for( int k = 0; k < 3; ++k)
            neighborVertices[3*i+k] += rigid_trans[k];

        }

        getPatchResidual(weight, pCamera, pFrame, pMesh, neighborVertices, numNeighbors, neighbors, residuals, PE_TYPE);

        return true;

    }

private:

    double weight;

    const PangaeaMeshData* pMesh;
    const PangaeaMeshData* pNeighborMesh;
    const CameraInfo* pCamera;
    const ImageLevel* pFrame;

    int numNeighbors;
    int numCoarseNeighbors;

    vector<unsigned int> neighborRadii;
    vector<unsigned int> neighbors;
    vector<double> neighborWeights;

    vector<unsigned int> parameterIndices;
    vector<unsigned int> coarseNeighborIndices;
    vector<unsigned int> coarseNeighborBiases;
    vector<double> coarseNeighborWeights;

    dataTermErrorType PE_TYPE;

};

class ResidualTV
{
public:

ResidualTV(double weight):
    weight(weight), optimizeDeformation(true) {}

ResidualTV(double weight, double* pVertex, double* pNeighbor):
    weight(weight), pVertex(pVertex), pNeighbor(pNeighbor), optimizeDeformation(false) {}

    template <typename T>
        bool operator()(const T* const pCurrentVertex,
            const T* const pCurrentNeighbor,
            T* residuals) const
    {

        if(optimizeDeformation){
            // in this case, pCurrentVertex and pCurrentNeighbor refer to translation
            for(int i = 0; i <3; ++i)
            residuals[i] = T(weight) * ( pCurrentVertex[i] - pCurrentNeighbor[i]);
        }
        else{
            for(int i = 0; i <3; ++i)
            residuals[i] = T(weight) * ( T(pVertex[i]  - pNeighbor[i]) -
                ( pCurrentVertex[i] - pCurrentNeighbor[i]) );
        }
        return true;
    }

private:
    bool optimizeDeformation;
    double weight;
    const double* pVertex;
    const double* pNeighbor;

};

// total variation on top of the local rotations
class ResidualRotTV
{
public:

ResidualRotTV(double weight):weight(weight){}

    template<typename T>
        bool operator()(const T* const pCurrentRot,
            const T* const pCurrentNeighbor,
            T* residuals) const
    {
        for(int i= 0; i < 3; ++i)
        residuals[i] = T(weight) * ( pCurrentRot[i] - pCurrentNeighbor[i] );

        return true;
    }

private:

    double weight;
};

class ResidualINEXTENT
{
public:

ResidualINEXTENT(double weight):
    weight(weight), optimizeDeformation(true)
    {}

ResidualINEXTENT(double weight, double* pVertex, double* pNeighbor):
    weight(weight),pVertex(pVertex),pNeighbor(pNeighbor), optimizeDeformation(false)
    {}

    template <typename T>
        bool operator()(const T* const pCurrentVertex,
            const T* const pCurrentNeighbor,
            T* residuals) const
    {
        T diff[3];
        T diffref[3];

        if(optimizeDeformation){
            for(int i = 0; i < 3; i++){
                diff[i] = pCurrentNeighbor[i];
                diffref[i] = T(pNeighbor[i]);
            }
        }
        else{
            for(int i = 0; i < 3; ++i){
                diff[i] = pCurrentVertex[i] - pCurrentNeighbor[i];
                diffref[i] = T(pVertex[i] - pNeighbor[i]);
            }
        }

        T length;
        T lengthref;

        length = sqrt(diff[0] * diff[0] +
            diff[1] * diff[1] +
            diff[2] * diff[2]);

        lengthref = sqrt(diffref[0] * diffref[0] +
            diffref[1] * diffref[1] +
            diffref[2] * diffref[2]);

        residuals[0] = T(weight) * (lengthref - length);

        return true;

    }

private:

    bool optimizeDeformation;
    double weight;
    const double* pVertex;
    const double* pNeighbor;

};

// the rotation to be optimized is from template mesh to current mesh
class ResidualARAP
{
public:

ResidualARAP(double weight, double* pVertex, double* pNeighbor, bool optDeform=false):
    weight(weight), pVertex(pVertex), pNeighbor(pNeighbor), optimizeDeformation(optDeform) {}

    template <typename T>
        bool operator()(const T* const pCurrentVertex,
            const T* const pCurrentNeighbor,
            const T* const pRotVertex,
            T* residuals) const
    {
        T templateDiff[3];
        T rotTemplateDiff[3];
        T currentDiff[3];

        if(optimizeDeformation){
            // deformation optimization
            for(int i = 0; i < 3; ++i){
                templateDiff[i] =  T(pVertex[i] - pNeighbor[i]);
                currentDiff[i] = templateDiff[i] + (pCurrentVertex[i] - pCurrentNeighbor[i]);
            }
        }
        else{
            for(int i = 0; i <3; ++i){
                templateDiff[i] =  T(pVertex[i] - pNeighbor[i]);
                currentDiff[i]  = pCurrentVertex[i] - pCurrentNeighbor[i];
            }
        }

        ceres::AngleAxisRotatePoint(pRotVertex, templateDiff, rotTemplateDiff);

        residuals[0] = T(weight) * ( currentDiff[0] - rotTemplateDiff[0] );
        residuals[1] = T(weight) * ( currentDiff[1] - rotTemplateDiff[1] );
        residuals[2] = T(weight) * ( currentDiff[2] - rotTemplateDiff[2] );

        return true;

    }

private:

    bool optimizeDeformation;
    double weight;
    const double* pVertex;
    const double* pNeighbor;

};

// temporal shape
class ResidualDeform
{
public:

ResidualDeform(double weight, double* pVertex):
    weight(weight), pVertex(pVertex) {}

    template <typename T>
        bool operator()(const T* const pCurrentVertex,
            T* residuals) const
    {
        for(int i = 0; i < 3; ++i)
        residuals[i] = T(weight) * (pCurrentVertex[i] - T(pVertex[i]));

        return true;
    }

private:

    bool optimizeDeformation;
    double weight;
    const double* pVertex;

};

class ResidualTemporalMotion
{
public:
ResidualTemporalMotion(double* pPrevRot, double* pPrevTrans,
    double rotWeight, double transWeight):
    pPrevRot(pPrevRot), pPrevTrans(pPrevTrans),
        rotWeight(rotWeight), transWeight(transWeight)
    {}

    template <typename T>
        bool operator()(const T* const pRot,
            const T* const pTrans,
            T* residuals) const
    {
        residuals[0] = rotWeight * (pRot[0] -     pPrevRot[0]);
        residuals[1] = rotWeight * (pRot[1] -     pPrevRot[1]);
        residuals[2] = rotWeight * (pRot[2] -     pPrevRot[2]);
        residuals[3] = transWeight * (pTrans[0] - pPrevTrans[0]);
        residuals[4] = transWeight * (pTrans[1] - pPrevTrans[1]);
        residuals[5] = transWeight * (pTrans[2] - pPrevTrans[2]);

        return true;
    }

    double rotWeight;
    double transWeight;
    const double* pPrevRot;
    const double* pPrevTrans;

};


class EnergyCallback: public ceres::IterationCallback
{

private:
    std::vector<double> m_EnergyRecord;

public:
    EnergyCallback(){}
    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary & summary)
    {
        m_EnergyRecord.push_back(summary.cost);
        return ceres::SOLVER_CONTINUE;
    }
    void PrintEnergy(std::ostream& output)
    {
        output << "Energy Started" << std::endl;
        for(int i=0; i< m_EnergyRecord.size(); ++i)
        output<<(i+1)<<" "<< m_EnergyRecord[i]<<std::endl;
        output << "Energy Ended" << std::endl;
    }
    void Reset()
    {
        m_EnergyRecord.clear();
    }

};