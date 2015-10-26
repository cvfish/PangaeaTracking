#ifndef _DEPTH_BUFFER_H
#define _DEPTH_BUFFER_H

#include "../utils/global.h"
#include "../tracker/Mesh.h"

#include "./CCamera.h"

#include <limits>

class DepthBuffer
{

public:

    DepthBuffer(const CCamera* pCamera, PangaeaMeshData* pMesh);
    ~DepthBuffer() {};

    // MatrixXCordRow _depthBufferMin;
    // MatrixXCordRow _depthBufferMax;
    MatrixXfRow _depthBufferMin;
    MatrixXfRow _depthBufferMax;

    float _zMin;
    float _zMax;

protected:

    void getTotalDepthRange(float& zMin, float& zMax) const;
    void renderMeshGL();
    void drawMeshGL();

    const CCamera* _pCamera;
    PangaeaMeshData* _pMesh;

    int _width;
    int _height;
    
};

#endif