#include "main_engine/rendering/DepthBuffer.h"
#include "main_engine/rendering/NcvGlXContext.h"

class DepthConverter
{
public:
    DepthConverter(const float a, const float b) : _a(a), _b(b)
    {}

    void operator() (float& v)
    {
        // opengl uses fixed-point representation inside, range [0 1]
        v = _b / (_a - 2.0*v + 1.0);
        // v = _b / (_a - v); // if the range is [-1 1]
    }

private:
    const float _a;
    const float _b;
};

DepthBuffer::DepthBuffer(const CCamera* pCamera, PangaeaMeshData* pMesh):
    _pCamera(pCamera), _pMesh(pMesh)
{
    _width = _pCamera->W();
    _height = _pCamera->H();

    _depthBufferMin.setZero(_width, _height);
    _depthBufferMax.setZero(_width, _height);

    // cout << "number of rows and cols" << endl;
    // cout << _depthbuffermin.rows() << endl;
    // cout << _depthbuffermin.cols() << endl;
    // cout << _depthbuffermin(0,0) << endl;

    float zMin, zMax;
    getTotalDepthRange(zMin, zMax);

    _zMin = zMin * 0.95f;
    _zMax = zMax * 1.05f;

    _depthBufferMin.fill(_zMin);
    _depthBufferMax.fill(_zMax);

    renderMeshGL();
}

void DepthBuffer::getTotalDepthRange(float& zMin, float& zMax) const
{
    zMax = - numeric_limits<float>::max();
    zMin = numeric_limits<float>::max();

    for(int I = _pMesh->numVertices, i = 0; i < I; ++i)
    {
        Vector3d Xc = _pCamera->worldToCamera(_pMesh->vertices[i]);
        float z = Xc[2];

        if(z > zMax)
        zMax = z;

        if(z < zMin)
        zMin = z;
    }

}

void DepthBuffer::renderMeshGL()
{
    uint W = _pCamera->W();
    uint H = _pCamera->H();
    ncv::GlXOffscreenContextPtr context(new ncv::GlXOffscreenContext(W, H));

    context->makeActive();
    // Should be able to use openGL here..

    glViewport(0,0,W,H);
    glDisable(GL_LIGHTING);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glLoadMatrix(_pCamera->getProjectionMatrix(_zMin, _zMax));

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrix(_pCamera->getModelViewMatrix());

    glEnable(GL_DEPTH_TEST);

    glClearDepth(1.0f);
    glDepthFunc(GL_LEQUAL);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw mesh..
    drawMeshGL();

    glFlush();

    double P[16];
    glGetDoublev(GL_PROJECTION_MATRIX, P);
    
    float a = - Map<Matrix4d>(P)(2,2);
    float b = - Map<Matrix4d>(P)(2,3);
    
    MatrixXfRow depthBuffer(H,W);
    MatrixXfRow depthBufferReverse(H,W);
    //    MatrixXfRow depthBufferReverseTest(H,W);
    glReadPixels(0, 0, W, H, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer.data());
    depthBufferReverse = depthBuffer.colwise().reverse();
    
    for_each(depthBufferReverse.data(), depthBufferReverse.data() + W*H, DepthConverter(a,b));
    _depthBufferMin = depthBufferReverse;

    // output depth data to txt file
    // std::ofstream depthBufferFile;
    // depthBufferFile.open("/cs/research/vision/humanis3/Rui/data/newsequence_3_19/photo_metric/depthBuffer.txt",
    //     std::ofstream::trunc);
    // depthBufferFile << _depthBufferMin << endl;

    // // render again to get the maximum depth
    // glClearDepth(0.0f);
    // glDepthFunc(GL_GEQUAL);
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // drawMeshGL();

    // glFlush();

    // glReadPixels(0, 0, W, H, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer.data());
    // depthBufferReverse = depthBuffer.colwise().reverse();
    // for_each(depthBufferReverse.data(), depthBufferReverse.data() + W*H, DepthConverter(a,b));
    // _depthBufferMax = depthBufferReverse;
    
}

void DepthBuffer::drawMeshGL()
{
    int numFaces = _pMesh -> numFaces;
    glBegin(GL_TRIANGLES);
    for(int i = 0; i < numFaces; ++i)
    {
        for(int k = 0; k < 3; ++k)
        {
            int offset = _pMesh->facesVerticesInd[i][k];
            glVertex3f(_pMesh->vertices[offset][0],
                _pMesh->vertices[offset][1],
                _pMesh->vertices[offset][2]);
        }
    }
    glEnd();
}