#include "main_engine/tracker/TrackingEngine.h"

MeshBufferReader::MeshBufferReader(MeshLoadingSettings& settings, int width,
    int height, double K[3][3], int startFrame, int numTrackingFrames): trackerInitialized(false)
{
    m_nWidth = width;
    m_nHeight = height;
    startFrameNo = startFrame;
    currentFrameNo = startFrame;

    pCurrentColorImageRGB = new unsigned char[3*width*height];
    // in this case camPose will always be zero
    for(int i = 0; i < 6; ++i)
    camPose[i] = 0;

    useVisibilityMask = settings.visibilityMask;

    setIntrinsicMatrix(K);

    nRenderingLevel = 0;
    m_nNumMeshLevels = settings.meshLevelList.size();

    // a bit ugly
    nFrameStep = imageSourceSettings.frameStep;

    // loading meshes into buffer
    // outputInfoPyramidBuffer.resize(numTrackingFrames);
    // outputPropPyramidBuffer.resize(numTrackingFrames);
    int bufferSize = (numTrackingFrames - startFrameNo)/nFrameStep + 1;
    outputInfoPyramidBuffer.resize(bufferSize);
    outputPropPyramidBuffer.resize(bufferSize);

    m_nGoodFrames = 0;
    btime::ptime mst1  = btime::microsec_clock::local_time();
    for(int i = startFrameNo; i <= numTrackingFrames; i = i + nFrameStep)
    {

        btime::ptime loading1  = btime::microsec_clock::local_time();
        if(!existenceTest(settings.meshPath, settings.meshLevelFormat,
                i, settings.meshLevelList))
        break;

        ++m_nGoodFrames;
        currentMeshPyramid = std::move(PangaeaMeshPyramid(settings.meshPath,
                settings.meshLevelFormat, i, settings.meshLevelList));

        btime::ptime loading2 = btime::microsec_clock::local_time();
        btime::time_duration loadingDiff = loading2 - loading1;
        std::cout << "Loading Mesh Pyramid Time: " << loadingDiff.total_milliseconds() << std::endl;

        if(settings.loadProp)
        {
            propMeshPyramid = std::move(PangaeaMeshPyramid(settings.meshPath,
                    settings.propLevelFormat, i, settings.meshLevelList));
        }
        if(!settings.fastLoading)
        propMeshPyramid = currentMeshPyramid;

        btime::ptime test1  = btime::microsec_clock::local_time();
        setMeshPyramid();
        btime::ptime test2 = btime::microsec_clock::local_time();
        btime::time_duration testdiff = test2 - test1;
        std::cout << "Setting Mesh Pyramid Time: " << testdiff.total_milliseconds() << std::endl;

        int bufferPos = (i-startFrameNo)/nFrameStep;
        outputInfoPyramidBuffer[ bufferPos ] = std::move(outputInfoPyramid);
        outputPropPyramidBuffer[ bufferPos ] = std::move(outputPropPyramid);
        cout << "loading frame " << i << endl;
    }

    btime::ptime mst2 = btime::microsec_clock::local_time();
    btime::time_duration msdiff = mst2 - mst1;
    std::cout << "Loading Mesh Pyramid Time: " << msdiff.total_milliseconds() << std::endl;

}

MeshBufferReader::~MeshBufferReader(){
    delete[] pCurrentColorImageRGB;
}

void MeshBufferReader::setIntrinsicMatrix(double K[3][3])
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            KK[i][j] = K[i][j];
        }
    }
}

bool MeshBufferReader::setCurrentFrame(int curFrame)
{
    double bufferPos = (curFrame - startFrameNo) / nFrameStep;
    if(bufferPos + 1 <= m_nGoodFrames){
        currentFrameNo = curFrame;
        return true;
    }else
    return false;
}

bool MeshBufferReader::trackFrame(int nFrame, unsigned char* pColorImageRGB,
    TrackerOutputInfo** pOutputInfo)
{
    memcpy(pCurrentColorImageRGB, pColorImageRGB, 3*m_nWidth*m_nHeight);
    cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageRGB);
    tempColorImageRGB.convertTo(colorImage, cv::DataType<Vec3d>::type, 1./255);
    cv::split(colorImage, colorImageSplit);

    if(!setCurrentFrame(nFrame))
    return false;

    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    *pOutputInfo = &outputInfoPyramidBuffer[ bufferPos ][ nRenderingLevel ];

    return true;
}

void MeshBufferReader::setMeshPyramid()
{
    visibilityMaskPyramid.resize(m_nNumMeshLevels);
    outputInfoPyramid.resize(m_nNumMeshLevels);
    outputPropPyramid.resize(m_nNumMeshLevels);

    for(int i = 0; i < m_nNumMeshLevels; ++i)
    {
        int numVertices = currentMeshPyramid.levels[i].numVertices;
        visibilityMaskPyramid[i].resize(numVertices,true);

        vector<CoordinateType> proj2D;
        proj2D.resize(2); proj2D[0] = 0; proj2D[1] = 0;

        outputInfoPyramid[i].meshData = std::move(currentMeshPyramid.levels[i]);
        // outputInfoPyramid[i].meshDataGT = currentMeshPyramid.levels[i];
        // outputInfoPyramid[i].meshDataColorDiff = currentMeshPyramid.levels[i];
        outputInfoPyramid[i].nRenderLevel = i;
        outputInfoPyramid[i].meshProj.resize(numVertices, proj2D);
        // outputInfoPyramid[i].meshProjGT = outputInfoPyramid[i].meshProj;
        outputInfoPyramid[i].visibilityMask.resize(numVertices, true);
        memset(outputInfoPyramid[i].camPose, 0, 6*sizeof(double));

        UpdateRenderingData(outputInfoPyramid[i], KK, camPose, outputInfoPyramid[i].meshData);

        //////////////////////////// outputPropPyramid

        if(meshLoadingSettings.loadProp)
        {
            outputPropPyramid[i].meshData = std::move(propMeshPyramid.levels[i]);
            // outputPropPyramid[i].meshDataGT = propMeshPyramid.levels[i];
            // outputPropPyramid[i].meshDataColorDiff = propMeshPyramid.levels[i];
            outputPropPyramid[i].nRenderLevel = i;
            outputPropPyramid[i].meshProj.resize(numVertices, proj2D);
            // outputPropPyramid[i].meshProjGT = outputPropPyramid[i].meshProj;
            outputPropPyramid[i].visibilityMask.resize(numVertices, true);
            memset(outputPropPyramid[i].camPose, 0, 6*sizeof(double));

            UpdateRenderingData(outputPropPyramid[i], KK, camPose, outputPropPyramid[i].meshData);
        }

        // update the visibility of each vertex
        if(useVisibilityMask)
        {
            btime::ptime visTimeGL1 = btime::microsec_clock::local_time();
            UpdateVisibilityMaskGL(outputInfoPyramid[i], visibilityMaskPyramid[i], KK, camPose, m_nWidth, m_nHeight);
            if(meshLoadingSettings.loadProp)
            UpdateVisibilityMaskGL(outputPropPyramid[i], visibilityMaskPyramid[i], KK, camPose, m_nWidth, m_nHeight);
            btime::ptime visTimeGL2 = btime::microsec_clock::local_time();
            btime::time_duration visGLDiff = visTimeGL2 - visTimeGL1;
            std::cout << "visibility mask time: " << visGLDiff.total_milliseconds() << std::endl;
        }
    }
}

void MeshBufferReader::updateRenderingLevel(TrackerOutputInfo** pOutputInfoRendering,
    int nRenderLevel, bool renderType)
{
    cout << "changing rendering type" << endl;
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;

    if(renderType)
    *pOutputInfoRendering = &(outputPropPyramidBuffer[ bufferPos ][ nRenderLevel ]);
    else
    *pOutputInfoRendering = &(outputInfoPyramidBuffer[ bufferPos ][ nRenderLevel ]);

    nRenderingLevel = nRenderLevel;
}