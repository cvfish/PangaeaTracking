#include "main_engine/tracker/TrackingEngine.h"

MeshSequenceReader::MeshSequenceReader(MeshLoadingSettings& settings, int width,
    int height, double K[3][3], int startFrame, int numTrackingFrames):
    trackerInitialized(false)
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

    // visibilityFacesTest.resize(width*height);
    // // 5 reserved faces for each pixel, to make stuff a bit faster
    // for(int i = 0; i < width*height; ++i)
    // visibilityFacesTest[i].reserve(5);

    setIntrinsicMatrix(K);

    // load the first frame
    loadMesh(meshLoadingSettings.meshPath, meshLoadingSettings.meshFormat,
        currentFrameNo);
}

MeshSequenceReader::~MeshSequenceReader() {
    delete[] pCurrentColorImageRGB;
}

bool MeshSequenceReader::setCurrentFrame(int curFrame)
{
    if(currentFrameNo != curFrame)
    {
        currentFrameNo = curFrame;

        // changing new frame time
        TICK("setCurrentFrame");

        if(!loadMesh(meshLoadingSettings.meshPath,
                meshLoadingSettings.meshFormat,currentFrameNo))
        return false;

        TOCK("setCurrentFrame");
    }
    return true;
}

void MeshSequenceReader::setIntrinsicMatrix(double K[3][3])
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            KK[i][j] = K[i][j];
        }
    }
}

bool MeshSequenceReader::loadMesh(std::string& meshPath,
    std::string& meshFormat, int curFrame)
{
    char buffer[BUFFER_SIZE];
    std::stringstream meshFile;
    sprintf(buffer, meshFormat.c_str(), curFrame);
    meshFile << meshPath << buffer;
    if(!bfs::exists(meshFile.str()))
    {
        cout << "File not existing: " << meshFile.str() << endl;
        return false;
    }
    
    if(!trackerInitialized)
    PangaeaMeshIO::loadfromFile(meshFile.str(), currentMesh);
    else
    PangaeaMeshIO::updateFromFile(meshFile.str(), currentMesh);

    return true;
}

bool MeshSequenceReader::trackFrame(int nFrame, unsigned char* pColorImageRGB,
                                    TrackerOutputInfo** pOutputInfo)
{
    *pOutputInfo = &outputInfo;
    //
    if(!setCurrentFrame(nFrame))
    return false;

    // set up the color used later
    memcpy(pCurrentColorImageRGB, pColorImageRGB, 3*m_nWidth*m_nHeight);
    cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageRGB);
    tempColorImageRGB.convertTo(colorImage, cv::DataType<Vec3d>::type, 1./255);
    cv::split(colorImage, colorImageSplit);
    
    if(!trackerInitialized)
    trackerInitSetup(outputInfo);
    else
    trackerUpdate(outputInfo);

    return true;
}

void MeshSequenceReader::trackerInitSetup(TrackerOutputInfo& outputInfo)
{
    TICK("visualRenderingInit");

    outputInfo.meshData = currentMesh;
    outputInfo.meshDataGT = outputInfo.meshData;

    // get 2d projections
    double X,Y,Z;
    double u,v,w;
    vector<CoordinateType> proj2D, proj2DGT;
    proj2D.resize(2); proj2DGT.resize(2);
    for(int vertex = 0; vertex < currentMesh.numVertices; ++vertex)
    {

        X = currentMesh.vertices[vertex][0];
        Y = currentMesh.vertices[vertex][1];
        Z = currentMesh.vertices[vertex][2];

        if(KK[0][2] == 0) // this is orthographic camera
        {
           proj2D[0] = X; proj2D[1] = Y;
           proj2DGT[0] = X; proj2DGT[1] = Y;
        }
        else
        {
            u = KK[0][0] * X + KK[0][1] * Y + KK[0][2] * Z;
            v = KK[1][0] * X + KK[1][1] * Y + KK[1][2] * Z;
            w = KK[2][0] * X + KK[2][1] * Y + KK[2][2] * Z;

            if(w != 0)
            {
                u = u/w;
                v = v/w;
            }

            proj2D[0] = u; proj2D[1] = v;
            proj2DGT[0] = u; proj2DGT[1] = v;

        }

        outputInfo.meshProj.push_back(proj2D);
        outputInfo.meshProjGT.push_back(proj2DGT);

    }

    outputInfo.visibilityMask.resize(outputInfo.meshData.numVertices,true);
    // update the visiblity mask
    if(useVisibilityMask)
    {
        UpdateVisibilityMaskGL(outputInfo, visibilityMask, KK, camPose, m_nWidth, m_nHeight);
        //UpdateVisibilityMask(outputInfo, visibilityMask, m_nWidth, m_nHeight);
        outputInfo.meshDataColorDiff = outputInfo.meshData;
        UpdateColorDiff(outputInfo, visibilityMask, colorImageSplit);
    }

    // camera pose is always 0 in this case
    for(int i = 0; i < 6; ++i)
    outputInfo.camPose[i] = 0;

    trackerInitialized = true;

    TOCK("visualRenderingInit");

}

void MeshSequenceReader::trackerUpdate(TrackerOutputInfo& outputInfo)
{
    TICK("visualRenderingUpdate");
    
    UpdateRenderingData(outputInfo, KK, camPose, currentMesh);
    UpdateRenderingDataFast(outputInfo, KK, currentMesh);
    
    if(useVisibilityMask)
    {
        UpdateVisibilityMaskGL(outputInfo, visibilityMask, KK, camPose, m_nWidth, m_nHeight);
        //UpdateVisibilityMask(outputInfo, visibilityMask, m_nWidth, m_nHeight);
        UpdateColorDiff(outputInfo, visibilityMask, colorImageSplit);
    }

    TOCK("visualRenderingUpdate");
}