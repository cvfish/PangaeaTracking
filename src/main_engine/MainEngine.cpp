#include "main_engine/MainEngine.h"

#ifdef _MSC_VER
#include "third_party/msvc/Stopwatch.h"
#else
#include "third_party/Stopwatch.h"
#endif

MainEngine::MainEngine():
    m_nNumMeshLevels(1),
    pInputThread(NULL),
    inputFlag(false)
{
}

MainEngine::~MainEngine()
{
    SafeDeleteArray(m_pColorImageRGB);
    SafeDeleteArray(m_pColorImageRGBBuffer);

    delete m_pImageSourceEngine;
    delete m_pTrackingEngine;
}


void MainEngine::GetInput(int nFrame)
{
    if(nFrame <= m_NumTrackingFrames)
    {
        m_pImageSourceEngine->setCurrentFrame(nFrame);
        // m_pImageSourceEngine->setCurrentFrame(m_nStartFrame +
        //     (nFrame-m_nStartFrame)*m_nFrameStep);
        unsigned char* m_pColorImage = m_pImageSourceEngine->getColorImage();

        //TICK("BGR2RGB");

        for(int i = 0; i < m_nWidth * m_nHeight; ++i)
        {
            m_pColorImageRGBBuffer[3*i] = m_pColorImage[3*i+2];
            m_pColorImageRGBBuffer[3*i+1] = m_pColorImage[3*i+1];
            m_pColorImageRGBBuffer[3*i+2] = m_pColorImage[3*i];

        }

        //TOCK("BGR2RGB");

        inputFlag = true;
        //        return true;
    }
    else
    {
        inputFlag = false;
        //return false;
    }

}

void MainEngine::SetIntrinsicMatrix(double K[3][3])
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j= 0; j < 3; ++j)
        {
            KK[i][j] = K[i][j];
        }
    }
}

void MainEngine::SetupInputAndTracker()
{

    switch(imageSourceType)
    {
        // start from the first frame
        case ALLIMAGESBUFFER:
            m_pImageSourceEngine = new ImagesBufferReader(imageSourceSettings);
            break;
        case IMAGESEQUENCE:
            m_pImageSourceEngine = new ImageSequenceReader(imageSourceSettings);
            break;
    }

    m_nWidth = imageSourceSettings.width;
    m_nHeight = imageSourceSettings.height;
    m_nStartFrame = imageSourceSettings.startFrame;
    m_nFrameStep = imageSourceSettings.frameStep;
    m_NumTrackingFrames = imageSourceSettings.numFrames;

    m_nCurrentFrame = m_nStartFrame;

    // update camera parameters
    SetIntrinsicMatrix(imageSourceSettings.KK);
    trackerSettings.isOrthoCamera = imageSourceSettings.isOrthoCamera;

    switch(trackingType)
    {
        case ALLSHAPESBUFFER:
            m_pTrackingEngine = new ShapesBufferReader(shapeLoadingSettings,
                m_nWidth, m_nHeight, KK, m_nCurrentFrame, m_NumTrackingFrames);
            break;
        case SHAPESEQUENCE:
            m_pTrackingEngine = new ShapeSequenceReader(shapeLoadingSettings,
                m_nWidth, m_nHeight, KK, m_nCurrentFrame, m_NumTrackingFrames);
            break;
        case MESHSEQUENCE:
            m_pTrackingEngine = new MeshSequenceReader(meshLoadingSettings,
                m_nWidth, m_nHeight, KK, m_nCurrentFrame, m_NumTrackingFrames);
            break;
        case MESHPYRAMID:
            m_pTrackingEngine = new MeshPyramidReader(meshLoadingSettings,
                m_nWidth, m_nHeight, KK, m_nCurrentFrame, m_NumTrackingFrames);
            break;
        case  MESHBUFFER:
            m_pTrackingEngine = new MeshBufferReader(meshLoadingSettings,
                m_nWidth, m_nHeight, KK, m_nCurrentFrame, m_NumTrackingFrames);
            break;
#ifndef VIS_ONLY
        case DEFORMNRSFM:
            m_pTrackingEngine = new DeformNRSFMTracker(trackerSettings,
                m_nWidth, m_nHeight, KK, m_nCurrentFrame, m_NumTrackingFrames);
#endif
    }

    // allocate memory
    int nPoints = m_nWidth * m_nHeight;
    SafeAllocArrayType(m_pColorImageRGB, 3*nPoints, unsigned char);
    SafeAllocArrayType(m_pColorImageRGBBuffer, 3*nPoints, unsigned char);

    // read input image
    GetInput(m_nCurrentFrame);
    memcpy(m_pColorImageRGB, m_pColorImageRGBBuffer, m_nWidth * m_nHeight * 3);

    // load initial mesh
    switch(trackingType)
    {
        case DEFORMNRSFM:
        {
            // LoadInitialMesh(); // this is loading mesh created from saved shape results
            if(trackerSettings.loadMesh)
            LoadInitialMeshFromFile();
            else
            LoadInitialMeshUVD(); // this is loading mesh from uImage, vImage and dImage

            //m_pTrackingEngine->setInitialMesh(templateMesh);
            //m_pTrackingEngine->setInitialMesh(templateMeshPyramid.meshPyramid[0]);
            m_pTrackingEngine->setInitialMeshPyramid(templateMeshPyramid);

            m_nNumMeshLevels = templateMeshPyramid.numLevels;

            break;
        }

        case MESHPYRAMID:
        {
            m_nNumMeshLevels = meshLoadingSettings.meshLevelList.size();
            break;
        }
        case MESHBUFFER:
        {
            m_nNumMeshLevels = meshLoadingSettings.meshLevelList.size();
            break;
        }

    }

    // set up the visiblity mask of outputInfo to all true and
    // we will update it only in the case of DeformNRSFMTracker

    // m_nCurrentFrame++;
    // GetInput(m_nCurrentFrame);
    m_pTrackingEngine->trackFrame(m_nCurrentFrame, m_pColorImageRGB, &pOutputInfo);

    // for(int i = 0; i < 3; ++i)
    // center[i] = (*pOutputInfo).meshDataGT.center[i];
    for(int i = 0; i < 3; ++i)
    center[i] = (*pOutputInfo).meshData.center[i];

}

bool MainEngine::ProcessOneFrame(int nFrame)
{
    // read input
    // if(!GetInput(nFrame))
    // return false;


    // if(inputThreadGroup.size() > 0){
    //     inputThreadGroup.join_all();
    //     memcpy(m_pColorImageRGB, m_pColorImageRGBBuffer, m_nWidth * m_nHeight * 3);
    //     inputThreadGroup.remove_thread(pInputThread);
    //     pInputThread = inputThreadGroup.create_thread( boost::bind(&MainEngine::GetInput, this, nFrame) );
    // }
    // // for the first frame, we have to wait
    // else{
    //     pInputThread = inputThreadGroup.create_thread( boost::bind(&MainEngine::GetInput, this, nFrame) );
    //     inputThreadGroup.join_all();
    //     memcpy(m_pColorImageRGB, m_pColorImageRGBBuffer, m_nWidth * m_nHeight * 3);
    // }

    TICK("timePerFrame");

    // TICK("getInput");
    // if(pInputThread == NULL)
    // {
    //     pInputThread = new boost::thread(boost::bind(&MainEngine::GetInput, this, nFrame));
    //     pInputThread->join();
    //     memcpy(m_pColorImageRGB, m_pColorImageRGBBuffer, m_nWidth * m_nHeight * 3);
    // }
    // else
    // {
    //     pInputThread->join();
    //     memcpy(m_pColorImageRGB, m_pColorImageRGBBuffer, m_nWidth * m_nHeight * 3);
    //     delete pInputThread;
    //     pInputThread = new boost::thread(boost::bind(&MainEngine::GetInput, this, nFrame));
    // }
    // TOCK("getInput");

    TICK("getInput");
    GetInput(nFrame);
    TOCK("getInput");

    if(!inputFlag)
      {
        cout << "getting input failure" << endl;
        return false;
      }

    // do tracking
    TICK("tracking");
    if(!m_pTrackingEngine->trackFrame(nFrame, m_pColorImageRGB, &pOutputInfo))
    {
        cout << "tracking failed: " << endl;
        return false;
    }
    TOCK("tracking");

    TOCK("timePerFrame");

    return true;
}

bool MainEngine::ProcessNextFrame()
{
    // set the current frame to next frame
    // call ProcessOneFrame
    mutex.lock();

    bool flag = false;

    if(m_nCurrentFrame + m_nFrameStep <= m_NumTrackingFrames)
    {
        m_nCurrentFrame += m_nFrameStep;
        flag = ProcessOneFrame(m_nCurrentFrame);
        cout << "tracking finished" << endl;
    }

    mutex.unlock();

    return flag;
}

void MainEngine::Run()
{
    // retrieve new frame
    // process frame
    while(ProcessNextFrame())
    {
        // update number of frames processed
        // how much time elapsed
        Stopwatch::getInstance().printAll();
    }
}


// load mesh from uImage, vImage, dImage and maskImage
// only for the first frame
void MainEngine::LoadInitialMeshUVD()
{
    //
    cout << "load mesh from uvd images" << endl;

    DepthImageType uImage(m_nHeight,m_nWidth);
    DepthImageType vImage(m_nHeight,m_nWidth);
    DepthImageType dImage(m_nHeight,m_nWidth);
    InternalIntensityImageType maskImage;

    m_pImageSourceEngine->readUVDImage(uImage,vImage,dImage,maskImage);

    //     // we need to compute normals
    // specify the depth scale for the level we want to do optimization on
    if(!trackerSettings.useDepthPyramid)
    {
        PangaeaMeshIO::createMeshFromDepth(templateMesh, m_pColorImageRGB,
            uImage, vImage, dImage, maskImage, m_nHeight, m_nWidth,
            trackerSettings.depth2MeshScale);
        templateMeshPyramid = std::move(PangaeaMeshPyramid(templateMesh));
    }else
    {
        int numMeshLevels = trackerSettings.imagePyramidSamplingFactors.size();
        templateMeshPyramid.numLevels = numMeshLevels;
        templateMeshPyramid.levels.resize(numMeshLevels);
        templateMeshPyramid.meshPyramidVertexNum.resize(numMeshLevels);
        // in this case, the shape and image are subsampled using the same factor
        for(int i = 0; i < numMeshLevels; ++i)
        {
            // PangaeaMeshIO::createMeshFromDepth(templateMesh, m_pColorImageRGB,
            // uImage, vImage, dImage, maskImage, m_nHeight, m_nWidth,
            // 1.0/trackerSettings.imagePyramidSamplingFactors[i]);
            // templateMeshPyramid.levels[i] = std::move(templateMesh);
            // templateMesh.clear();

            InternalColorImageType colorImage;
            cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, m_pColorImageRGB);
            tempColorImageRGB.convertTo(colorImage, cv::DataType<Vec3d>::type, 1./255);

            int blurSize = trackerSettings.blurFilterSizes[i];
            if(blurSize > 0)
            cv::GaussianBlur(colorImage, colorImage, cv::Size(blurSize, blurSize), 3);

            PangaeaMeshIO::createMeshFromDepth(templateMesh, colorImage,
                uImage, vImage, dImage, maskImage, m_nHeight, m_nWidth,
                1.0/trackerSettings.imagePyramidSamplingFactors[i]);

            templateMeshPyramid.meshPyramidVertexNum[i] = templateMesh.numVertices;
            templateMeshPyramid.levels[i] = std::move(templateMesh);
            templateMesh.clear();
        }
    }

}

void MainEngine::LoadInitialMeshFromFile()
{

    if(trackerSettings.meshLevelFormat.empty())
    {
        // load mesh from obj, off or ply file
        PangaeaMeshIO::loadfromFile(trackerSettings.meshFile, templateMesh,
                                    trackerSettings.clockwise);
        templateMeshPyramid = std::move(PangaeaMeshPyramid(templateMesh));
    }else{
        templateMeshPyramid = std::move(
            PangaeaMeshPyramid(trackerSettings.meshLevelFormat,
                trackerSettings.meshVertexNum, trackerSettings.clockwise));
    }

}

// void MainEngine::ReadConfigurationFile(int argc, wxChar* argv[])
// void MainEngine::ReadConfigurationFile(int argc, char** argv)
void MainEngine::ReadConfigurationFile(int argc, char* argv[])
{

    // settings test
    // cv::FileStorage fs(wxString(argv[1]).ToStdString(), cv::FileStorage::READ);
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);

    // read settings
    std::string imageSource;
    std::string tracker;

    if(!fs["ImageSourceType"].empty())
    fs["ImageSourceType"] >> imageSource;
    imageSourceType = mapImageSourceType( imageSource );

    if(!fs["TrackingType"].empty())
    fs["TrackingType"] >> tracker;
    trackingType = mapTrackingType(tracker);

    if(!fs["ImageSourceSettings"].empty())
    fs["ImageSourceSettings"] >> imageSourceSettings;

    if(!fs["ShapeLoadingSettings"].empty())
    fs["ShapeLoadingSettings"] >> shapeLoadingSettings;

    if(!fs["MeshLoadingSettings"].empty())
    fs["MeshLoadingSettings"] >> meshLoadingSettings;

    if(!fs["TrackerSettings"].empty())
    fs["TrackerSettings"] >> trackerSettings;

    fs.release();

}
