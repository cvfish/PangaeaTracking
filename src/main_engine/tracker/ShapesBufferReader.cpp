#include "main_engine/tracker/TrackingEngine.h"

ShapesBufferReader::ShapesBufferReader(ShapeLoadingSettings& settings, int width,
    int height, double K[3][3], int startFrame, int numTrackingFrames)
{

    samplingScale = settings.shapeSamplingScale;

    m_hasGT = settings.hasGT;
    m_nWidth = width * settings.shapeSamplingScale;
    m_nHeight = height * settings.shapeSamplingScale;
    m_colMajor = settings.shapeColMajor;
    m_maskColMajor = settings.maskColMajor;

    startFrameNo = startFrame;
    currentFrameNo = startFrame;
    totalFrameNo = numTrackingFrames;

    shapePath = settings.resultsPath;
    shapeFormat = settings.shapeFormat;
    shapeFormatGT =  settings.shapeFormatGT;

    gtModelFile = settings.gtModelFile;
    solModelFile = settings.solModelFile;
    labelColorFile = settings.labelColorFile;

    useMask = settings.loadShapeMask;
    shapeMaskFile = settings.shapeMaskFile;

    m_nModelNum = settings.modelNum;

    nFrameStep = imageSourceSettings.frameStep;

    m_nGoodFrames = 0;

    // Initialization
    setIntrinsicMatrix(K);
    Initialization();
    readModelInfo();

}

ShapesBufferReader::~ShapesBufferReader()
{

    SafeDeleteArray(m_pTextureColors);

    SafeDeleteArray(m_pProjResultsBuffer);
    SafeDeleteArray(m_pTrackingResultsBuffer);
    SafeDeleteArray(m_pTrackingResultsNormal);

    if(m_hasGT)
    {
        SafeDeleteArray(m_pProjResultsBufferGT);
        SafeDeleteArray(m_pTrackingResultsBufferGT);
        SafeDeleteArray(m_pTrackingResultsNormalGT);
    }

}

void ShapesBufferReader::Initialization()
{

    isTextured = false;
    trackerInitialized = false;

    // initialize the center
    for(int i = 0; i < 3; ++i)
    {
        centerGT[i] = 0;
        centerEST[i] = 0;
    }

    int bufferSize = (totalFrameNo - startFrameNo)/nFrameStep + 1;
    SafeAllocArrayType(m_pTextureColors, 3 * m_nHeight * m_nWidth * bufferSize, CoordinateType);
    SafeAllocArrayType(m_pTrackingResultsBuffer, 3 * m_nHeight * m_nWidth * bufferSize, CoordinateType);
    SafeAllocArrayType(m_pProjResultsBuffer, 2 * m_nHeight * m_nWidth * bufferSize, CoordinateType);
    SafeAllocArrayType(m_pTrackingResultsNormal, 3 * m_nHeight* m_nWidth * bufferSize, CoordinateType);
    if(m_hasGT)
    {
        SafeAllocArrayType(m_pProjResultsBufferGT, 2 * m_nHeight * m_nWidth* bufferSize, CoordinateType);
        SafeAllocArrayType(m_pTrackingResultsBufferGT, 3 * m_nHeight * m_nWidth * bufferSize, CoordinateType);
        SafeAllocArrayType(m_pTrackingResultsNormalGT, 3 * m_nHeight * m_nWidth * bufferSize, CoordinateType);
    }
    else
    {
        m_pTrackingResultsBufferGT = m_pTrackingResultsBuffer;
        m_pProjResultsBufferGT = m_pProjResultsBuffer;
        m_pTrackingResultsNormalGT = m_pTrackingResultsNormal;
    }

    int numPnts = m_nWidth * m_nHeight;
    // load mask
    if(useMask)
    {
        maskImage.resize(numPnts);
        std::stringstream maskImagePath;
        maskImagePath << shapePath << shapeMaskFile;
        if(bfs::exists(maskImagePath.str()))
        {
            ifstream maskFileFID(maskImagePath.str().c_str(), std::ios::binary);
            double temp;
            for(unsigned int j = 0; j < numPnts; ++j)
            {
                maskFileFID.read(reinterpret_cast<char*>(&temp),8);
                maskImage[j] = temp;
            }
            maskFileFID.close();
        }
        else
        {
            cerr << maskImagePath.str() << " does not exist! " << endl;
        }
    }
    else
    {
        maskImage.resize(numPnts);
        for(unsigned int j = 0; j < numPnts; ++j)
        {
            maskImage[j] = 1;
        }
    }
    // if mask is colMajor, convert it to row major and use it afterwards
    if(m_maskColMajor)
    {
        vector<double> tempMaskImage;
        tempMaskImage.resize(numPnts);
        for(int i = 0; i < m_nWidth; ++i)
        {
            for(int j = 0; j < m_nHeight; ++j)
            {
                tempMaskImage[j*m_nWidth+i] = maskImage[j+i*m_nHeight];
            }
        }
        maskImage = tempMaskImage;
    }

    // load shapes
    loadShapesBuffer(false, shapePath, shapeFormat, startFrameNo, totalFrameNo); // load reconstruction result
    if(m_hasGT)
    {
        loadShapesBuffer(true, shapePath, shapeFormatGT, startFrameNo, totalFrameNo); // load reconstruction result
    }
    else // load grount truth points
    {
        centerGT[0] = centerEST[0];
        centerGT[1] = centerEST[1];
        centerGT[2] = centerEST[2];
    }

    // project 3d shapes to 2d projection points on images
    projShapes(totalFrameNo);

    unsigned int radius = 1;
    computeNormals(false, radius);
    if(m_hasGT)
    computeNormals(true, radius);
}

void ShapesBufferReader::loadShapesBuffer(bool isGT, std::string& shapePath,
    std::string& shapeFormat, int startFrame, int numTrackingFrames)
{
    int numPnts = m_nHeight * m_nWidth;
    int uselessPntsNum = 0;

    char buffer[BUFFER_SIZE];

    CoordinateType* pTrackingResults;
    double* pCenter;

    if(isGT)
    {
        pTrackingResults = m_pTrackingResultsBufferGT;
        pCenter = centerGT;
    }
    else
    {
        pTrackingResults = m_pTrackingResultsBuffer;
        pCenter = centerEST;
    }

    for(int i = startFrame; i <=  numTrackingFrames; i = i + nFrameStep)
    {
        std::stringstream shapeFilePath;
        sprintf(buffer, shapeFormat.c_str(), i);
        shapeFilePath << shapePath << buffer;
        //memset(&buffer[0], 0, sizeof(buffer));

        std::cout << shapeFilePath.str() << std::endl;

        if(!bfs::exists(shapeFilePath.str()))
        {
            cout << shapeFilePath.str() << " does not exist" << endl;
            break;
        }

        ++m_nGoodFrames;

        std::ifstream shapeFile(shapeFilePath.str().c_str());

        // int frame = i - startFrame;
        int bufferPos = (i - startFrame) / nFrameStep;
        int pntsInd = 0;
        int saveInd = 0;

        int width = m_nWidth;
        int height = m_nHeight;
        if(m_colMajor)
        {
            width = m_nHeight;
            height = m_nWidth;
        }

        for(unsigned int j = 0; j < height; ++j)
        {
            for(unsigned int k = 0; k < width; ++k)
            {

                if(m_colMajor)
                saveInd = j + k * m_nWidth;  // In this case, k is row number really
                else
                saveInd = k + j * m_nWidth;  // In this case, j is row number

                if(useMask)
                {
                    if(maskImage[saveInd] > 0)
                    {
                        shapeFile >> pTrackingResults[ bufferPos*3*numPnts + 3*saveInd ];
                        shapeFile >> pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 1 ];
                        shapeFile >> pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 2 ];
                    }
                    else
                    {
                        if(bufferPos == 0)
                        uselessPntsNum++;
                        continue;
                    }
                }
                else
                {
                    shapeFile >> pTrackingResults[ bufferPos*3*numPnts + 3*saveInd ];
                    shapeFile >> pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 1 ];
                    shapeFile >> pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 2 ];
                    // if(    pTrackingResults[ bufferPos*3*numPnts + 3*saveInd ] == 0
                    //     && pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 1 ] == 0
                    //     && pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 2 ] == 0)
                    if(pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 2 ] == 0)
                    {
                        maskImage[saveInd] = 0;
                        if(bufferPos == 0)
                        uselessPntsNum++;
                        continue;
                    }
                }
                if(bufferPos == 0)
                {
                    pCenter[0] = pCenter[0] + pTrackingResults[ bufferPos*3*numPnts + 3*saveInd ];
                    pCenter[1] = pCenter[1] + pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 1 ];
                    pCenter[2] = pCenter[2] + pTrackingResults[ bufferPos*3*numPnts + 3*saveInd + 2 ];
                }
            }
        }

        shapeFile.close();

    }


    int leftPntsNum = numPnts - uselessPntsNum;
    pCenter[0] = pCenter[0]/leftPntsNum;
    pCenter[1] = pCenter[1]/leftPntsNum;
    pCenter[2] = pCenter[2]/leftPntsNum;

}

void ShapesBufferReader::projShapes(int numTrackingFrames)
{
    int numPnts = m_nHeight * m_nWidth;
    int pntsInd = 0;
    int saveInd = 0;
    double X,Y,Z;
    double u,v,w;
    for(int i = startFrameNo; i <= numTrackingFrames; i = i + nFrameStep)
    {
        for(int j = 0; j < m_nHeight; ++j)
        {
            for(int k = 0; k < m_nWidth; ++k)
            {

                pntsInd = k + j * m_nWidth;

                int bufferPos = (i-startFrameNo)/nFrameStep;
                saveInd = bufferPos * numPnts + k + j * m_nWidth;

                if((useMask && maskImage[pntsInd] > 0) || !useMask)
                {
                    // do projection
                    X = m_pTrackingResultsBuffer[ 3*saveInd ];
                    Y = m_pTrackingResultsBuffer[ 3*saveInd + 1];
                    Z = m_pTrackingResultsBuffer[ 3*saveInd + 2];

                    if(KK[0][2] == 0)
                    {
                        m_pProjResultsBuffer[ 2*saveInd ] = X;
                        m_pProjResultsBuffer[ 2*saveInd + 1 ] = Y;
                    }
                    else
                    {
                        u = KK[0][0] * X + KK[0][1] * Y + KK[0][2] * Z;
                        v = KK[1][0] * X + KK[1][1] * Y + KK[1][2] * Z;
                        w = KK[2][0] * X + KK[2][1] * Y + KK[2][2] * Z;

                        if(w != 0)
                        {
                            m_pProjResultsBuffer[ 2*saveInd ] = u/w;
                            m_pProjResultsBuffer[ 2*saveInd + 1 ] = v/w;
                        }
                    }

                    if(m_hasGT)
                    {
                        X = m_pTrackingResultsBufferGT[ 3*saveInd ];
                        Y = m_pTrackingResultsBufferGT[ 3*saveInd + 1];
                        Z = m_pTrackingResultsBufferGT[ 3*saveInd + 2];

                        if(KK[0][2] == 0) //  we have orthographic camaera in this case
                        {
                            m_pProjResultsBufferGT[ 2*saveInd ] = X;
                            m_pProjResultsBufferGT[ 2*saveInd + 1 ] = Y;
                        }
                        else
                        {

                            u = KK[0][0] * X + KK[0][1] * Y + KK[0][2] * Z;
                            v = KK[1][0] * X + KK[1][1] * Y + KK[1][2] * Z;
                            w = KK[2][0] * X + KK[2][1] * Y + KK[2][2] * Z;

                            if(w != 0)
                            {
                                m_pProjResultsBufferGT[ 2*saveInd ] = u/w;
                                m_pProjResultsBufferGT[ 2*saveInd + 1 ] = v/w;
                            }
                        }

                    }
                    else
                    {
                        m_pProjResultsBufferGT[ 2*saveInd ] =     m_pProjResultsBuffer[ 2*saveInd ];
                        m_pProjResultsBufferGT[ 2*saveInd + 1 ] = m_pProjResultsBuffer[ 2*saveInd + 1 ];
                    }

                }
            }
        }

    }

}

bool ShapesBufferReader::setCurrentFrame(int curFrame)
{
    // compare bufferPos of curFrame with m_nGoodFrames
    double bufferPos = (curFrame - startFrameNo) / nFrameStep;
    if(bufferPos + 1 <= m_nGoodFrames){
        currentFrameNo = curFrame;
        return true;
    }else
    return false;
}

void ShapesBufferReader::setIntrinsicMatrix(double K[3][3])
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        KK[i][j] = K[i][j];
    }
}

void ShapesBufferReader::getGTCenter(double* pGTCenter)
{
    pGTCenter[0] = centerGT[0];
    pGTCenter[1] = centerGT[1];
    pGTCenter[2] = centerGT[2];
}

void ShapesBufferReader::getESTCenter(double* pESTCenter)
{
    pESTCenter[0] = centerEST[0];
    pESTCenter[1] = centerEST[1];
    pESTCenter[2] = centerEST[2];
}

CoordinateType* ShapesBufferReader::getTrackingResult()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    return &m_pTrackingResultsBuffer[ 3*m_nHeight*m_nWidth*bufferPos ];
}

CoordinateType* ShapesBufferReader::getGTTrackingResult()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    return &m_pTrackingResultsNormalGT[ 3*m_nHeight*m_nWidth*bufferPos ];
}

CoordinateType* ShapesBufferReader::getProjResult()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    cout << "bufferPos: " << bufferPos << endl;
    return &m_pProjResultsBuffer[ 2*m_nHeight*m_nWidth*bufferPos ];
}

CoordinateType* ShapesBufferReader::getGTProjResult()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    return &m_pProjResultsBufferGT[ 2*m_nHeight*m_nWidth*bufferPos ];
}

CoordinateType* ShapesBufferReader::getNormals()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    return &m_pTrackingResultsNormal[ 3*m_nHeight*m_nWidth*bufferPos ];
}

CoordinateType* ShapesBufferReader::getGTNormals()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    return &m_pTrackingResultsNormalGT[ 3*m_nHeight*m_nWidth*bufferPos ];
}

CoordinateType* ShapesBufferReader::getTextureColors()
{
    return m_pTextureColors;
}

void ShapesBufferReader::computeNormals(bool isGT, unsigned int radius)
{
    CoordinateType* pResults;
    CoordinateType* pNormals;
    if(isGT)
    {
        pResults = m_pTrackingResultsBufferGT;
        pNormals = m_pTrackingResultsNormalGT;
    }
    else
    {
        pResults = m_pTrackingResultsBuffer;
        pNormals = m_pTrackingResultsNormal;
    }

    int nopoints = m_nHeight * m_nWidth;

    for(int i = startFrameNo; i <= totalFrameNo; i = i + nFrameStep)
    {
        int bufferPos = (i-startFrameNo)/nFrameStep;
        for(int j = 0; j < m_nHeight - radius; ++j)
        {
            for(int i = 0; i < m_nWidth - radius; ++ i)
            compnorm(&pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+i),
                &pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+radius+i),
                &pResults[0]+bufferPos*3*nopoints+3*((j+radius)*m_nWidth+i),
                &pNormals[0]+3*(bufferPos*nopoints+j*m_nWidth+i), true);

            for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
            compnorm(&pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+i-radius),
                &pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+i),
                &pResults[0]+bufferPos*3*nopoints+3*((j+radius)*m_nWidth+i),
                &pNormals[0]+3*(bufferPos*nopoints+j*m_nWidth+i), true);
        }

        for (int j = m_nHeight-radius; j < m_nHeight; ++j)
        {
            for(int i = 0; i < m_nWidth-radius; ++i)
            compnorm(&pResults[0]+bufferPos*3*nopoints+3*((j-radius)*m_nWidth+i),
                &pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+radius+i),
                &pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+i),
                &pNormals[0]+3*(bufferPos*nopoints+j*m_nWidth+i), true);

            for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
            compnorm(&pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth+i),
                &pResults[0]+bufferPos*3*nopoints+3*(j*m_nWidth-radius+i),
                &pResults[0]+bufferPos*3*nopoints+3*((j-radius)*m_nWidth+i),
                &pNormals[0]+3*(bufferPos*nopoints+j*m_nWidth+i), true);
        }
    }

}

void ShapesBufferReader::readModelInfo()
{
    m_vModelColors.resize(3*m_nModelNum);
    int nPoints = m_nWidth * m_nHeight;
    m_vModelGT.resize(nPoints);
    m_vModelEST.resize(nPoints);

    // read model labeling
    std::stringstream modelFilePathGT;
    modelFilePathGT << shapePath << gtModelFile;
    std::ifstream modelFileGT(modelFilePathGT.str().c_str());
    for(int i = 0; i < nPoints; ++i)
    modelFileGT >> m_vModelGT[i];
    modelFileGT.close();

    std::stringstream modelFilePathEST;
    modelFilePathEST << shapePath << solModelFile;
    std::ifstream modelFileEST(modelFilePathEST.str().c_str());
    for(int i = 0; i < nPoints; ++i)
    modelFileEST >> m_vModelEST[i];
    modelFileEST.close();

    // read model colors
    std::stringstream modelColorFilePath;
    modelColorFilePath << shapePath << labelColorFile;
    std::ifstream modelColor(modelColorFilePath.str().c_str());
    for(int i = 0; i < m_vModelColors.size(); ++i)
    modelColor >> m_vModelColors[i];
    modelColor.close();

}

bool ShapesBufferReader::trackFrame(int nFrame, unsigned char* pColorImageRGB,
    TrackerOutputInfo** pOutputInfo)
{
    *pOutputInfo = &outputInfo;

    if(currentFrameNo == startFrameNo && isTextured == false)
    {
        setTextureColors(pColorImageRGB);
        isTextured = true;
    }

    if(!setCurrentFrame(nFrame))
    return false;

    if(!trackerInitialized)
    trackerInitSetup(outputInfo);
    else
    trackerUpdate(outputInfo);

    return true;
}

void ShapesBufferReader::setTextureColors(unsigned char* pColorImageRGB)
{
    for(int i = 0; i < m_nHeight; ++i)
    {
        for(int j = 0; j < m_nWidth; ++j)
        {
            int offset = 3*(i*m_nWidth + j);
            int width = (1/samplingScale) * m_nWidth;
            int offsetImage = 3*(i * 1/samplingScale * width + j/samplingScale);
            m_pTextureColors[ offset ] =    pColorImageRGB[ offsetImage ] / 255.0;
            m_pTextureColors[ offset + 1] = pColorImageRGB[ offsetImage + 1] / 255.0;
            m_pTextureColors[ offset + 2] = pColorImageRGB[ offsetImage + 2] / 255.0;

        }
    }
}


void ShapesBufferReader::trackerInitSetup(TrackerOutputInfo& outputInfo)
{
    TICK("setupShapeBufferRendering");

    // copy over initial data
    PangaeaMeshIO::createMeshFromPoints(outputInfo.meshData, m_nHeight, m_nWidth,
        getTrackingResult(), getNormals(), getTextureColors(),
        maskImage, m_vModelEST, m_vModelColors);

    if(m_hasGT)
    PangaeaMeshIO::createMeshFromPoints(outputInfo.meshDataGT, m_nHeight, m_nWidth,
        getGTTrackingResult(), getGTNormals(), getTextureColors(),
        maskImage, m_vModelGT, m_vModelColors);
    else
    {
        outputInfo.meshDataGT = outputInfo.meshData;
    }

    outputInfo.meshDataColorDiff = outputInfo.meshData;

    // get 2d projections
    vector<CoordinateType> proj2D, proj2DGT;
    proj2D.resize(2); proj2DGT.resize(2);
    int bufferPos = (currentFrameNo - startFrameNo)/nFrameStep;
    int offset = 2 * m_nHeight * m_nWidth * bufferPos;
    for(int i = 0; i < m_nHeight*m_nWidth; ++i)
    {

        if(maskImage[i])
        {
            proj2D[0] =   m_pProjResultsBuffer[offset + 2*i];
            proj2D[1] =   m_pProjResultsBuffer[offset + 2*i + 1];
            proj2DGT[0] = m_pProjResultsBufferGT[offset + 2*i];
            proj2DGT[1] = m_pProjResultsBufferGT[offset + 2*i + 1];

            outputInfo.meshProj.push_back(proj2D);
            outputInfo.meshProjGT.push_back(proj2DGT);

        }
    }

    outputInfo.visibilityMask.resize(outputInfo.meshData.numVertices,true);

    // camera pose is always 0 in this case
    for(int i = 0; i < 6; ++i)
    outputInfo.camPose[i] = 0;

    trackerInitialized = true;

    TOCK("setupShapeBufferRendering");


}

void ShapesBufferReader::trackerUpdate(TrackerOutputInfo& outputInfo)
{
    TICK("shapeBufferNewFrame");

    // copy over initial data
    PangaeaMeshIO::updateMesh(outputInfo.meshData, m_nHeight, m_nWidth,
        getTrackingResult(), getNormals(), getTextureColors(),
        maskImage, m_vModelEST, m_vModelColors);

    PangaeaMeshIO::updateMesh(outputInfo.meshDataGT, m_nHeight, m_nWidth,
        getGTTrackingResult(), getGTNormals(), getTextureColors(),
        maskImage, m_vModelGT, m_vModelColors);


    // get 2d projections
    int pntsId = 0;
    int bufferPos = (currentFrameNo - startFrameNo)/nFrameStep;
    int offset = 2 * m_nHeight * m_nWidth * bufferPos;
    for(int i = 0; i < m_nHeight*m_nWidth; ++i)
    {
        if(maskImage[i])
        {
            outputInfo.meshProj[pntsId][0] = m_pProjResultsBuffer[offset + 2*i];
            outputInfo.meshProj[pntsId][1] = m_pProjResultsBuffer[offset + 2*i + 1];
            outputInfo.meshProjGT[pntsId][0] = m_pProjResultsBufferGT[offset + 2*i];
            outputInfo.meshProjGT[pntsId][1] = m_pProjResultsBufferGT[offset + 2*i + 1];
            ++pntsId;
        }
    }

    // camera pose is always 0 in this case
    for(int i = 0; i < 6; ++i)
    outputInfo.camPose[i] = 0;

    TOCK("shapeBufferNewFrame");

}
