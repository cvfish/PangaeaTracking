#include "main_engine/tracker/TrackingEngine.h"

ShapeSequenceReader::ShapeSequenceReader(ShapeLoadingSettings& settings, int
    width, int height, double K[3][3], int startFrame, int numTrackingFrames)
{
    samplingScale = settings.shapeSamplingScale;

    m_hasGT = settings.hasGT;
    m_nWidth = width * settings.shapeSamplingScale;
    m_nHeight = height * settings.shapeSamplingScale;
    m_colMajor = settings.shapeColMajor;

    startFrameNo = startFrame;
    currentFrameNo = startFrame;
    totalFrameNo = numTrackingFrames;

    shapePath = settings.resultsPath;
    shapeFormat = settings.shapeFormat;
    shapeFormatGT = settings.shapeFormatGT;

    gtModelFile = settings.gtModelFile;
    solModelFile = settings.solModelFile;
    labelColorFile = settings.labelColorFile;

    useMask = settings.loadShapeMask;
    shapeMaskFile = settings.shapeMaskFile;

    m_nModelNum = settings.modelNum;

    // Initialization
    setIntrinsicMatrix(K);
    Initialization();
    readModelInfo();

}

ShapeSequenceReader::~ShapeSequenceReader()
{
    SafeDeleteArray(m_pTextureColors);

    SafeDeleteArray(m_pCurProjResult);
    SafeDeleteArray(m_pCurTrackingResult);
    SafeDeleteArray(m_pCurTrackingResultNormal);

    if(m_hasGT)
    {
        SafeDeleteArray(m_pCurProjResultGT);
        SafeDeleteArray(m_pCurTrackingResultGT);
        SafeDeleteArray(m_pCurTrackingResultNormalGT);
    }
}

void ShapeSequenceReader::Initialization()
{
    isTextured = false;
    trackerInitialized = false;

    // initialize the center
    for(int i = 0; i < 3; ++i)
    {
        centerGT[i] = 0;
        centerEST[i] = 0;
    }

    SafeAllocArrayType(m_pTextureColors, 3*m_nHeight*m_nWidth, CoordinateType);
    SafeAllocArrayType(m_pCurTrackingResult, 3*m_nHeight*m_nWidth, CoordinateType);
    SafeAllocArrayType(m_pCurProjResult, 2*m_nHeight*m_nWidth, CoordinateType);
    SafeAllocArrayType(m_pCurTrackingResultNormal, 3*m_nHeight*m_nWidth, CoordinateType);

    if(m_hasGT)
    {
        SafeAllocArrayType(m_pCurTrackingResultGT, 3*m_nHeight*m_nWidth, CoordinateType);
        SafeAllocArrayType(m_pCurProjResultGT, 2*m_nHeight*m_nWidth, CoordinateType);
        SafeAllocArrayType(m_pCurTrackingResultNormalGT, 3*m_nHeight*m_nWidth, CoordinateType);
    }
    else
    {
        m_pCurTrackingResultGT = m_pCurTrackingResult;
        m_pCurProjResultGT = m_pCurProjResult;
        m_pCurTrackingResultNormalGT = m_pCurTrackingResultNormal;
    }

    // load mask
    int numPnts = m_nWidth*m_nHeight;
    if(useMask)        // column major mask
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
    //if mask is colMajor, convert it to row major and use it afterwards
    if(m_colMajor)
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

    loadShape(false, shapePath, shapeFormat, currentFrameNo);
    if(m_hasGT)
    {
        loadShape(true, shapePath, shapeFormatGT, currentFrameNo);
    }
    else
    {
        centerGT[0] = centerEST[0];
        centerGT[1] = centerEST[1];
        centerGT[2] = centerEST[2];
    }

    // project 3d shapes to 2d projections
    projShape();

    unsigned int radius = 1;
    computeNormal(false, radius);
    if(m_hasGT)
    computeNormal(true, radius);

}

bool ShapeSequenceReader::loadShape(bool isGT, std::string& shapePath,
    std::string& shapeFormat, int curFrame)
{
    int numPnts = m_nHeight * m_nWidth;
    int uselessPntsNum = 0;

    char buffer[BUFFER_SIZE];

    CoordinateType* pCurTrackingResult;
    double* pCenter;

    if(isGT)
    {
        pCurTrackingResult = m_pCurTrackingResultGT;
        pCenter = centerGT;
    }
    else
    {
        pCurTrackingResult = m_pCurTrackingResult;
        pCenter = centerEST;
    }

    std::stringstream shapeFilePath;
    sprintf(buffer, shapeFormat.c_str(), curFrame);
    shapeFilePath << shapePath << buffer;
    //memset(&buffer[0], 0, sizeof(buffer));

    if(!bfs::exists(shapeFilePath.str()))
    {
        cout << shapeFilePath.str() << " does not exist" << endl;
        return false;
    }
    
    std::ifstream shapeFile(shapeFilePath.str().c_str());
        
    int frame = curFrame - startFrameNo;
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
            saveInd = j + k * m_nWidth;
            else
            saveInd = k + j * m_nWidth;

            if(useMask)
            {
                if(maskImage[saveInd] > 0)
                {
                    shapeFile >> pCurTrackingResult[ 3*saveInd ];
                    shapeFile >> pCurTrackingResult[ 3*saveInd + 1];
                    shapeFile >> pCurTrackingResult[ 3*saveInd + 2];
                }
                else
                {
                    if(frame == 0)
                    uselessPntsNum++;
                    continue;
                }
            }
            else
            {
                shapeFile >> pCurTrackingResult[ 3*saveInd ];
                shapeFile >> pCurTrackingResult[ 3*saveInd + 1];
                shapeFile >> pCurTrackingResult[ 3*saveInd + 2];
                // if(    pCurTrackingResult[ 3*saveInd ] == 0
                //     && pCurTrackingResult[ 3*saveInd + 1 ] == 0
                //     && pCurTrackingResult[ 3*saveInd + 2 ] == 0)
                if(pCurTrackingResult[ 3*saveInd + 2 ] == 0)
                {
                    maskImage[saveInd] = 0;
                    if(frame == 0)
                    {
                        uselessPntsNum++;
                    }
                    continue;
                }
            }

            if(frame == 0)
            {
                pCenter[0] = pCenter[0] + pCurTrackingResult[ 3*saveInd ];
                pCenter[1] = pCenter[1] + pCurTrackingResult[ 3*saveInd + 1 ];
                pCenter[2] = pCenter[2] + pCurTrackingResult[ 3*saveInd + 2 ];
            }
        }
    }

    if(frame == 0)
    {
        int leftPntsNum = numPnts - uselessPntsNum;
        pCenter[0] = pCenter[0]/leftPntsNum;
        pCenter[1] = pCenter[1]/leftPntsNum;
        pCenter[2] = pCenter[2]/leftPntsNum;
    }

    shapeFile.close();

    return true;
}

void ShapeSequenceReader::projShape()
{
    int pntsInd = 0;
    int saveInd = 0;
    double X,Y,Z;
    double u,v,w;
    for(int j = 0; j < m_nHeight; ++j)
    {
        for(int k = 0; k < m_nWidth; ++k)
        {

            saveInd = k + j * m_nWidth;

            if((useMask && maskImage[saveInd] > 0) || !useMask)
            {
                // do projection
                X = m_pCurTrackingResult[ 3*saveInd ];
                Y = m_pCurTrackingResult[ 3*saveInd + 1];
                Z = m_pCurTrackingResult[ 3*saveInd + 2];

                if(KK[0][2] == 0) //  we have orthographic camaera in this case
                {
                    m_pCurProjResult[ 2*saveInd ] = X;
                    m_pCurProjResult[ 2*saveInd + 1 ] = Y;
                }
                else
                {
                    u = KK[0][0] * X + KK[0][1] * Y + KK[0][2] * Z;
                    v = KK[1][0] * X + KK[1][1] * Y + KK[1][2] * Z;
                    w = KK[2][0] * X + KK[2][1] * Y + KK[2][2] * Z;   
                    if(w != 0)
                    {
                        m_pCurProjResult[ 2*saveInd ] = u/w;
                        m_pCurProjResult[ 2*saveInd + 1 ] = v/w;
                    }
                }

                if(m_hasGT)
                {
                    X = m_pCurTrackingResultGT[ 3*saveInd ];
                    Y = m_pCurTrackingResultGT[ 3*saveInd + 1];
                    Z = m_pCurTrackingResultGT[ 3*saveInd + 2];
                    
                    if(KK[0][2] == 0) //  we have orthographic camaera in this case
                    {
                        m_pCurProjResult[ 2*saveInd ] = X;
                        m_pCurProjResult[ 2*saveInd + 1 ] = Y;
                    }
                    else
                    {

                        u = KK[0][0] * X + KK[0][1] * Y + KK[0][2] * Z;
                        v = KK[1][0] * X + KK[1][1] * Y + KK[1][2] * Z;
                        w = KK[2][0] * X + KK[2][1] * Y + KK[2][2] * Z;

                        if(w != 0)
                        {
                            m_pCurProjResultGT[ 2*saveInd ] = u/w;
                            m_pCurProjResultGT[ 2*saveInd + 1 ] = v/w;
                        }
                    }

                }
                else
                {
                    m_pCurProjResultGT[ 2*saveInd ] = m_pCurProjResult[ 2*saveInd ];
                    m_pCurProjResultGT[ 2*saveInd + 1] = m_pCurProjResult[ 2*saveInd + 1 ];
                }

            }

        }

    }

}

bool ShapeSequenceReader::setCurrentFrame(int curFrame)
{
    if(currentFrameNo != curFrame)
    {
        currentFrameNo = curFrame;

        // changing new frame time
        btime::ptime mst1 = btime::microsec_clock::local_time();

        if(!loadShape(false, shapePath, shapeFormat, currentFrameNo))
        return false;
        
        projShape();
        unsigned int radius = 1;
        computeNormal(false, radius);
        if(m_hasGT)
        {
            loadShape(true, shapePath, shapeFormatGT, currentFrameNo);
            computeNormal(true, radius);
        }

        btime::ptime mst2 = btime::microsec_clock::local_time();
        btime::time_duration msdiff = mst2 - mst1;
        std::cout << "Change New Frame Time: " << msdiff.total_milliseconds() << std::endl;
    }
    return true;
}

void ShapeSequenceReader::setIntrinsicMatrix(double K[3][3])
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        KK[i][j] = K[i][j];
    }
}

void ShapeSequenceReader::computeNormal(bool isGT, unsigned int radius)
{
    CoordinateType* pResults;
    CoordinateType* pNormals;
    if(isGT)
    {
        pResults = m_pCurTrackingResultGT;
        pNormals = m_pCurTrackingResultNormalGT;
    }
    else
    {
        pResults = m_pCurTrackingResult;
        pNormals = m_pCurTrackingResultNormal;
    }

    for(int j = 0; j < m_nHeight - radius; ++j)
    {
        for(int i = 0; i < m_nWidth - radius; ++ i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i),&pResults[0]+3*(j*m_nWidth+radius+i),
            &pResults[0]+3*((j+radius)*m_nWidth+i),&pNormals[0]+3*(j*m_nWidth+i),1);

        for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i-radius), &pResults[0]+3*(j*m_nWidth+i),
            &pResults[0]+3*((j+radius)*m_nWidth+i),&pNormals[0]+3*(j*m_nWidth+i),1);
    }

    for (int j = m_nHeight-radius; j < m_nHeight; ++j)
    {
        for(int i = 0; i < m_nWidth-radius; ++i)
        compnorm(&pResults[0]+3*((j-radius)*m_nWidth+i), &pResults[0]+3*(j*m_nWidth+radius+i),
            &pResults[0]+3*(j*m_nWidth+i), &pNormals[0]+3*(j*m_nWidth+i),1);

        for(int i = m_nWidth-radius; i !=m_nWidth; ++i)
        compnorm(&pResults[0]+3*(j*m_nWidth+i), &pResults[0]+3*(j*m_nWidth-radius+i),
            &pResults[0]+3*((j-radius)*m_nWidth+i), &pNormals[0]+3*(j*m_nWidth+i),1);
    }

}

void ShapeSequenceReader::getGTCenter(double* pGTCenter)
{
    pGTCenter[0] = centerGT[0];
    pGTCenter[1] = centerGT[1];
    pGTCenter[2] = centerGT[2];
}

void ShapeSequenceReader::getESTCenter(double* pESTCenter)
{
    pESTCenter[0] = centerEST[0];
    pESTCenter[1] = centerEST[1];
    pESTCenter[2] = centerEST[2];
}

CoordinateType* ShapeSequenceReader::getTrackingResult()
{
    return m_pCurTrackingResult;
}

CoordinateType* ShapeSequenceReader::getGTTrackingResult()
{
    return m_pCurTrackingResultGT;
}

CoordinateType* ShapeSequenceReader::getProjResult()
{
    return m_pCurProjResult;
}

CoordinateType* ShapeSequenceReader::getGTProjResult()
{
    return m_pCurProjResultGT;
}

CoordinateType* ShapeSequenceReader::getNormals()
{
    return m_pCurTrackingResultNormal;
}

CoordinateType* ShapeSequenceReader::getGTNormals()
{
    return m_pCurTrackingResultNormalGT;
}

CoordinateType* ShapeSequenceReader::getTextureColors()
{
    return m_pTextureColors;
}

// In this case, pColorImageRGB is not necessary
bool ShapeSequenceReader::trackFrame(int nFrame, unsigned char* pColorImageRGB,
    TrackerOutputInfo** pOutputInfo)
{
    *pOutputInfo= &outputInfo;
    
    // if this is the first frame, read the coolors of the image
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

void ShapeSequenceReader::setTextureColors(unsigned char* pColorImageRGB)
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

// just loading the first frame
void ShapeSequenceReader::trackerInitSetup(TrackerOutputInfo& outputInfo)
{
    btime::ptime mst1 = btime::microsec_clock::local_time();

    // copy over data
    PangaeaMeshIO::createMeshFromPoints(outputInfo.meshData, m_nHeight, m_nWidth,
        getTrackingResult(), getNormals(), getTextureColors(),
        maskImage, m_vModelEST, m_vModelColors);

    if(m_hasGT)
    PangaeaMeshIO::createMeshFromPoints(outputInfo.meshDataGT, m_nHeight, m_nWidth,
        getGTTrackingResult(), getGTNormals(), getTextureColors(),
        maskImage, m_vModelGT, m_vModelColors);
    else
    outputInfo.meshDataGT = outputInfo.meshData;

    outputInfo.meshDataColorDiff = outputInfo.meshData;

    // get 2d projections
    vector<CoordinateType> proj2D, proj2DGT;
    proj2D.resize(2); proj2DGT.resize(2);
    for(int i = 0; i < m_nHeight*m_nWidth; ++i)
    {
        if(maskImage[i])
        {
            proj2D[0] = m_pCurProjResult[2*i];
            proj2D[1] = m_pCurProjResult[2*i + 1];
            proj2DGT[0] = m_pCurProjResultGT[2*i];
            proj2DGT[1] = m_pCurProjResultGT[2*i + 1];

            outputInfo.meshProj.push_back(proj2D);
            outputInfo.meshProjGT.push_back(proj2DGT);
        }

    }

    outputInfo.visibilityMask.resize(outputInfo.meshData.numVertices,true);
    
    // camera pose is always 0 in this case
    for(int i = 0; i < 6; ++i)
    outputInfo.camPose[i] = 0;

    trackerInitialized = true;

    btime::ptime mst2 = btime::microsec_clock::local_time();
    btime::time_duration msdiff = mst2 - mst1;

    std::cout << "Mesh Initialization Time: " << msdiff.total_milliseconds() << std::endl;

}

void ShapeSequenceReader::trackerUpdate(TrackerOutputInfo& outputInfo)
{
    // copy over data

    btime::ptime mst1 = btime::microsec_clock::local_time();

    PangaeaMeshIO::updateMesh(outputInfo.meshData, m_nHeight, m_nWidth,
        getTrackingResult(), getNormals(), getTextureColors(),
        maskImage, m_vModelEST, m_vModelColors);

    PangaeaMeshIO::updateMesh(outputInfo.meshDataGT, m_nHeight, m_nWidth,
        getGTTrackingResult(), getGTNormals(), getTextureColors(),
        maskImage, m_vModelGT, m_vModelColors);

    // get 2d projections
    int pntsId = 0;
    for(int i = 0; i < m_nHeight*m_nWidth; ++i)
    {
        if(maskImage[i])
        {
            outputInfo.meshProj[pntsId][0] = m_pCurProjResult[ 2*i ];
            outputInfo.meshProj[pntsId][1] = m_pCurProjResult[ 2*i+1 ];
            outputInfo.meshProjGT[pntsId][0] = m_pCurProjResultGT[ 2*i];
            outputInfo.meshProjGT[pntsId][1] = m_pCurProjResultGT[ 2*i + 1];
            ++pntsId;
        }
    }
    std::cout << "Projection Points Number: " << outputInfo.meshProj.size() <<
        std::endl;

    // camera pose is always 0 in this case
    for(int i = 0; i < 6; ++i)
    outputInfo.camPose[i] = 0;

    btime::ptime mst2 = btime::microsec_clock::local_time();
    btime::time_duration msdiff = mst2 - mst1;

    std::cout << "Mesh Update Time: " << msdiff.total_milliseconds() << std::endl;

}

void ShapeSequenceReader::readModelInfo()
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