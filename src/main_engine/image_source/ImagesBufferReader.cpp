#include "main_engine/image_source/ImageSourceEngine.h"

ImagesBufferReader::ImagesBufferReader(ImageSourceSettings& settings)
{
    startFrameNo = settings.startFrame;
    totalFrameNo = settings.numFrames;
    m_nWidth = settings.width;
    m_nHeight = settings.height;
    inputPath = settings.dataPath;
    imgFormat = settings.imageFormat;

    currentFrameNo = startFrameNo;

    // for loop load all the images
    nFrameStep = settings.frameStep;
    int bufferSize = (totalFrameNo - startFrameNo)/nFrameStep + 1;
    m_vImages.resize(bufferSize);

    nFrameStep = settings.frameStep;
    char buffer[BUFFER_SIZE];
    for(int i = startFrameNo; i <= totalFrameNo; i = i + nFrameStep)
    {
        std::stringstream imagePath;
        sprintf(buffer, imgFormat.c_str(), i);
        imagePath << inputPath << buffer;
        //memset(&buffer[0], 0, sizeof(buffer));

        std::cout << imagePath.str() << std::endl;

        // ColorImageType tempImage;
        // tempImage = cv::imread(imagePath.str().c_str(),1); // read in color image
        // // cv::resize(tempImage, m_vImages[i-startFrame], cv::Size(), m_ShapeScale, m_ShapeScale);
        // m_vImages[i-startFrame] = tempImage;
        int bufferPos = (i-startFrameNo)/nFrameStep;
        m_vImages[ bufferPos ] = cv::imread(imagePath.str().c_str(),1); // read in color image
    }

    // read the calibration information
    std::stringstream intrinsicsFileName;
    intrinsicsFileName << inputPath << settings.intrinsicsFile;
    ifstream intrinsicsFile(intrinsicsFileName.str().c_str());
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            intrinsicsFile >> KK[i][j];
            std::cout << KK[i][j] << " ";
        }
        std::cout << std::endl;
    }

}

ImagesBufferReader::~ImagesBufferReader()
{
}

void ImagesBufferReader::setCurrentFrame(int curFrame)
{
    currentFrameNo = curFrame;  // update current frame number
}

// void ImagesBufferReader::getColorImage(unsigned char* pColorImage)
// {
//     pColorImage =  m_vImages[currentFrameNo - startFrameNo].data;
// }

unsigned char* ImagesBufferReader::getColorImage()
{
    int bufferPos = (currentFrameNo - startFrameNo) / nFrameStep;
    return m_vImages[bufferPos].data;
}

void ImagesBufferReader::readUVDImage(DepthImageType& uImage, DepthImageType& vImage,
    DepthImageType& dImage, InternalIntensityImageType& maskImageAux)
{
    // // read in uImage, vImage and dImage
    // ReadRawDepth(inputPath,"refX.raw",m_nWidth,m_nHeight,uImage);
    // ReadRawDepth(inputPath,"refY.raw",m_nWidth,m_nHeight,vImage);
    // ReadRawDepth(inputPath,"depth0001.raw",m_nWidth,m_nHeight,dImage);

    // // read in maskImage
    // std::stringstream imagePath;
    // imagePath << inputPath << "mask.png";
    // IntensityImageType maskImage = cv::imread(imagePath.str().c_str(),0);
    // maskImage.convertTo( maskImageAux, cv::DataType<CoordinateType>::type);

    // read in uImage, vImage and dImage
    std::stringstream data_path;
    data_path << inputPath;
    // DepthImageType uImage(m_nHeight,m_nWidth);
    // DepthImageType vImage(m_nHeight,m_nWidth);
    // DepthImageType dImage(m_nHeight,m_nWidth);
    if(trackerSettings.useXYZ)
    {
        ReadRawDepth(data_path,"refX.raw",m_nWidth,m_nHeight,uImage);
        ReadRawDepth(data_path,"refY.raw",m_nWidth,m_nHeight,vImage);
    }
    else
    {
        for(int i=0; i<m_nHeight; ++i)
        {
            for(int j=0; j<m_nWidth; ++j)
            {
                uImage(i,j) = j+1;
                vImage(i,j) = i+1;
            }
        }
    }
    // ReadRawDepth(data_path,"refX.raw",m_nWidth,m_nHeight,uImage);
    // ReadRawDepth(data_path,"refY.raw",m_nWidth,m_nHeight,vImage);
    ReadRawDepth(data_path,"depth0001.raw",m_nWidth,m_nHeight,dImage);

    // read in maskImage
    std::stringstream imagePath;

    if(trackerSettings.cropMask)
      imagePath << data_path.str() << "mask_crop.png";
    else
      imagePath << data_path.str() << "mask.png";

    IntensityImageType maskImage = cv::imread(imagePath.str().c_str(),0);
    maskImage.convertTo( maskImageAux, cv::DataType<CoordinateType>::type);

}

void ImagesBufferReader::ReadRawDepth(std::stringstream& data_path, std::string filename,
    int width, int height, DepthImageType& resImage)
{
    std::ifstream inputFileStream;
    std::stringstream imagePath;
    CoordinateType* ref_buffer;
    ref_buffer = new CoordinateType[width*height];
    imagePath << data_path.str() << filename;
    //            std::cout << data_path.str() << filename << std::endl;
    inputFileStream.open(imagePath.str().c_str(),std::ios::binary);
    inputFileStream.read((char*)ref_buffer,sizeof(CoordinateType)*width*height);
    inputFileStream.close();
    DepthImageType imgTemp(width,height,ref_buffer);
    imgTemp = imgTemp.t();
    imgTemp.copyTo(resImage);
    delete[] ref_buffer;
}
