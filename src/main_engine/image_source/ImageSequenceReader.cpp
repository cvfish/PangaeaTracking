#include "main_engine/image_source/ImageSourceEngine.h"

ImageSequenceReader::ImageSequenceReader(ImageSourceSettings& settings)
{
    startFrameNo = settings.startFrame;
    totalFrameNo = settings.numFrames;
    m_nWidth = settings.width;
    m_nHeight = settings.height;
    inputPath = settings.dataPath;
    imgFormat = settings.imageFormat;

    currentFrameNo = startFrameNo;

    char buffer[BUFFER_SIZE];
    std::stringstream imagePath;
    sprintf(buffer, imgFormat.c_str(), currentFrameNo);
    imagePath << inputPath << buffer;
    //memset(&buffer[0], 0, sizeof(buffer));

    m_curImage = cv::imread(imagePath.str().c_str(),1); // read in color image

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

ImageSequenceReader::ImageSequenceReader(std::string& inputPath, std::string& imgFormat,
    int nHeight, int nWidth, int startFrame, int numTrackingFrames, double shapeScale):
    startFrameNo(startFrame),
    totalFrameNo(numTrackingFrames),
    currentFrameNo(startFrame),
    m_nHeight(nHeight),
    m_nWidth(nWidth),
    m_ShapeScale(shapeScale),
    inputPath(inputPath),
    imgFormat(imgFormat)
{
    char buffer[BUFFER_SIZE];
    std::stringstream imagePath;
    sprintf(buffer, imgFormat.c_str(), currentFrameNo);
    imagePath << inputPath << buffer;
    //memset(&buffer[0], 0, sizeof(buffer));

    m_curImage = cv::imread(imagePath.str().c_str(),1); // read in color image
}

ImageSequenceReader::~ImageSequenceReader()
{}

void ImageSequenceReader::setCurrentFrame(int curFrame)
{
    if(currentFrameNo != curFrame)
    {
        currentFrameNo = curFrame;  // update current frame number

        char buffer[BUFFER_SIZE];
        std::stringstream imagePath;
        sprintf(buffer, imgFormat.c_str(), currentFrameNo);
        imagePath << inputPath << buffer;
        //memset(&buffer[0], 0, sizeof(buffer));

        std::cout << imagePath.str() << std::endl;

        try
        {
            m_curImage = cv::imread(imagePath.str().c_str(),1); // read in color image
        }
        catch(exception& e)
        {
            cerr << "ERROR: cv::imread failed." << endl;
            cerr << "(Exception = " << e.what() << ")" << endl;
        }
        
    }
}

unsigned char* ImageSequenceReader::getColorImage()
{
    return m_curImage.data;
}

void ImageSequenceReader::readUVDImage(DepthImageType& uImage, DepthImageType& vImage,
    DepthImageType& dImage, InternalIntensityImageType& maskImageAux)
{
    // read in uImage, vImage and dImage
    std::stringstream data_path;
    data_path << inputPath;
    if(trackerSettings.useXYZ)
    {
        // for perspective camera, in this case
        ReadRawDepth(data_path,"refX.raw",m_nWidth,m_nHeight,uImage);
        ReadRawDepth(data_path,"refY.raw",m_nWidth,m_nHeight,vImage);
    }
    else
    {
        // mainly for orthographic camera
        for(int i=0; i<m_nHeight; ++i)
        {
            for(int j=0; j<m_nWidth; ++j)
            {
                uImage(i,j) = j+1;
                vImage(i,j) = i+1;
            }
        }        
    }
    ReadRawDepth(data_path,"depth0001.raw",m_nWidth,m_nHeight,dImage);

    // read in maskImage
    std::stringstream imagePath;
    imagePath << data_path.str() << "mask.png";
    IntensityImageType maskImage = cv::imread(imagePath.str().c_str(),0);
    maskImage.convertTo( maskImageAux, cv::DataType<CoordinateType>::type);
    
}

void ImageSequenceReader::ReadRawDepth(std::stringstream& data_path, std::string filename,
    int width, int height, DepthImageType& resImage)
{
    std::ifstream inputFileStream;
    std::stringstream imagePath;
    CoordinateType* ref_buffer;
    imagePath << data_path.str() << filename;
    if(bfs::exists(imagePath.str()))
    {
        ref_buffer = new CoordinateType[width*height];
        //            std::cout << data_path.str() << filename << std::endl;
        inputFileStream.open(imagePath.str().c_str(),std::ios::binary);
        inputFileStream.read((char*)ref_buffer,sizeof(CoordinateType)*width*height);
        inputFileStream.close();
        DepthImageType imgTemp(width,height,ref_buffer);
        imgTemp = imgTemp.t();
        imgTemp.copyTo(resImage);
        delete[] ref_buffer;
    }
    else
    {
        cerr << imagePath.str() << " does not exist! " << endl;
    }    
}