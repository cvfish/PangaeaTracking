#ifndef _IMAGE_SOURCE_ENGINE_H
#define _IMAGE_SOURCE_ENGINE_H

#include "../utils/global.h"
#include "../utils/settings.h"

class ImageSourceEngine
{

public:

    ImageSourceEngine(){};
    virtual ~ImageSourceEngine(){};

    virtual void setCurrentFrame(int curFrame) = 0;
    virtual unsigned char* getColorImage() = 0;
    virtual void readUVDImage(DepthImageType& uImage, DepthImageType& vImage,
        DepthImageType& dImage, InternalIntensityImageType& maskImage) = 0;

};

class ImagesBufferReader : public ImageSourceEngine
{

public:

    ImagesBufferReader(ImageSourceSettings& settings);

    ~ImagesBufferReader();

    void setCurrentFrame(int curFrame);
    unsigned char* getColorImage();
    void readUVDImage(DepthImageType& uImage, DepthImageType& vImage,
        DepthImageType& dImage, InternalIntensityImageType& maskImage);
    void ReadRawDepth(std::stringstream& data_path, std::string filename,
        int width, int height, DepthImageType& resImage);

    int startFrameNo;
    int currentFrameNo;
    int totalFrameNo;

    int m_nHeight;
    int m_nWidth;
    double KK[3][3];

    double m_ShapeScale;

    std::string inputPath;
    std::string imgFormat;

    ColorImageContainerType m_vImages; // all images
    
    // uvd and mask image gives the initial shape to start with
    DepthImageType uImage;
    DepthImageType vImage;
    DepthImageType dImage;
    InternalIntensityImageType maskImage;

    int nFrameStep;

};

class ImageSequenceReader : public ImageSourceEngine
{

public:

    ImageSequenceReader(ImageSourceSettings& settings);

    ImageSequenceReader(std::string& inputPath, std::string& imgFormat,
        int nHeight, int nWidth, int startFrame, int numTrackingFrames, double shapeScale = 1.0);

    ~ImageSequenceReader();

    void setCurrentFrame(int curFrame);
    unsigned char* getColorImage();
    void readUVDImage(DepthImageType& uImage, DepthImageType& vImage,
        DepthImageType& dImage, InternalIntensityImageType& maskImage);
    void ReadRawDepth(std::stringstream& data_path, std::string filename,
        int width, int height, DepthImageType& resImage);

    int startFrameNo;
    int currentFrameNo;
    int totalFrameNo;

    int m_nHeight;
    int m_nWidth;
    double KK[3][3];

    double m_ShapeScale;

    std::string inputPath;
    std::string imgFormat;

    ColorImageType m_curImage; // current image

    // uvd and mask image gives the initial shape to start with
    DepthImageType uImage;
    DepthImageType vImage;
    DepthImageType dImage;
    InternalIntensityImageType maskImage;

};

#endif