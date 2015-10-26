#pragma once

#include "../utils/global.h"

struct CameraInfo
{
    bool isOrthoCamera;
    double KK[3][3];
    //double pos[6];
    int width;
    int height;

    //
    double invKK[3][3];
};

struct ImageLevel
{
    InternalIntensityImageType grayImage;
    InternalIntensityImageType gradXImage;
    InternalIntensityImageType gradYImage;

    InternalColorImageType colorImage;
    InternalIntensityImageType colorImageSplit[3];
    InternalIntensityImageType colorImageGradXSplit[3];
    InternalIntensityImageType colorImageGradYSplit[3];

    // support for depth image to be added
    // depth image, just ignore if not used
    InternalIntensityImageType depthImage;
    InternalIntensityImageType depthGradXImage;
    InternalIntensityImageType depthGradYImage;

    // depth normal image
    InternalIntensityImageType depthNormalImageSplit[3];
    InternalIntensityImageType depthNormalImageGradXSplit[3];
    InternalIntensityImageType depthNormalImageGradYSplit[3];

};

class ImagePyramid
{
public:
    ImagePyramid();
    ~ImagePyramid();

    void create(int width, int height);
    void allocateMemory(int width, int height); // allocate memory based on the size
    void deallocateMemory();

    void setupPyramid(unsigned char* pColorImageRGB, int numLevels);
    void setupCameraPyramid(int numLevels, CameraInfo& camInfo);

    unsigned char* pCurrentColorImageRGB;
    unsigned char* pCurrentColorImageBGR;
    unsigned char* pCurrentGrayImage;

    vector<ImageLevel> levels;
    vector<CameraInfo> camInfoLevels;

    int m_nWidth;
    int m_nHeight;
};