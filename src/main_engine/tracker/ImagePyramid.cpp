#include "main_engine/tracker/ImagePyramid.h"
#include "main_engine/utils/settings.h"

ImagePyramid::ImagePyramid(){}

void ImagePyramid::create(int width, int height)
{
    allocateMemory(width, height);
    m_nWidth = width;
    m_nHeight = height;
}

void ImagePyramid::setupCameraPyramid(int numLevels, CameraInfo& camInfo)
{
    double factor = 1;
    for(int i = 0; i < numLevels; ++i)
    {
        if(trackerSettings.imagePyramidSamplingFactors.size() > 0)
        {
            factor = trackerSettings.imagePyramidSamplingFactors[i];
            CameraInfo camInfoLevel = camInfo;
            // update the parameters
            camInfoLevel.width = camInfo.width  / factor;
            camInfoLevel.height = camInfo.height  / factor;
            camInfoLevel.KK[0][0] = camInfo.KK[0][0] / factor;
            camInfoLevel.KK[0][2] = camInfo.KK[0][2] / factor;
            camInfoLevel.KK[1][1] = camInfo.KK[1][1] / factor;
            camInfoLevel.KK[1][2] = camInfo.KK[1][2] / factor;
            camInfoLevel.invKK[0][0] = camInfo.invKK[0][0] * factor;
            camInfoLevel.invKK[1][1] = camInfo.invKK[1][1] * factor;
            // the invKK[0][2] and invKK[1][2] remains the same
            camInfoLevels.push_back(camInfoLevel);
        }
        else
        {
            camInfoLevels.push_back(camInfo);
        }
    }

}

ImagePyramid::~ImagePyramid()
{
    deallocateMemory();
}

void ImagePyramid::allocateMemory(int width, int height)
{
    // allocate memory
    pCurrentGrayImage = new unsigned char[width*height];
    pCurrentColorImageRGB = new unsigned char[3*width*height];
    pCurrentColorImageBGR = new unsigned char[3*width*height];
}

void ImagePyramid::deallocateMemory()
{
    if(pCurrentGrayImage)
    delete []pCurrentGrayImage;

    if(pCurrentColorImageRGB)
    delete []pCurrentColorImageRGB;

    if(pCurrentColorImageBGR)
    delete []pCurrentColorImageBGR;
}

void ImagePyramid::setupPyramid(unsigned char* pColorImageRGB, int numLevels)
{
    levels.resize(numLevels);

    // copy over new data
    memcpy(pCurrentColorImageRGB, pColorImageRGB, 3*m_nWidth*m_nHeight);

    for(int i = 0; i < m_nWidth*m_nHeight; ++i)
    {
        pCurrentColorImageBGR[ 3*i ] = pCurrentColorImageRGB[ 3*i + 2 ];
        pCurrentColorImageBGR[ 3*i + 1] = pCurrentColorImageRGB[ 3*i + 1 ];
        pCurrentColorImageBGR[ 3*i + 2 ] = pCurrentColorImageRGB[ 3*i ];

        pCurrentGrayImage[i] = 0.299*pCurrentColorImageRGB[ 3*i ] +
            0.587*pCurrentColorImageRGB[ 3*i+1 ] +
            0.114*pCurrentColorImageRGB[ 3*i+2 ];
    }

    IntensityImageType grayImageBYTE;
    InternalIntensityImageType& grayImage = levels[0].grayImage;
    InternalColorImageType& colorImage = levels[0].colorImage;

    // all the memory with be allocated only during the first frame
    cv::Mat tempColorImageBGR(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageBGR);
    cv::cvtColor( tempColorImageBGR, grayImageBYTE, CV_BGR2GRAY );
    grayImageBYTE.convertTo( grayImage, cv::DataType<CoordinateType>::type, 1./255 );

    cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageRGB);
    tempColorImageRGB.convertTo(colorImage, cv::DataType<Vec3d>::type, 1./255);

    // there is memory allocation only for the first time
    for(int i = 1; i < numLevels; ++i)
    {
        grayImage.copyTo(levels[i].grayImage);
        colorImage.copyTo(levels[i].colorImage);
    }

    InternalIntensityImageType grayTempImage;
    InternalColorImageType colorTempImage;
    int factor=1;

    cout << "number of sampling factors specified " <<
        trackerSettings.imagePyramidSamplingFactors.size() << endl;

    // starts doing smoothing and gets gradients
    for(int i = 0; i < numLevels; ++i)
    {
        // the standard deviation of gaussian blur is fixed as 3
        int blurSize = trackerSettings.blurFilterSizes[i];
        if(blurSize > 0)
        {
            cv::GaussianBlur(levels[i].grayImage,
                levels[i].grayImage,
                cv::Size(blurSize, blurSize),
                3);

            cv::GaussianBlur(levels[i].colorImage,
                levels[i].colorImage,
                cv::Size(blurSize, blurSize),
                3);
        }
        // do some proper downsampling at this point
        if(trackerSettings.imagePyramidSamplingFactors.size() > 0)
        {
            factor = trackerSettings.imagePyramidSamplingFactors[i];
            resize(levels[i].grayImage, grayTempImage, cv::Size(), 1.0/factor, 1.0/factor);
            grayTempImage.copyTo(levels[i].grayImage);
            resize(levels[i].colorImage, colorTempImage, cv::Size(), 1.0/factor, 1.0/factor);
            colorTempImage.copyTo(levels[i].colorImage);
        }

    }

    // have to check gradient scale properly
    for(int i = 0; i < numLevels; ++i)
    {
        double gradScale = trackerSettings.imageGradientScalingFactors[i];

        cv::Scharr(levels[i].grayImage,
            levels[i].gradXImage,
            grayImage.depth(),
            1,0,
            gradScale);
        cv::Scharr(levels[i].grayImage,
            levels[i].gradYImage,
            grayImage.depth(),
            0,1,
            gradScale);

        cv::split(levels[i].colorImage, levels[i].colorImageSplit);

        for(int j = 0; j < 3; ++j)
        {
            cv::Scharr(levels[i].colorImageSplit[j],
                levels[i].colorImageGradXSplit[j],
                grayImage.depth(),
                1,0,
                gradScale);

            cv::Scharr(levels[i].colorImageSplit[j],
                levels[i].colorImageGradYSplit[j],
                grayImage.depth(),
                0,1,
                gradScale);
        }
    }

    // support for depth image will be added here

}