#include "main_engine/tracker/ImagePyramid.h"
#include "main_engine/utils/settings.h"

ImagePyramid::ImagePyramid(){}

ImagePyramid& ImagePyramid::operator=(const ImagePyramid& imagePyramid)
{
  m_nWidth = imagePyramid.m_nWidth;
  m_nHeight = imagePyramid.m_nHeight;

  // ignore the pCurrentColorImageBGR, pCurrentColorImageRGB, pCurrentGrayImage
  // copy over the data in each ImageLevel
  levels.resize(imagePyramid.levels.size());
  levelsBuffer.resize(imagePyramid.levelsBuffer.size());

  camInfoLevels.resize(imagePyramid.camInfoLevels.size());

  for(int i = 0; i < imagePyramid.levels.size(); ++i)
    {
      imagePyramid.levels[i].grayImage.copyTo( levels[i].grayImage );
      imagePyramid.levels[i].gradXImage.copyTo( levels[i].gradXImage );
      imagePyramid.levels[i].gradYImage.copyTo( levels[i].gradYImage );

      imagePyramid.levels[i].colorImage.copyTo( levels[i].colorImage );

      imagePyramid.levels[i].depthImage.copyTo( levels[i].depthImage );
      imagePyramid.levels[i].depthGradXImage.copyTo( levels[i].depthGradXImage );
      imagePyramid.levels[i].depthGradYImage.copyTo( levels[i].depthGradYImage );

      for(int j = 0; j < 3; ++j)
        {
          imagePyramid.levels[i].colorImageSplit[j].copyTo( levels[i].colorImageSplit[j] );
          imagePyramid.levels[i].colorImageGradXSplit[j].copyTo( levels[i].colorImageGradXSplit[j] );
          imagePyramid.levels[i].colorImageGradYSplit[j].copyTo( levels[i].colorImageGradYSplit[j] );

          imagePyramid.levels[i].depthNormalImageSplit[j].copyTo( levels[i].depthNormalImageSplit[j] );
          imagePyramid.levels[i].depthNormalImageGradXSplit[j].copyTo( levels[i].depthNormalImageGradXSplit[j] );
          imagePyramid.levels[i].depthNormalImageGradYSplit[j].copyTo( levels[i].depthNormalImageGradYSplit[j] );

        }

    }

	return *this;
}

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
  int numImages = 1;
  if(imageSourceSettings.useMultiImages)
    numImages = imageSourceSettings.dataPathLevelList.size();

  pCurrentGrayImage =     new unsigned char[width*height*numImages];
  pCurrentColorImageRGB = new unsigned char[3*width*height*numImages];
  pCurrentColorImageBGR = new unsigned char[3*width*height*numImages];
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
  static int currFrame = imageSourceSettings.startFrame;

  int numImages = 1;
  if(imageSourceSettings.useMultiImages)
    {
      numImages = imageSourceSettings.dataPathLevelList.size();
      if(numImages != numLevels)
        cout << "levels of input images not consistent with mesh pyramid" << endl;
    }

  levelsBuffer.resize(numLevels);

  // copy over new data
  memcpy(pCurrentColorImageRGB, pColorImageRGB, 3*m_nWidth*m_nHeight*numImages);

  for(int k = 0; k < numImages; ++k)
    {
      int shift = k * 3 * m_nWidth * m_nHeight;

      for(int i = 0; i < m_nWidth * m_nHeight; ++i)
        {
          pCurrentColorImageBGR[ shift + 3*i ] =     pCurrentColorImageRGB[ shift + 3*i + 2 ];
          pCurrentColorImageBGR[ shift + 3*i + 1] =  pCurrentColorImageRGB[ shift + 3*i + 1 ];
          pCurrentColorImageBGR[ shift + 3*i + 2 ] = pCurrentColorImageRGB[ shift + 3*i ];

          pCurrentGrayImage[i] = 0.299*pCurrentColorImageRGB[ shift + 3*i ] +
                                 0.587*pCurrentColorImageRGB[ shift + 3*i+1 ] +
                                 0.114*pCurrentColorImageRGB[ shift + 3*i+2 ];

          // if( i == 709*m_nWidth + 706 )
          //   {
          //     cout << "term1 " << (int)pCurrentColorImageRGB[ shift + 3*i ]
          //          << "term2 " << (int)pCurrentColorImageRGB[ shift + 3*i + 1]
          //          << "term3 " << (int)pCurrentColorImageRGB[ shift + 3*i + 2] << endl;
          //     double haha = 0.299*pCurrentColorImageRGB[ shift + 3*i ] +
          //       0.587*pCurrentColorImageRGB[ shift + 3*i+1 ] +
          //       0.114*pCurrentColorImageRGB[ shift + 3*i+2 ];

          //     cout << "haha " << haha << endl;
          //     cout << "grayScale" << (int)pCurrentGrayImage[i] << endl;
          //     cout << "test " << (int)haha << endl;

          //     double tt = 0.0;
          //   }
        }

    }

  // all the memory with be allocated only during the first frame
  cv::Mat tempColorImageBGR(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageBGR);
  cv::cvtColor( tempColorImageBGR, grayImageBYTE, CV_BGR2GRAY );
  grayImageBYTE.convertTo( grayBufferImage, cv::DataType<CoordinateType>::type, 1./255 );

  cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageRGB);
  tempColorImageRGB.convertTo(colorBufferImage, cv::DataType<Vec3d>::type, 1./255);

  int factor = 1;
  double blurSigma;

  cout << "number of sampling factors specified " <<
    trackerSettings.imagePyramidSamplingFactors.size() << endl;

  // starts doing smoothing and gets gradients
  for(int i = 0; i < numLevels; ++i)
    {
      // use multiple images as input
      if(numImages > 1)
        {
          cout << "different image level " << i << endl;

          cv::Mat tempColorImageBGR(m_nHeight, m_nWidth, CV_8UC3, &pCurrentColorImageBGR[ i*3*m_nHeight*m_nWidth ] );
          cv::cvtColor( tempColorImageBGR, grayImageBYTE, CV_BGR2GRAY );
          grayImageBYTE.convertTo( grayBufferImage, cv::DataType<CoordinateType>::type, 1./255 );

          cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, &pCurrentColorImageRGB[ i*3*m_nHeight*m_nWidth ] );
          tempColorImageRGB.convertTo(colorBufferImage, cv::DataType<Vec3d>::type, 1./255);
        }

      // //check the input images properly
      // char imName[BUFFER_SIZE];
      // sprintf(imName, "%s/color_frame%04d_level%02d.png", trackerSettings.savePath.c_str(),
      //         currFrame, i);
      // cv::imwrite(imName, colorBufferImage*255);

      // sprintf(imName, "%s/gray_frame%04d_level%02d.png", trackerSettings.savePath.c_str(),
      //         currFrame, i);
      // cv::imwrite(imName, grayBufferImage*255);

      // cout << "double to int test " << (int)(1.6) << endl;

      // //test pCurrentGrayImage
      // cv::Mat tempByteImage(m_nHeight, m_nWidth, CV_8U, pCurrentGrayImage);
      // sprintf(imName, "%s/original_gray_frame%04d_level%02d.png", trackerSettings.savePath.c_str(),
      //         currFrame, i);
      // cv::imwrite(imName, tempByteImage);

      // cv::Mat tempColorImageBGR(m_nHeight, m_nWidth, CV_8UC3, &pCurrentColorImageBGR[ i*3*m_nHeight*m_nWidth ] );
      // sprintf(imName, "%s/original_color_frame%04d_level%02d.png", trackerSettings.savePath.c_str(),
      //         currFrame, i);
      // cv::imwrite(imName, tempColorImageBGR);

      // the standard deviation of gaussian blur is fixed as 3
      int blurSize = trackerSettings.blurFilterSizes[i];
      if(trackerSettings.blurSigmaSizes.size() > 0)
        blurSigma = trackerSettings.blurSigmaSizes[i];
      else
        blurSigma = -1;

      if(trackerSettings.useSigmaOnly && blurSigma > 0){
        cv::GaussianBlur(grayBufferImage,
                         blurGrayBufferImage,
                         cv::Size(0, 0),
                         blurSigma);

        cv::GaussianBlur(colorBufferImage,
                         blurColorBufferImage,
                         cv::Size(0, 0),
                         blurSigma);
      }
      else if(blurSize > 0)
        {
          cv::GaussianBlur(grayBufferImage,
                           blurGrayBufferImage,
                           cv::Size(blurSize, blurSize),
                           blurSigma);

          cv::GaussianBlur(colorBufferImage,
                           blurColorBufferImage,
                           cv::Size(blurSize, blurSize),
                           blurSigma);
        }else
        {
          grayBufferImage.copyTo(blurGrayBufferImage);
          colorBufferImage.copyTo(blurColorBufferImage);
        }

      // do some proper downsampling at this point
      if(trackerSettings.imagePyramidSamplingFactors.size() > 0)
        {
          factor = trackerSettings.imagePyramidSamplingFactors[i];

          resize(blurGrayBufferImage,
                 levelsBuffer[i].grayImage,
                 cv::Size(),
                 1.0/factor,
                 1.0/factor);

          resize(blurColorBufferImage,
                 levelsBuffer[i].colorImage,
                 cv::Size(),
                 1.0/factor,
                 1.0/factor);
        }

    }

  // have to check gradient scale properly
  for(int i = 0; i < numLevels; ++i)
    {
      double gradScale = trackerSettings.imageGradientScalingFactors[i];

      cv::Scharr(levelsBuffer[i].grayImage,
                 levelsBuffer[i].gradXImage,
                 -1,
                 1,0,
                 gradScale);
      cv::Scharr(levelsBuffer[i].grayImage,
                 levelsBuffer[i].gradYImage,
                 -1,
                 0,1,
                 gradScale);

      cv::split(levelsBuffer[i].colorImage, levelsBuffer[i].colorImageSplit);

      for(int j = 0; j < 3; ++j)
        {
          cv::Scharr(levelsBuffer[i].colorImageSplit[j],
                     levelsBuffer[i].colorImageGradXSplit[j],
                     -1,
                     1,0,
                     gradScale);

          cv::Scharr(levelsBuffer[i].colorImageSplit[j],
                     levelsBuffer[i].colorImageGradYSplit[j],
                     -1,
                     0,1,
                     gradScale);
        }

      // cv::namedWindow("level check");
      // cv::imshow("level check", levelsBuffer[i].colorImage);
      // cv::waitKey(0);
      // char imName[BUFFER_SIZE];
      // sprintf(imName, "level%02d.png", i);
      // cv::imwrite(imName, 255*levelsBuffer[i].colorImage);

    }

  // support for depth image will be added here
  // currFrame++;

}

void ImagePyramid::updateData()
{
  //exchange levels and levelsBuffer
  levels.swap( levelsBuffer );
}

CameraInfo& ImagePyramid::getCameraInfo(int nLevel)
{
  return camInfoLevels[nLevel];
}

ImageLevel& ImagePyramid::getImageLevel(int nLevel)
{
  return levels[nLevel];
}

InternalIntensityImageType* ImagePyramid::getColorImageSplit(int nLevel)
{
  return levels[nLevel].colorImageSplit;
}

IntensityImageType& ImagePyramid::getIntensityImageByte()
{
  return grayImageBYTE;
}

unsigned char* ImagePyramid::getCurrentGrayImage()
{
  return pCurrentGrayImage;
}

unsigned char* ImagePyramid::getColorImage()
{
  return pCurrentColorImageRGB;
}
