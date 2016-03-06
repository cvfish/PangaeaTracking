#include "main_engine/tracker/FeaturePyramid.h"

#include "main_engine/utils/settings.h"

FeaturePyramid::FeaturePyramid()
  :prevLevels(NULL),
   currLevels(NULL),
   currLevelsBuffer(NULL)
{
  pFeatureReader = new HDF5Reader(featureSettings.dbPath);
}

FeaturePyramid::~FeaturePyramid()
{
  SafeDeleteArray(prevLevels);
  SafeDeleteArray(currLevels);
  SafeDeleteArray(currLevelsBuffer);

  delete pFeatureReader;
}

void FeaturePyramid::create(int nW, int nH, int nChannels, int numLevels)
{
  m_nWidth = nW;
  m_nHeight = nH;
  m_nNumChannels = nChannels;
  m_nNumLevels = numLevels;
  m_bInitialized = false;

  prevLevels = new FeatureLevel[numLevels];
  currLevels = new FeatureLevel[numLevels];
  currLevelsBuffer = new FeatureLevel[numLevels];

  pFeatureReader->InitializeDB(nH, nW, nChannels);
}

void FeaturePyramid::setupCameraPyramid(int numLevels, CameraInfo& camInfo)
{
  double factor = featureSettings.scalingFactor;

  for(int i = 0; i < numLevels; ++i)
    {
      double factor;
      if(featureSettings.featurePyramidSamplingFactors.size() > 0)
        {
          factor = featureSettings.scalingFactor *
            featureSettings.featurePyramidSamplingFactors[i];
        }
      else
        {
          factor = featureSettings.scalingFactor;
        }

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
}

void FeaturePyramid::setupPyramid(string key)
{

  for(int i = 0; i < m_nNumLevels; ++i)
    {
      currLevelsBuffer[i].featureImageVec.resize(m_nNumChannels);
      currLevelsBuffer[i].featureImageGradXVec.resize(m_nNumChannels);
      currLevelsBuffer[i].featureImageGradYVec.resize(m_nNumChannels);
    }

  int factor = 1;
  double blurSigma;

  for(int j = 0; j < m_nNumChannels; ++j)
    {
      for(int i = 0; i < m_nNumLevels; ++i)
        {
          pFeatureReader->getFeatureLevel(key, j, featureBufferImage);
          // features reading test
          // if(j == 1)
          //   {
          //   for(int mm = 881; mm < 891; ++mm)
          //     cout << featureBufferImage.at<double>(544, mm) << endl;
          // }

          int blurSize = featureSettings.blurFeatureFilterSizes[i];
          if(featureSettings.blurFeatureSigmaSizes.size() > 0)
            blurSigma = featureSettings.blurFeatureSigmaSizes[i];
          else
            blurSigma = 3;

          cout << "channel " << j << " " << "level " << i << endl;
          cout << "feature buffer: " << featureBufferImage.rows << " " << featureBufferImage.cols << endl;
          cout << "blur size: " << blurSize << endl;
          cout << "blur sigma: " << blurSigma << endl;

          if(blurSize > 0){
            cv::GaussianBlur(featureBufferImage,
                             blurBufferImage,
                             cv::Size(blurSize, blurSize),
                             blurSigma);
          }else
            featureBufferImage.copyTo(blurBufferImage);

          cout << "blur buffer: " << blurBufferImage.rows << " " << blurBufferImage.cols << endl;

          // do some proper downsampling at this point
          if(featureSettings.featurePyramidSamplingFactors.size() > 0){
            factor = featureSettings.featurePyramidSamplingFactors[i];
            resize(blurBufferImage,
                   currLevelsBuffer[i].featureImageVec[j],
                   cv::Size(),
                   1.0/factor,
                   1.0/factor);
          }
          else{
            blurBufferImage.copyTo(currLevelsBuffer[i].featureImageVec[j]);
          }

          cout << "currLevelsBuffer: " << currLevelsBuffer[i].featureImageVec[j].rows << " "
               << currLevelsBuffer[i].featureImageVec[j].cols << endl;

          // setup gradient
          double gradScale = featureSettings.featureGradientScalingFactors[i];

          cv::Scharr(currLevelsBuffer[i].featureImageVec[j],
                     currLevelsBuffer[i].featureImageGradXVec[j],
                     featureSettings.gradTypeINT,
                     1,0,
                     gradScale);

          cv::Scharr(currLevelsBuffer[i].featureImageVec[j],
                     currLevelsBuffer[i].featureImageGradYVec[j],
                     featureSettings.gradTypeINT,
                     0,1,
                     gradScale);

        }
    }

  // setup feature pyramid for previous frame
  if(!m_bInitialized)
    {
      // copy over the data from currLevelsBuffer
      for(int i = 0; i < m_nNumLevels; ++i){
        prevLevels[i].featureImageVec.resize(m_nNumChannels);
        prevLevels[i].featureImageGradXVec.resize(m_nNumChannels);
        prevLevels[i].featureImageGradYVec.resize(m_nNumChannels);
        for(int j = 0; j < m_nNumChannels; ++j){
          currLevelsBuffer[i].featureImageVec[j].copyTo( prevLevels[i].featureImageVec[j] );
          currLevelsBuffer[i].featureImageGradXVec[j].copyTo( prevLevels[i].featureImageGradXVec[j] );
          currLevelsBuffer[i].featureImageGradYVec[j].copyTo( prevLevels[i].featureImageGradYVec[j] );
        }
      }

      m_bInitialized = true;
    }

}

void FeaturePyramid::updateData()
{
  FeatureLevel* tempPointer;
  tempPointer = currLevelsBuffer;
  currLevelsBuffer = currLevels;
  currLevels = tempPointer;
}

void FeaturePyramid::updatePrev()
{
  // make sure prevLevels has already been initialized

  // exchange prevLevels and currLevels
  FeatureLevel* tempPointer;
  tempPointer = prevLevels;
  prevLevels = currLevels;
  currLevels = tempPointer;

}

CameraInfo& FeaturePyramid::getCameraInfo(int nLevel)
{
  return camInfoLevels[nLevel];
}

FeatureLevel& FeaturePyramid::getPrevFeatureLevel(int nLevel)
{
  return prevLevels[nLevel];
}

FeatureLevel& FeaturePyramid::getCurrFeatureLevel(int nLevel)
{
  return currLevels[nLevel];
}
