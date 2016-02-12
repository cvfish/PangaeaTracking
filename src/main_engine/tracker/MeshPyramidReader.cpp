#include "main_engine/tracker/TrackingEngine.h"

MeshPyramidReader::MeshPyramidReader(MeshLoadingSettings& settings, int width,
                                     int height, double K[3][3], int startFrame, int numTrackingFrames): trackerInitialized(false)
{
  m_nWidth = width;
  m_nHeight = height;
  startFrameNo = startFrame;
  currentFrameNo = startFrame;

  pCurrentColorImageRGB = new unsigned char[3*width*height];
  // in this case camPose will always be zero
  for(int i = 0; i < 6; ++i)
    camPose[i] = 0;

  useVisibilityMask = settings.visibilityMask;

  setIntrinsicMatrix(K);

  TICK("loadingMesh");

  currentMeshPyramid = std::move(
                                 PangaeaMeshPyramid(
                                                    settings.meshPath,
                                                    settings.meshLevelFormat,
                                                    currentFrameNo,
                                                    settings.meshLevelList,
                                                    settings.clockwise));

  if(settings.loadProp)
    {
      propMeshPyramid = std::move(
                                  PangaeaMeshPyramid(
                                                     settings.meshPath,
                                                     settings.propLevelFormat,
                                                     currentFrameNo,
                                                     settings.meshLevelList,
                                                     settings.clockwise));
    }

  if(settings.hasGT)
    {
      currentMeshPyramidGT = std::move(
                                       PangaeaMeshPyramid(
                                                          settings.meshPathGT,
                                                          settings.meshLevelFormatGT,
                                                          currentFrameNo,
                                                          settings.meshLevelListGT,
                                                          settings.clockwise));
    }

  TOCK("loadingMesh");

  m_nNumMeshLevels = settings.meshLevelList.size();

}

MeshPyramidReader::~MeshPyramidReader(){
  delete[] pCurrentColorImageRGB;
}

bool MeshPyramidReader::setCurrentFrame(int curFrame)
{
  if(currentFrameNo != curFrame)
    {
      currentFrameNo = curFrame;

      TICK("setCurrentFrame");

      if(!loadMeshPyramid(
                          meshLoadingSettings.meshPath,
                          meshLoadingSettings.meshLevelFormat,
                          currentFrameNo,
                          meshLoadingSettings.meshLevelList))
        return false;

      TOCK("setCurrentFrame");

    }
  return true;

}

bool MeshPyramidReader::loadMeshPyramid(string meshPath, string meshLevelFormat,
                                        int frame, IntegerContainerType& meshLevelList)
{
  // check if file exist or not first
  if(!existenceTest(meshPath, meshLevelFormat, frame, meshLevelList))
    return false;

  if(!trackerInitialized){

    currentMeshPyramid = std::move(
                                   PangaeaMeshPyramid(
                                                      meshLoadingSettings.meshPath,
                                                      meshLoadingSettings.meshLevelFormat,
                                                      currentFrameNo,
                                                      meshLoadingSettings.meshLevelList,
                                                      meshLoadingSettings.clockwise));
    if(meshLoadingSettings.loadProp)
      propMeshPyramid = std::move(
                                  PangaeaMeshPyramid(
                                                     meshLoadingSettings.meshPath,
                                                     meshLoadingSettings.propLevelFormat,
                                                     currentFrameNo,
                                                     meshLoadingSettings.meshLevelList,
                                                     meshLoadingSettings.clockwise));

    if(meshLoadingSettings.hasGT)
      currentMeshPyramidGT = std::move(
                                       PangaeaMeshPyramid(
                                                          meshLoadingSettings.meshPathGT,
                                                          meshLoadingSettings.meshLevelFormatGT,
                                                          currentFrameNo,
                                                          meshLoadingSettings.meshLevelListGT,
                                                          meshLoadingSettings.clockwise));

  }else
    {
      currentMeshPyramid.updatePyramid(
                                       meshLoadingSettings.meshPath,
                                       meshLoadingSettings.meshLevelFormat,
                                       currentFrameNo,
                                       meshLoadingSettings.meshLevelList);

      if(meshLoadingSettings.loadProp)
        propMeshPyramid.updatePyramid(
                                      meshLoadingSettings.meshPath,
                                      meshLoadingSettings.propLevelFormat,
                                      currentFrameNo,
                                      meshLoadingSettings.meshLevelList);

      if(meshLoadingSettings.hasGT)
        currentMeshPyramidGT.updatePyramid(
                                           meshLoadingSettings.meshPathGT,
                                           meshLoadingSettings.meshLevelFormatGT,
                                           currentFrameNo,
                                           meshLoadingSettings.meshLevelListGT);

      // check the normals
      double normals[3];
      double normals_gt[3];

      for(int i = 0; i < 3; ++i)
        {
          normals[i] = 0.0;
          normals_gt[i] = 0.0;
        }

      for(int i = 0; i < currentMeshPyramid.levels[0].numVertices; ++i)
        {
          normals[0] += currentMeshPyramid.levels[0].normals[i][0];
          normals[1] += currentMeshPyramid.levels[0].normals[i][1];
          normals[2] += currentMeshPyramid.levels[0].normals[i][2];
          normals_gt[0] += currentMeshPyramidGT.levels[0].normals[i][0];
          normals_gt[1] += currentMeshPyramidGT.levels[0].normals[i][1];
          normals_gt[2] += currentMeshPyramidGT.levels[0].normals[i][2];

        }

      cout << "normals of tracking results: " << normals[0] << " " << normals[1] << " " << normals[2] << endl;
      cout << "normals of ground truth results: " << normals_gt[0] << " " << normals_gt[1] << " " << normals_gt[2] << endl;

    }
  return true;
}

void MeshPyramidReader::setIntrinsicMatrix(double K[3][3])
{
  for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
        {
          KK[i][j] = K[i][j];
        }
    }
}

bool MeshPyramidReader::trackFrame(int nFrame, unsigned char* pColorImageRGB,
                                   TrackerOutputInfo** pOutputInfo)
{
  if(!setCurrentFrame(nFrame))
    return false;

  // set up the color used later
  memcpy(pCurrentColorImageRGB, pColorImageRGB, 3*m_nWidth*m_nHeight);
  cv::Mat tempColorImageRGB(m_nHeight, m_nWidth, CV_8UC3, pCurrentColorImageRGB);
  tempColorImageRGB.convertTo(colorImage, cv::DataType<Vec3d>::type, 1./255);
  cv::split(colorImage, colorImageSplit);

  setMeshPyramid();

  // cout << "frame " << nFrame << ":" << endl;
  // //print mesh center
  // for(int i = 0; i < currentMeshPyramid.numLevels; ++i)
  // {
  //     cout << "level " << i << ":" << endl;
  //     cout << "center: " << currentMeshPyramid.levels[i].center[0] << " "
  //          << currentMeshPyramid.levels[i].center[1] << " "
  //          << currentMeshPyramid.levels[i].center[2] << " "
  //          << endl;
  // }

  // cout << "number of pyramid levels " << outputInfoPyramid.size() << endl;
  // *pOutputInfo = &outputInfoPyramid[0];
  if(!trackerInitialized)
    {
      *pOutputInfo = &outputInfoPyramid[0];
      trackerInitialized = true;
    }

  return true;
}

void MeshPyramidReader::setMeshPyramid()
{
  visibilityMaskPyramid.resize(m_nNumMeshLevels);
  outputInfoPyramid.resize(m_nNumMeshLevels);
  outputPropPyramid.resize(m_nNumMeshLevels);

  for(int i = 0; i < m_nNumMeshLevels; ++i)
    {
      int numVertices = currentMeshPyramid.levels[i].numVertices;
      visibilityMaskPyramid[i].resize(numVertices,true);

      vector<CoordinateType> proj2D;
      proj2D.resize(2); proj2D[0] = 0; proj2D[1] = 0;

      outputInfoPyramid[i].meshData = currentMeshPyramid.levels[i];
      outputInfoPyramid[i].nRenderLevel = i;
      outputInfoPyramid[i].meshProj.resize(numVertices, proj2D);

      // memset(outputInfoPyramid[i].camPose, 0, 6*sizeof(double));
      // UpdateRenderingData(outputInfoPyramid[i], KK, camPose, outputInfoPyramid[i].meshData);
      UpdateRenderingDataFast(outputInfoPyramid[i], KK, outputInfoPyramid[i].meshData);

      if(!meshLoadingSettings.fastLoading)
        {
          if(!meshLoadingSettings.hasGT)
            {
              outputInfoPyramid[i].meshDataGT = outputInfoPyramid[i].meshData;
              outputInfoPyramid[i].meshProjGT = outputInfoPyramid[i].meshProj;
            }else
            {
              outputInfoPyramid[i].meshDataGT = currentMeshPyramidGT.levels[i];
              outputInfoPyramid[i].meshProjGT.resize(numVertices, proj2D);
              UpdateRenderingDataFast(outputInfoPyramid[i], KK, outputInfoPyramid[i].meshDataGT, true);
            }

          outputInfoPyramid[i].meshDataColorDiff = outputInfoPyramid[i].meshData;
          outputInfoPyramid[i].meshDataColorDiffGT = outputInfoPyramid[i].meshDataColorDiff;
          outputInfoPyramid[i].visibilityMaskGT = outputPropPyramid[i].visibilityMask;

        }

      // update the visibility of each vertex
      if(useVisibilityMask)
        {

          long long int ii = i;

          TICK( "visibilityMaskLevel" + std::to_string(ii) );

          UpdateVisibilityMaskGL(outputInfoPyramid[i], visibilityMaskPyramid[i], KK, camPose, m_nWidth, m_nHeight);

          if(!meshLoadingSettings.fastLoading)
            UpdateColorDiff(outputInfoPyramid[i], outputInfoPyramid[i].visibilityMask, colorImageSplit);


          TOCK( "visibilityMaskLevel"  + std::to_string(ii) );
        }

      //////////////////////////// outputPropPyramid
      if(meshLoadingSettings.loadProp)
        {
          outputPropPyramid[i].meshData = propMeshPyramid.levels[i];
          outputPropPyramid[i].nRenderLevel = i;
          outputPropPyramid[i].meshProj.resize(numVertices, proj2D);
          outputPropPyramid[i].visibilityMask.resize(numVertices, true);

          // memset(outputPropPyramid[i].camPose, 0, 6*sizeof(double));
          // UpdateRenderingData(outputPropPyramid[i], KK, camPose, propMeshPyramid.levels[i]);
          UpdateRenderingDataFast(outputPropPyramid[i], KK, propMeshPyramid.levels[i]);

          if(!meshLoadingSettings.fastLoading &&
             !meshLoadingSettings.hasGT)
            {
              outputPropPyramid[i].meshDataGT = propMeshPyramid.levels[i];
              outputPropPyramid[i].meshDataColorDiff = propMeshPyramid.levels[i];
              outputPropPyramid[i].meshProjGT = outputPropPyramid[i].meshProj;
            }

          // update the visibility of each vertex
          if(useVisibilityMask)
            {
              UpdateVisibilityMaskGL(outputPropPyramid[i], visibilityMaskPyramid[i], KK, camPose, m_nWidth, m_nHeight);
              if(!meshLoadingSettings.fastLoading)
                UpdateColorDiff(outputPropPyramid[i], outputPropPyramid[i].visibilityMask, colorImageSplit);
            }
        }
    }
}

void MeshPyramidReader::updateRenderingLevel(TrackerOutputInfo** pOutputInfoRendering,
                                             int nRenderLevel, bool renderType)
{
  cout << "changing rendering type" << endl;

  if(renderType)
    *pOutputInfoRendering = &(outputPropPyramid[nRenderLevel]);
  else
    *pOutputInfoRendering = &(outputInfoPyramid[nRenderLevel]);
}
