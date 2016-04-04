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

  // check the normals
  // double normals[3];
  // double normals_gt[3];

  // for(int i = 0; i < 3; ++i)
  //   {
  //     normals[i] = 0.0;
  //     normals_gt[i] = 0.0;
  //   }

  // for(int i = 0; i < currentMeshPyramid.levels[0].numVertices; ++i)
  //   {
  //     normals[0] += currentMeshPyramid.levels[0].normals[i][0];
  //     normals[1] += currentMeshPyramid.levels[0].normals[i][1];
  //     normals[2] += currentMeshPyramid.levels[0].normals[i][2];
  //     normals_gt[0] += currentMeshPyramidGT.levels[0].normals[i][0];
  //     normals_gt[1] += currentMeshPyramidGT.levels[0].normals[i][1];
  //     normals_gt[2] += currentMeshPyramidGT.levels[0].normals[i][2];

  //   }

  // cout << "during initialization " << endl;
  // cout << "normals of tracking results: " << normals[0] << " " << normals[1] << " " << normals[2] << endl;
  // cout << "normals of ground truth results: " << normals_gt[0] << " " << normals_gt[1] << " " << normals_gt[2] << endl;


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
      if(!meshLoadingSettings.rigidRendering || currentFrameNo == startFrameNo)
        {
          if(!loadMeshPyramid(
                              meshLoadingSettings.meshPath,
                              meshLoadingSettings.meshLevelFormat,
                              currentFrameNo,
                              meshLoadingSettings.meshLevelList))
            return false;
        }

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
        {

          currentMeshPyramidGT.updatePyramid(
                                             meshLoadingSettings.meshPathGT,
                                             meshLoadingSettings.meshLevelFormatGT,
                                             currentFrameNo,
                                             meshLoadingSettings.meshLevelListGT);

          // // check the normals
          // double normals[3];
          // double normals_gt[3];

          // for(int i = 0; i < 3; ++i)
          //   {
          //     normals[i] = 0.0;
          //     normals_gt[i] = 0.0;
          //   }

          // for(int i = 0; i < currentMeshPyramid.levels[0].numVertices; ++i)
          //   {
          //     normals[0] += currentMeshPyramid.levels[0].normals[i][0];
          //     normals[1] += currentMeshPyramid.levels[0].normals[i][1];
          //     normals[2] += currentMeshPyramid.levels[0].normals[i][2];
          //     normals_gt[0] += currentMeshPyramidGT.levels[0].normals[i][0];
          //     normals_gt[1] += currentMeshPyramidGT.levels[0].normals[i][1];
          //     normals_gt[2] += currentMeshPyramidGT.levels[0].normals[i][2];

          //   }

          // cout << "normals of tracking results: " << normals[0] << " " << normals[1] << " " << normals[2] << endl;
          // cout << "normals of ground truth results: " << normals_gt[0] << " " << normals_gt[1] << " " << normals_gt[2] << endl;

        }


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
  if(currentFrameNo > startFrameNo && meshLoadingSettings.rigidRendering)
    {
      // do not load new meshes, just rigid transform the first mesh
      // translate this mesh along x axis
      double camPose[6] = {0,0,0,0,0,0};
      double rot_angle = (currentFrameNo - startFrameNo)*10*3.14/180;
      int numLevels = currentMeshPyramid.numLevels;

      camPose[0] = rot_angle;
      double* center = currentMeshPyramid.levels[0].center;

      // cout << "print center coordinates" << endl;
      // cout << std::setprecision(15) << center[0] << " "
      //      << std::setprecision(15) << center[1] << " "
      //      << std::setprecision(15) << center[2] << endl;

      Vector3d axis;
      double angle = Map<Vector3d>(camPose).norm();
      if(angle != 0)
        axis = Map<Vector3d>(camPose)/angle;
      else
        axis << 1, 0, 0;

      // cout << "rotation angle" << endl;
      // cout << rot_angle << " " << angle << endl;

      Matrix3d R = AngleAxisd(angle, axis).toRotationMatrix();

      cout << "rotation matrix" << endl;
      cout << R << endl;
      // cout << "rotation matrix * center" << endl;
      // cout << R * Vector3d::Map(center) << endl;

      Map<Vector3d>(camPose+3) =  -R * Vector3d::Map(center) + Vector3d::Map(center);

      // cout << "translation vector" << endl;
      // cout << Map<Vector3d>(camPose+3) << endl;

      for(int i = 0; i < numLevels; ++i)
        {
          UpdateRenderingData(outputInfoPyramid[i], KK, camPose, currentMeshPyramid.levels[i], true);
          UpdateRenderingData(outputInfoPyramid[i], KK, camPose, currentMeshPyramid.levels[i], false);
        }
      return;
    }

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

          UpdateVisibilityMaskGL(outputInfoPyramid[i],
                                 visibilityMaskPyramid[i],
                                 KK, camPose, m_nWidth, m_nHeight);
          UpdateVisibilityMaskGL(outputInfoPyramid[i].meshDataGT,
                                 outputInfoPyramid[i].visibilityMaskGT,
                                 KK, camPose, m_nWidth, m_nHeight);

          // check the visiblity mask during results loading
          int visibleNum = 0;
          int visibleNumGT = 0;
          for(int kk = 0; kk < visibilityMaskPyramid[i].size(); ++kk)
            {
              if(visibilityMaskPyramid[i][kk])
                visibleNum++;
              if(outputInfoPyramid[i].visibilityMaskGT[kk])
                visibleNumGT++;
            }
          cout << "total number of points " << visibilityMaskPyramid[i].size() << endl;
          cout << "number of visible points for tracking results " << visibleNum << endl;
          cout << "number of visible points for ground truth " << visibleNumGT << endl;

          if(!meshLoadingSettings.fastLoading)
            {
              UpdateColorDiff(outputInfoPyramid[i], outputInfoPyramid[i].visibilityMask, colorImageSplit);
              UpdateColorDiffGT(outputInfoPyramid[i], outputInfoPyramid[i].visibilityMaskGT, colorImageSplit);
            }

          TOCK( "visibilityMaskLevel"  + std::to_string(ii) );
        }
      else
        {
          outputInfoPyramid[i].visibilityMask.resize(outputInfoPyramid[i].meshData.numVertices, true);
          outputInfoPyramid[i].visibilityMaskGT.resize(outputInfoPyramid[i].meshData.numVertices, true);
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
          else
            {
              outputPropPyramid[i].visibilityMask.resize(outputPropPyramid[i].meshData.numVertices, true);
              outputPropPyramid[i].visibilityMaskGT.resize(outputPropPyramid[i].meshData.numVertices, true);
            }
        }
    }

  if(meshLoadingSettings.hasGT)
    setErrorWithGT();

}

void MeshPyramidReader::setErrorWithGT()
{
  for(int i = 0; i < m_nNumMeshLevels; ++i)
    {

      double diff[3];
      double diff_range;
      double sum_square_diff;
      double normalized_diff;
      double minError = std::numeric_limits<double>::max();
      double maxError = std::numeric_limits<double>::min();

      int numVertices = outputInfoPyramid[i].meshData.vertices.size();
      outputInfoPyramid[i].meshData.diffWithGT.resize(numVertices);

      for(int j = 0; j < numVertices; ++j)
        {
          outputInfoPyramid[i].meshData.diffWithGT[j].resize(3);

          for(int k = 0; k < 3; ++k)
            diff[k] = currentMeshPyramidGT.levels[i].vertices[j][k] - currentMeshPyramid.levels[i].vertices[j][k];

          sum_square_diff = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);

          outputInfoPyramid[i].meshData.diffWithGT[j][0] =  sum_square_diff;
          outputInfoPyramid[i].meshData.diffWithGT[j][1] =  0;
          outputInfoPyramid[i].meshData.diffWithGT[j][2] =  0;

          if(minError > sum_square_diff)
            minError = sum_square_diff;
          if(maxError < sum_square_diff)
            maxError = sum_square_diff;
        }

      diff_range = maxError - minError;

      if(diff_range == 0)
        {
          for(int j = 0; j < numVertices; ++j)
            outputInfoPyramid[i].meshData.diffWithGT[j][0] = 0;
        }
      else
        {

          for(int j = 0; j < numVertices; ++j)
            {
              normalized_diff = outputInfoPyramid[i].meshData.diffWithGT[j][0] /diff_range;
              // if(normalized_diff <= 1.0/3)
              //   {
              //     // interpolation between blue and green
              //     outputInfoPyramid[i].meshData.diffWithGT[j][0] = 0;                      // reset red channel
              //     outputInfoPyramid[i].meshData.diffWithGT[j][2] = normalized_diff*3;      // set blue channel
              //   }
              // else if(normalized_diff > 1.0/3 && normalized_diff <= 2.0/3)
              //   {
              //     outputInfoPyramid[i].meshData.diffWithGT[j][0] = 0;                      // reset red channel
              //     outputInfoPyramid[i].meshData.diffWithGT[j][1] = normalized_diff*3 - 1;  // set blue channel
              //   }
              // else if(normalized_diff > 2.0/3)
              //   {
              //     outputInfoPyramid[i].meshData.diffWithGT[j][0] = normalized_diff*3 - 2;  // set red channel
              //   }
              if(normalized_diff <= 1.0/2)
                {
                  // interpolation between blue and green
                  outputInfoPyramid[i].meshData.diffWithGT[j][0] = 0;
                  outputInfoPyramid[i].meshData.diffWithGT[j][1] = normalized_diff*2;
                  outputInfoPyramid[i].meshData.diffWithGT[j][2] = 1-normalized_diff*2;
                }
              else
                {
                  // interpolation between green and red
                  outputInfoPyramid[i].meshData.diffWithGT[j][0] = normalized_diff*2-1;
                  outputInfoPyramid[i].meshData.diffWithGT[j][1] = 2-2*normalized_diff;
                  outputInfoPyramid[i].meshData.diffWithGT[j][2] = 0;
                }

            }
          // normalize the diff of vertices over the whole mesh
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
