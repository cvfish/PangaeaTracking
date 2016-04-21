#include "main_engine/tracker/Mesh.h"

#include "main_engine/rendering/DepthBuffer.h"

void UpdateHelper(MeshData<CoordinateType>& meshData, vector<vector<CoordinateType> >& meshProj, int vertex,
                  CoordinateType trans_uvd[3], CoordinateType trans_normals[3], double KK[3][3])
{

  // update 3d point cloud
  meshData.vertices[ vertex ][ 0 ] = trans_uvd[0];
  meshData.vertices[ vertex ][ 1 ] = trans_uvd[1];
  meshData.vertices[ vertex ][ 2 ] = trans_uvd[2];
  meshData.normals[ vertex ][ 0 ] = trans_normals[0];
  meshData.normals[ vertex ][ 1 ] = trans_normals[1];
  meshData.normals[ vertex ][ 2 ] = trans_normals[2];

  if(KK[0][2] == 0)  // orthographic projection
    {
      meshProj[vertex][0] = (KK[0][0]*trans_uvd[0] + KK[0][1]*trans_uvd[1] + KK[0][2]*trans_uvd[2]);
      meshProj[vertex][1] = (KK[1][0]*trans_uvd[0] + KK[1][1]*trans_uvd[1] + KK[1][2]*trans_uvd[2]);
    }
  else
    {
      CoordinateType temp = (KK[2][0]*trans_uvd[0] + KK[2][1]*trans_uvd[1] + KK[2][2]*trans_uvd[2]);
      meshProj[vertex][0] = (KK[0][0]*trans_uvd[0] + KK[0][1]*trans_uvd[1] + KK[0][2]*trans_uvd[2])/temp;
      meshProj[vertex][1] = (KK[1][0]*trans_uvd[0] + KK[1][1]*trans_uvd[1] + KK[1][2]*trans_uvd[2])/temp;
    }

}

void UpdateRenderingData(TrackerOutputInfo& outputInfo, double KK[3][3],
                         CoordinateType camPose[6], PangaeaMeshData& currentMesh, bool updateGT)
{
  // copy result to output, has to add rotation and translation to make proper visualization
  CoordinateType Rotation[9]; // column major due to ceres implementation
  CoordinateType uvd[3];
  CoordinateType trans_uvd[3];
  CoordinateType normals[3];
  CoordinateType trans_normals[3];

  //ceres::AngleAxisToRotationMatrix(camPose,Rotation);
  Vector3d axis;
  double angle = Map<Vector3d>(camPose).norm();
  if(angle != 0)
    axis = Map<Vector3d>(camPose)/angle;
  else
    axis << 1, 0, 0;

  Map< Matrix3d > rot(Rotation);
  rot = AngleAxisd(angle, axis).toRotationMatrix();

  for(int vertex = 0; vertex < currentMesh.numVertices; ++vertex)
    {
      for(int ind = 0; ind < 3; ++ind)
        {
          uvd[ind] = currentMesh.vertices[ vertex ][ind];
          normals[ind] = currentMesh.normals[ vertex ][ind];
        }

      trans_uvd[0] = uvd[0]*Rotation[0] + uvd[1]*Rotation[3] + uvd[2]*Rotation[6] + camPose[3];
      trans_uvd[1] = uvd[0]*Rotation[1] + uvd[1]*Rotation[4] + uvd[2]*Rotation[7] + camPose[4];
      trans_uvd[2] = uvd[0]*Rotation[2] + uvd[1]*Rotation[5] + uvd[2]*Rotation[8] + camPose[5];
      trans_normals[0] = normals[0]*Rotation[0] + normals[1]*Rotation[3] + normals[2]*Rotation[6];
      trans_normals[1] = normals[0]*Rotation[1] + normals[1]*Rotation[4] + normals[2]*Rotation[7];
      trans_normals[2] = normals[0]*Rotation[2] + normals[1]*Rotation[5] + normals[2]*Rotation[8];

      if(updateGT)
        UpdateHelper(outputInfo.meshDataGT, outputInfo.meshProjGT, vertex, trans_uvd, trans_normals, KK);
      else
        UpdateHelper(outputInfo.meshData, outputInfo.meshProj, vertex, trans_uvd, trans_normals, KK);

    }

  // outputInfo.meshProjGT = outputInfo.meshProj;

  // print out how many points are visible and how many points are occluded
  // int visible_num = 0;
  // int occluded_num = 0;
  // for(int i = 0; i < visibilityMask.size(); ++i)
  // if(visibilityMask[i])
  // visible_num++;
  // else
  // occluded_num++;

  // cout << "visible num: " << visible_num << endl;
  // cout << "occluded num: " << occluded_num << endl;

  // print out how many points are visible and how many points are occluded
  // int visible_num = 0;
  // int occluded_num = 0;
  // for(int i = 0; i < outputInfo.visibilityMask.size(); ++i)
  // if(outputInfo.visibilityMask[i])
  // visible_num++;
  // else
  // occluded_num++;

  // cout << "visible num: " << visible_num << endl;
  // cout << "occluded num: " << occluded_num << endl;

}

void UpdateRenderingData(TrackerOutputInfo& outputInfo, double KK[3][3],
                         CoordinateType camPose[6], PangaeaMeshData& templateMesh,
                         MeshDeformation& mesh_trans, bool updateGT)
{
  // copy result to output, has to add rotation and translation to make proper visualization
  CoordinateType Rotation[9]; // column major due to ceres implementation
  CoordinateType uvd[3];
  CoordinateType trans_uvd[3];
  CoordinateType normals[3];
  CoordinateType trans_normals[3];

  //ceres::AngleAxisToRotationMatrix(camPose,Rotation);
  Vector3d axis;
  double angle = Map<Vector3d>(camPose).norm();
  if(angle != 0)
    axis = Map<Vector3d>(camPose)/angle;
  else
    axis << 1, 0, 0;

  Map< Matrix3d > rot(Rotation);
  rot = AngleAxisd(angle, axis).toRotationMatrix();

  for(int vertex = 0; vertex < templateMesh.numVertices; ++vertex)
    {
      for(int ind = 0; ind < 3; ++ind)
        {
          uvd[ind] = templateMesh.vertices[ vertex ][ind] + mesh_trans[ vertex ][ind];
          normals[ind] = templateMesh.normals[ vertex ][ind];
        }

      trans_uvd[0] = uvd[0]*Rotation[0] + uvd[1]*Rotation[3] + uvd[2]*Rotation[6] + camPose[3];
      trans_uvd[1] = uvd[0]*Rotation[1] + uvd[1]*Rotation[4] + uvd[2]*Rotation[7] + camPose[4];
      trans_uvd[2] = uvd[0]*Rotation[2] + uvd[1]*Rotation[5] + uvd[2]*Rotation[8] + camPose[5];
      trans_normals[0] = normals[0]*Rotation[0] + normals[1]*Rotation[3] + normals[2]*Rotation[6];
      trans_normals[1] = normals[0]*Rotation[1] + normals[1]*Rotation[4] + normals[2]*Rotation[7];
      trans_normals[2] = normals[0]*Rotation[2] + normals[1]*Rotation[5] + normals[2]*Rotation[8];

      if(updateGT)
        UpdateHelper(outputInfo.meshDataGT, outputInfo.meshProjGT, vertex, trans_uvd, trans_normals, KK);
      else
        UpdateHelper(outputInfo.meshData, outputInfo.meshProj, vertex, trans_uvd, trans_normals, KK);

    }
}

void UpdateRenderingDataFast(TrackerOutputInfo& outputInfo, double KK[3][3],
                             PangaeaMeshData& currentMesh, bool updateGT)
{
  // copy result to output, has to add rotation and translation to make proper visualization
  CoordinateType uvd[3];
  CoordinateType normals[3];

  for(int vertex = 0; vertex < currentMesh.numVertices; ++vertex)
    {
      // update 3d point cloud and normals
      uvd[0] = currentMesh.vertices[ vertex ][ 0 ];
      uvd[1] = currentMesh.vertices[ vertex ][ 1 ];
      uvd[2] = currentMesh.vertices[ vertex ][ 2 ];
      normals[0] = currentMesh.normals[ vertex ][ 0 ];
      normals[1] = currentMesh.normals[ vertex ][ 1 ];
      normals[2] = currentMesh.normals[ vertex ][ 2 ];

      if(updateGT)
        UpdateHelper(outputInfo.meshDataGT, outputInfo.meshProjGT, vertex, uvd, normals, KK);
      else
        UpdateHelper(outputInfo.meshData, outputInfo.meshProj, vertex, uvd, normals, KK);

    }
}

void UpdateVisibilityMask(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
                          int width, int height, bool isGT)
{
  static vector< vector<unsigned int> > visibilityFacesTest;

  // visibility mask, used for speed up old rendering code
  // this will set up in the first call
  // will not do anything during the second time
  visibilityFacesTest.resize(width*height);
  for(int i = 0; i < width*height; ++i)
    visibilityFacesTest[i].reserve(5);   // 5 reserved faces for each pixel.

  // image width and height
  int m_nWidth, m_nHeight;
  m_nWidth = width; m_nHeight = height;

  // reset the visibility mask to true first.
  visibilityMask.resize(outputInfo.meshData.numVertices,true);

  // update the visibility mask using previous mesh
  // assign faces to visibilityFacesTest according to 2d projections
  double x1, x2, x3, y1, y2, y3;
  double xmin, xmax, ymin, ymax;
  int xminI, xmaxI, yminI, ymaxI;
  int xx, yy;
  int vertex1, vertex2, vertex3;
  vector< vector<unsigned int> >& meshFaces = outputInfo.meshData.facesVerticesInd;
  vector< vector<CoordinateType> >& meshProj = isGT ? outputInfo.meshProjGT : outputInfo.meshProj;
  vector< vector<CoordinateType> >& meshVertices = isGT ? outputInfo.meshDataGT.vertices :
    outputInfo.meshData.vertices;

	bool clockwise = outputInfo.meshData.clockwise;

  vector<CoordinateType> faceNormals;
  vector<CoordinateType> faceCenters;
  faceNormals.resize(3*outputInfo.meshData.numFaces);
  faceCenters.resize(3*outputInfo.meshData.numFaces);

  for(int faceInd = 0; faceInd < outputInfo.meshData.numFaces; ++faceInd)
    {
      // if(faceInd % 100 == 0)
      // cout << "face id: " << faceInd << endl;

      vertex1 = meshFaces[faceInd][0];
      vertex2 = meshFaces[faceInd][1];
      vertex3 = meshFaces[faceInd][2];

      x1 = meshProj[ vertex1 ][0];
      x2 = meshProj[ vertex2 ][0];
      x3 = meshProj[ vertex3 ][0];
      y1 = meshProj[ vertex1 ][1];
      y2 = meshProj[ vertex2 ][1];
      y3 = meshProj[ vertex3 ][1];
      xmin = min( min( x1, x2), x3);
      xmax = max( max( x1, x2), x3);
      ymin = min( min( y1, y2), y3);
      ymax = max( max( y1, y2), y3);
      // if the projection of the face is outside the image, we give up
      if(xmin > m_nWidth+1 || ymin > m_nHeight+1 ||
         xmax < 0 || ymax < 0)
        continue;

      xminI = max(1.0,floor(xmin));
      yminI = max(1.0,floor(ymin));
      xmaxI = min(ceil(xmax), double(m_nWidth));
      ymaxI = min(ceil(ymax), double(m_nHeight));
      for(xx = xminI - 1; xx < xmaxI; ++xx)
        {
          for(yy = yminI - 1; yy < ymaxI; ++yy)
            {
              visibilityFacesTest[yy*m_nWidth + xx].push_back(faceInd);
            }
        }

      // prepare the face center and face normals for the purpose of gettin
      // intersection between faces and back-projected ray.
      faceCenters[3*faceInd] =  (meshVertices[ vertex1 ][0] +
                                 meshVertices[ vertex2 ][0] +
                                 meshVertices[ vertex3 ][0]) / 3;
      faceCenters[3*faceInd + 1] =  (meshVertices[ vertex1 ][1] +
                                     meshVertices[ vertex2 ][1] +
                                     meshVertices[ vertex3 ][1]) / 3;
      faceCenters[3*faceInd + 2] =  (meshVertices[ vertex1 ][2] +
                                     meshVertices[ vertex2 ][2] +
                                     meshVertices[ vertex3 ][2]) / 3;

      compnorm(&meshVertices[vertex1][0], &meshVertices[vertex2][0],
               &meshVertices[vertex3][0], &faceNormals[3*faceInd], clockwise);

    }

  // // check if the projection of all faces looks reasonable
  // // PixelType* pMaskBuffer = new PixelType[m_nHeight*m_nWidth];
  // // IntensityImageType maskImage(m_nHeight,m_nWidth,pMaskBuffer);
  // IntensityImageType maskImage(m_nHeight,m_nWidth);
  // for(int i = 0; i < m_nHeight; ++i)
  // {
  //     for(int j = 0; j < m_nWidth; ++j)
  //     {
  //         if(visibilityFacesTest[i*m_nWidth+j].size() > 0)
  //         maskImage(i,j) = 255;
  //         else
  //         maskImage(i,j) = 0;
  //     }
  // }
  // // cv::imshow("maskImage",maskImage);
  // // cv::waitKey(0);
  // cv::imwrite("visMaskImage.png",maskImage);
  // // delete[] pMaskBuffer;

  // loop over vertices to find the intersection between backprojected race
  // and the possible faces of the mesh
  double projX, projY;
  int projXminI, projYminI;
  int projXmaxI, projYmaxI;
  int position, faceInd;

  //  cout << "finish faces visibility" << endl;

  for(unsigned int vertexInd = 0; vertexInd < outputInfo.meshData.numVertices;
      ++vertexInd)
    {

      // if(vertexInd % 100 == 0)
      //   cout << "vertex id " << vertexInd << endl;

      // if the projection is outside the image, it is not visible

      if( meshProj[vertexInd][0] < 0 || meshProj[vertexInd][0] > m_nWidth ||
          meshProj[vertexInd][1] < 0 || meshProj[vertexInd][1] > m_nHeight)
        {
          visibilityMask[vertexInd] = false;
          continue;
        }

      projX = meshProj[vertexInd][0];
      projY = meshProj[vertexInd][1];

      projXminI = max(1.0, floor(projX));
      projYminI = max(1.0, floor(projY));

      projXmaxI = min(ceil(projX), double(m_nWidth));
      projYmaxI = min(ceil(projY), double(m_nHeight));

      bool done = false;

      for(xx = projXminI - 1; xx < projXmaxI; ++xx)
        {
          for(yy = projYminI - 1; yy < projYmaxI; ++yy)
            {
              // loop over all the faces here
              position = yy*m_nWidth + xx;
              int facesNum = visibilityFacesTest[position].size();
              for(int k = 0; k < facesNum; ++k)
                {
                  faceInd = visibilityFacesTest[position][k];
                  vertex1 = meshFaces[faceInd][0];
                  vertex2 = meshFaces[faceInd][1];
                  vertex3 = meshFaces[faceInd][2];
                  // check if the vertex belongs to this face
                  if(vertex1 == vertexInd ||
                     vertex2 == vertexInd ||
                     vertex3 == vertexInd)
                    continue;
                  // First test if the projection of the point is inside the triangle,
                  // If not, we continue to the next face
                  if(pointInTriangleTest2(&meshProj[vertexInd][0],
                                          &meshProj[vertex1][0],
                                          &meshProj[vertex2][0],
                                          &meshProj[vertex3][0]))
                    {
                      // compute the intersection between the
                      // backtraced ray of vertexInd and the face faceInd
                      // point, center, normal, point1, point2, point3
                      visibilityMask[vertexInd] = visibilityTest(&meshVertices[vertexInd][0],
                                                                 &faceCenters[3*faceInd],
                                                                 &faceNormals[3*faceInd],
                                                                 &meshVertices[vertex1][0],
                                                                 &meshVertices[vertex2][0],
                                                                 &meshVertices[vertex3][0]);
                    }
                  else
                    continue;

                  // check if the vertex is already occluded or not
                  if(visibilityMask[vertexInd] == false)
                    {
                      done = true;
                      break;
                    }
                }

              if(done)
                break;

            }

          if(done)
            break;

        }

    }

  for(int i = 0; i < visibilityFacesTest.size(); ++i)
    visibilityFacesTest[i].clear();   // 5 reserved faces for each pixel, capacity doesn't change

  if(isGT)
    outputInfo.visibilityMaskGT = visibilityMask;
  else
    outputInfo.visibilityMask = visibilityMask;

  // outputInfo.visibilityMask.resize(visibilityMask.size());
  // for(int i = 0; i < visibilityMask.size(); ++i)
  //   outputInfo.visibilityMask[i] = visibilityMask[i];

  // // print out how many points are visible and how many points are occluded
  // int visible_num = 0;
  // int occluded_num = 0;
  // for(int i = 0; i < visibilityMask.size(); ++i)
  //   if(visibilityMask[i])
  //     visible_num++;
  //   else
  //     occluded_num++;

  // cout << "visible num: " << visible_num << endl;
  // cout << "occluded num: " << occluded_num << endl;

}

void UpdateVisibilityMaskGL(PangaeaMeshData& meshData, vector<bool>& visibilityMask,
                            double KK[3][3], CoordinateType camPose[6], int width,
                            int height, double toleranceRatio)
{

  visibilityMask.resize(meshData.numVertices,true);

  CCamera camera(KK, camPose, width, height);

  DepthBuffer depthBuffer(&camera, &meshData);

  // at this point, check first if we got the depth buffer right

  // next step, do interpolation and compare the values to get visibility
  // cv::Mat cvDepthBuffer(camera.H(), camera.W(), CV_32FC1,
  //     depthBuffer._depthBufferMin.data());
  cv::Mat_<float> cvDepthBuffer(camera.H(), camera.W(),
                                depthBuffer._depthBufferMin.data());

  float depthValue;
  float depthRange = depthBuffer._zMax - depthBuffer._zMin;
  float depthTolerance = depthRange * toleranceRatio;

  for(int i = 0; i < meshData.numVertices; ++i)
    {
      Vector3d point3d = camera.worldToCamera(meshData.vertices[i]);
      Vector2d point2d = camera.worldToPixel(meshData.vertices[i]);
      Vector2f point2f(point2d[0], point2d[1]);
      //Vector2f point2f(outputInfo.meshProj[i][0], outputInfo.meshProj[i][1]);
      SampleLinear(cvDepthBuffer, point2f[1], point2f[0], &depthValue);

      //if(point3d[2] < depthValue)
      if(point3d[2] < depthTolerance + depthValue)
        visibilityMask[i] = true;
      else
        visibilityMask[i] = false;
    }
}

void UpdateVisibilityMaskGL(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
                            double KK[3][3], CoordinateType camPose[6], int width,
                            int height, double toleranceRatio)
{

  UpdateVisibilityMaskGL(outputInfo.meshData, visibilityMask, KK,
                         camPose, width, height, toleranceRatio);

  outputInfo.visibilityMask = visibilityMask;

}

void UpdateColorDiff(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
                     InternalIntensityImageType colorImageSplit[3])
{
  // compute the color difference between mesh and current frame
  for(int i = 0; i < outputInfo.meshData.numVertices; ++i) {
    // only compute the color difference for visible points
    if(visibilityMask[i])
      {
        CoordinateType u = outputInfo.meshProj[i][0];
        CoordinateType v = outputInfo.meshProj[i][1];
        // compute the color for location (u,v)
        CoordinateType interpolate_value[3];
        for(int k = 0; k <3 ; ++k)
          {
            SampleLinear(colorImageSplit[k], v, u, interpolate_value+k);
            outputInfo.meshDataColorDiff.colors[i][k] = abs(interpolate_value[k] -
                                                            outputInfo.meshData.colors[i][k]);
          }
        // // test the values of ground truth color difference
        // cout << outputInfo.meshDataColorDiffGT.colors[i][0] << " "
        //      << abs(interpolate_value[0]) << " "
        //      << outputInfo.meshDataGT.colors[i][0] << endl;

      }
    else // all occluded points set to black
      {
        outputInfo.meshDataColorDiff.colors[i][0] = 0;
        outputInfo.meshDataColorDiff.colors[i][1] = 0;
        outputInfo.meshDataColorDiff.colors[i][2] = 1;
      }
  }
}

void UpdateColorDiffGT(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
                       InternalIntensityImageType colorImageSplit[3])
{
    // compute the color difference between mesh and current frame
  for(int i = 0; i < outputInfo.meshData.numVertices; ++i) {
    // only compute the color difference for visible points
    if(visibilityMask[i])
      {
        CoordinateType u = outputInfo.meshProjGT[i][0];
        CoordinateType v = outputInfo.meshProjGT[i][1];
        // compute the color for location (u,v)
        CoordinateType interpolate_value[3];

        for(int k = 0; k <3 ; ++k)
          {
            SampleLinear(colorImageSplit[k], v, u, interpolate_value+k);
            outputInfo.meshDataColorDiffGT.colors[i][k] = abs(interpolate_value[k] -
                                                              outputInfo.meshDataGT.colors[i][k]);
          }
      }
    else // all occluded points set to black
      {
        outputInfo.meshDataColorDiffGT.colors[i][0] = 0;
        outputInfo.meshDataColorDiffGT.colors[i][1] = 0;
        outputInfo.meshDataColorDiffGT.colors[i][2] = 1;
      }
  }
}

void UpdateFeatureDiff(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
                       FeatureImageType& featureImage)
{
  for(int i = 0; i < outputInfo.meshData.numVertices; ++i) {

    if(visibilityMask[i])
      {
        CoordinateType u = outputInfo.meshProj[i][0];
        CoordinateType v = outputInfo.meshProj[i][1];

        // compute the color for location (u,v)
        CoordinateType interpolate_value;

        SampleLinear(featureImage, v, u, &interpolate_value);

        outputInfo.meshDataFeatDiff.colors[i][0] = abs(interpolate_value - outputInfo.meshData.featuresBuffer[i][0]);
        outputInfo.meshDataFeatDiff.colors[i][1] = outputInfo.meshDataFeatDiff.colors[i][0];
        outputInfo.meshDataFeatDiff.colors[i][2] = outputInfo.meshDataFeatDiff.colors[i][0];

        // // test the values of ground truth color difference
        // cout << outputInfo.meshDataColorDiffGT.colors[i][0] << " "
        //      << abs(interpolate_value[0]) << " "
        //      << outputInfo.meshDataGT.colors[i][0] << endl;

      }
    else // all occluded points set to black
      {
        outputInfo.meshDataFeatDiff.colors[i][0] = 0;
        outputInfo.meshDataFeatDiff.colors[i][1] = 0;
        outputInfo.meshDataFeatDiff.colors[i][2] = 1;
      }

  }

}

void UpdateFeatureDiffGT(TrackerOutputInfo& outputInfo, vector<bool>& visibilityMask,
                         FeatureImageType& featureImage)
{

    for(int i = 0; i < outputInfo.meshData.numVertices; ++i) {

    if(visibilityMask[i])
      {
        CoordinateType u = outputInfo.meshProj[i][0];
        CoordinateType v = outputInfo.meshProj[i][1];

        // compute the color for location (u,v)
        CoordinateType interpolate_value;

        SampleLinear(featureImage, v, u, &interpolate_value);

        outputInfo.meshDataFeatDiffGT.colors[i][0] = abs(interpolate_value - outputInfo.meshDataGT.featuresBuffer[i][0]);
        outputInfo.meshDataFeatDiffGT.colors[i][1] = outputInfo.meshDataFeatDiffGT.colors[i][0];
        outputInfo.meshDataFeatDiffGT.colors[i][2] = outputInfo.meshDataFeatDiffGT.colors[i][0];

        // // test the values of ground truth color difference
        // cout << outputInfo.meshDataColorDiffGT.colors[i][0] << " "
        //      << abs(interpolate_value[0]) << " "
        //      << outputInfo.meshDataGT.colors[i][0] << endl;

      }
    else // all occluded points set to black
      {
        outputInfo.meshDataFeatDiffGT.colors[i][0] = 0;
        outputInfo.meshDataFeatDiffGT.colors[i][1] = 0;
        outputInfo.meshDataFeatDiffGT.colors[i][2] = 1;
      }

  }

}
