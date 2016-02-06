#include "main_engine/tracker/DeformNRSFMTracker.h"

baType mapBA(std::string const& inString) {
  if (inString == "mot") return BA_MOT;
  if (inString == "str") return BA_STR;
  if (inString == "motstr") return BA_MOTSTR;
}

dataTermErrorType mapErrorType(std::string const& inString){
  if(inString == "gray") return PE_INTENSITY;
  if(inString == "color") return PE_COLOR;
  if(inString == "depth") return PE_DEPTH;
  if(inString == "depth_plane") return PE_DEPTH_PLANE;
  if(inString == "ncc") return PE_NCC;
  if(inString == "color_ncc") return PE_COLOR_NCC;
}

// Linear Solver mapping
ceres::LinearSolverType mapLinearSolver(std::string const& inString){
  if(inString == "SNC") return ceres::SPARSE_NORMAL_CHOLESKY;
  if(inString == "DNC") return ceres::DENSE_NORMAL_CHOLESKY;
  if(inString == "DS") return  ceres::DENSE_SCHUR;
  if(inString == "SS") return  ceres::SPARSE_SCHUR;
  if(inString == "IS") return  ceres::ITERATIVE_SCHUR;
  if(inString == "CG") return  ceres::CGNR;
  if(inString == "DQR") return ceres::DENSE_QR;
}

ceres::MinimizerType mapMinimizerType(std::string const& inString){
  if(inString == "TRUST_REGION") return ceres::TRUST_REGION;
  if(inString == "LINE_SEARCH") return ceres::LINE_SEARCH;
}

ceres::LineSearchDirectionType mapLineSearchDirectionType(std::string const& inString){
  if(inString == "STEEPEST_DESCENT") return ceres::STEEPEST_DESCENT;
  if(inString == "NONLINEAR_CONJUGATE_GRADIENT") return ceres::NONLINEAR_CONJUGATE_GRADIENT;
  if(inString == "LBFGS") return ceres::LBFGS;
  if(inString == "BFGS") return ceres::BFGS;
}

ceres::LineSearchType mapLineSearchType(std::string const& inString){
  if(inString == "ARMIJO") return ceres::ARMIJO;
  if(inString == "WOLFE") return ceres::WOLFE;
}

ceres::LineSearchInterpolationType mapLineSearchInterpType(std::string const& inString){
  if(inString == "BISECTION") return ceres::BISECTION;
  if(inString == "QUADRATIC") return ceres::QUADRATIC;
  if(inString == "CUBIC") return ceres::CUBIC;
}

ceres::NonlinearConjugateGradientType mapNonLinearCGType(std::string const& inString){
  if(inString == "FLETCHER_REEVES") return ceres::FLETCHER_REEVES;
  if(inString == "POLAK_RIBIERE") return ceres::POLAK_RIBIERE;
  if(inString == "HESTENES_STIEFEL") return ceres::HESTENES_STIEFEL;
}

DeformNRSFMTracker::DeformNRSFMTracker(TrackerSettings& settings, int width, int height, double K[3][3],
                                       int startFrame, int numTrackingFrames):
  trackerInitialized(false),
  preProcessingThread(NULL),
  savingThread(NULL),
  dataInBuffer(false),
  useProblemWrapper(false)
{
  BAType = mapBA(settings.baType);
  PEType = mapErrorType(settings.errorType);

  m_nWidth = width;
  m_nHeight = height;
  startFrameNo = startFrame;

  // setting up cameras
  setIntrinsicMatrix(K);
  initializeCamera();

  // debug info output
  std::stringstream ceresOutputPath;
  ceresOutputPath << settings.savePath << settings.ceresOutputFile;
  ceresOutput.open(ceresOutputPath.str().c_str(), std::ofstream::trunc);

  // cout << ceresOutputPath.str() << endl;

  pImagePyramid = new ImagePyramid;
  pFeaturePyramid = new FeaturePyramid;

}

DeformNRSFMTracker::~DeformNRSFMTracker()
{
  if(preProcessingThread != NULL)
    preProcessingThread->join();

  if(savingThread != NULL)
    savingThread->join();

  if(pImagePyramid) delete pImagePyramid;
  if(pFeaturePyramid) delete pFeaturePyramid;

  if(pStrategy) delete pStrategy;

}

bool DeformNRSFMTracker::setCurrentFrame(int curFrame)
{
  //cannot really do this for a real tracker
  return true;
}

void DeformNRSFMTracker::setIntrinsicMatrix(double K[3][3])
{
  for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
        {
          KK[i][j] = K[i][j];
        }
    }
}

void DeformNRSFMTracker::initializeCamera()
{
  camInfo.isOrthoCamera = trackerSettings.isOrthoCamera;
  memcpy(camInfo.KK, KK, 9*sizeof(double));
  memset(camInfo.invKK, 0, 9*sizeof(double));
  // set up invKK
  camInfo.invKK[0][0] = 1/KK[0][0];
  camInfo.invKK[1][1] = 1/KK[1][1];
  camInfo.invKK[0][2] = -KK[0][2] * camInfo.invKK[0][0];
  camInfo.invKK[1][2] = -KK[1][2] * camInfo.invKK[1][1];
  camInfo.invKK[2][2] = 1;

  camInfo.width = m_nWidth;
  camInfo.height = m_nHeight;

  memset(camPose, 0, 6*sizeof(double));
}

void DeformNRSFMTracker::setInitialMeshPyramid(PangaeaMeshPyramid& initMeshPyramid)
{

  templateMeshPyramid = std::move(initMeshPyramid);

  m_nMeshLevels = templateMeshPyramid.numLevels;

  // create the optimization strategy and setting up corresponding parameters
  pStrategy = new FreeNeighborStrategy(m_nMeshLevels);

  // set up the optimization strategy
  pStrategy->Initialize();

  // initialize problemWrapper
  problemWrapper.Initialize( pStrategy->numOptimizationLevels );

  // setting parameters
  WeightPara weightPara;
  weightPara.dataTermWeight     = trackerSettings.weightPhotometric;
  weightPara.tvTermWeight       = trackerSettings.weightTV;
  weightPara.tvRotTermWeight    = trackerSettings.weightRotTV;
  weightPara.deformWeight       = trackerSettings.weightDeform;
  weightPara.arapTermWeight     = trackerSettings.weightARAP;
  weightPara.inextentTermWeight = trackerSettings.weightINEXTENT;
  weightPara.transWeight        = trackerSettings.weightTransPrior;
  weightPara.rotWeight = 0;

  //    weightPara.dataHuberWidth = trackerSettings.dataHuberWidth;
  weightPara.dataHuberWidth  = trackerSettings.photometricHuberWidth;
  weightPara.tvHuberWidth    = trackerSettings.tvHuberWidth;
  weightPara.tvRotHuberWidth = trackerSettings.tvRotHuberWidth;

  weightPara.featureTermWeight = featureSettings.featureTermWeight;
  weightPara.featureHuberWidth = featureSettings.featureHuberWidth;

  // setting weights of the 0th level of the pyramid
  pStrategy->setWeightParameters(weightPara);

  // setting weighting scales over different levels of the pyramid
  //pStrategy->setWeightScale(trackerSettings.meshVertexNum);
  pStrategy->setWeightScale(templateMeshPyramid.meshPyramidVertexNum);

  pStrategy->setWeightParametersVec();

  // setup propagation pyramid
  // check the way of converting distances to weights
  setupPropagation(
                   templateMeshPyramid,
                   meshPropagation,
                   trackerSettings.meshNeighborNum,
                   trackerSettings.meshNeighborRadius,
                   trackerSettings.meshPyramidUseRadius);

  // now we need to add those different pairs necessarily
  for(int i = 0; i < pStrategy->numOptimizationLevels; ++i)
    {
      vector<std::pair<int, int> >& dataTermPairs = pStrategy->optimizationSettings[i].dataTermPairs;
      vector<std::pair<int, int> >& regTermPairs = pStrategy->optimizationSettings[i].regTermPairs;

      AddMeshToMeshPropagation(
                               templateMeshPyramid,
                               dataTermPairs,
                               meshPropagation,
                               trackerSettings.meshNeighborNum,
                               trackerSettings.meshNeighborRadius);

      AddMeshToMeshPropagation(
                               templateMeshPyramid,
                               regTermPairs,
                               meshPropagation,
                               trackerSettings.meshNeighborNum,
                               trackerSettings.meshNeighborRadius);
    }

  // print everything used in meshPropagation
  for(std::map< pair<int, int>, int>::iterator it = meshPropagation.neighborMap.begin();
      it != meshPropagation.neighborMap.end(); ++it)
    {
      std::cout << "(" << it->first.first << "," << it->first.second << ")"
                << " => " << it->second << '\n';
    }

  // setup patch neighbors
  setupPatchNeighbor(
                     templateMeshPyramid,
                     meshPropagation,
                     trackerSettings.neighborPatchRadius);

  // imagePyramid will be created during the processing of the first image
  pImagePyramid->create(m_nWidth, m_nHeight);
  pImagePyramid->setupCameraPyramid(m_nMeshLevels, camInfo);

  // setup the feature pyramid
  if(trackerSettings.useFeatureImages)
    {
      pFeaturePyramid->create(m_nWidth / featureSettings.scalingFactor,
                              m_nHeight / featureSettings.scalingFactor,
                              featureSettings.channels,
                              m_nMeshLevels);
      pFeaturePyramid->setupCameraPyramid(m_nMeshLevels, camInfo);
      pFeaturePyramid->InitializeDB( featureSettings.dbPath.c_str() );

      //need to update the number of channels for feature residuals
      PE_RESIDUAL_NUM_ARRAY[PE_FEATURE] = featureSettings.channels;
      PE_RESIDUAL_NUM_ARRAY[PE_FEATURE_NCC] = featureSettings.channels;
    }

  // setup visibilitymask pyramid
  visibilityMaskPyramid.resize(m_nMeshLevels);
  meshTransPyramid.resize(m_nMeshLevels);
  meshRotPyramid.resize(m_nMeshLevels);

  prevMeshTransPyramid.resize(m_nMeshLevels);
  prevMeshRotPyramid.resize(m_nMeshLevels);

  outputInfoPyramid.resize(m_nMeshLevels);
  outputPropPyramid.resize(m_nMeshLevels);

  int imageSize = m_nWidth*m_nHeight;
  for(int i = 0; i < m_nMeshLevels; ++i)
    {

      int numVertices = templateMeshPyramid.levels[i].numVertices;
      visibilityMaskPyramid[i].resize(numVertices,true);

      meshTransPyramid[i].resize(numVertices);
      meshRotPyramid[i].resize(numVertices);

      vector<CoordinateType> zeros3D;
      zeros3D.resize(3); zeros3D[0] = 0; zeros3D[1] = 0; zeros3D[2] = 0;

      meshTransPyramid[i].resize(numVertices, zeros3D);
      meshRotPyramid[i].resize(numVertices, zeros3D);

      prevMeshTransPyramid[i].resize(numVertices, zeros3D);
      prevMeshRotPyramid[i].resize(numVertices, zeros3D);

      for(int j = 0; j < numVertices; ++j)
        {
          meshTransPyramid[i][j].resize(3,0);
          meshRotPyramid[i][j].resize(3,0);
          prevMeshTransPyramid[i][j].resize(3,0);
          prevMeshRotPyramid[i][j].resize(3,0);
        }

      outputInfoPyramid[i].meshData = templateMeshPyramid.levels[i];
      outputInfoPyramid[i].meshDataGT = templateMeshPyramid.levels[i];
      outputInfoPyramid[i].meshDataColorDiff = templateMeshPyramid.levels[i];

      outputInfoPyramid[i].nRenderLevel = i;

      vector<CoordinateType> proj2D;
      proj2D.resize(2); proj2D[0] = 0; proj2D[1] = 0;

      outputInfoPyramid[i].meshProj.resize(numVertices, proj2D);
      outputInfoPyramid[i].meshProjGT = outputInfoPyramid[i].meshProj;

      outputInfoPyramid[i].visibilityMask.resize(numVertices, true);

      memset(outputInfoPyramid[i].camPose, 0, 6*sizeof(double));

      // get the outputInfo and visibilityMask of the first frame
      UpdateRenderingData(outputInfoPyramid[i], KK, camPose, templateMeshPyramid.levels[i]);

      // initialize outputPropPyramid as the same with outInfoPyramid
      outputPropPyramid[i] = outputInfoPyramid[i];

      // update the visibility of each vertex
      if(trackerSettings.useVisibilityMask)
        {

          long long int ii = i;

          TICK( "visibilityMask" + std::to_string(ii) );

          if(trackerSettings.useOpenGLMask)
            {
              double tempCamPose[6] = {0,0,0,0,0,0};
              UpdateVisibilityMaskGL(outputInfoPyramid[i], visibilityMaskPyramid[i], KK, tempCamPose, m_nWidth, m_nHeight);
            }
          else
            {
              UpdateVisibilityMask(outputInfoPyramid[i], visibilityMaskPyramid[i], m_nWidth, m_nHeight);
            }

          TOCK( "visibilityMask" + std::to_string(ii) );

        }

    }

  trackerInitialized = true;

}

bool DeformNRSFMTracker::trackFrame(int nFrame, unsigned char* pColorImageRGB,
                                    TrackerOutputInfo** pOutputInfoRendering)
{
  if(!trackerInitialized)
    cout << "this tracker has been initialized with a template mesh" << endl;

  currentFrameNo = nFrame;

  // if(currentFrameNo == 1)
  // {
  //     // output initialization
  //     cout << "intializing outputInfoRending" << endl;
  //     outputInfoRendering = outputInfoPyramid[0];
  // }

  // copy over previous camera pose
  memcpy(prevCamPose, camPose, 6*sizeof(double));

  TICK("imagePreprocessing");
  // update the image pyramid, including images and gradients
  // also depth and normals if there is anything related
  //imagePyramid.setupPyramid(pColorImageRGB, m_nMeshLevels);

  // if(preProcessingThread == NULL)
  //   {
  //     preProcessingThread = new boost::thread(
  //                                             boost::bind(&ImagePyramid::setupPyramid,
  //                                                         pImagePyramidBuffer,
  //                                                         pColorImageRGB,
  //                                                         m_nMeshLevels));
  //     preProcessingThread->join();
  //     dataInBuffer = true;
  //     ImagePyramid* temp = pImagePyramid;
  //     pImagePyramid = pImagePyramidBuffer;
  //     pImagePyramidBuffer = temp;
  //     dataInBuffer = false;
  //   }
  // else
  //   {
  //     preProcessingThread->join();

  //     TICK("assignmentTime");
  //     if(dataInBuffer)
  //       {
  //         ImagePyramid* temp = pImagePyramid;
  //         pImagePyramid = pImagePyramidBuffer;
  //         pImagePyramidBuffer = temp;
  //         dataInBuffer = false;
  //       }
  //     TOCK("assignmentTime");

  //     delete preProcessingThread;
  //     preProcessingThread = new boost::thread(
  //                                             boost::bind(&ImagePyramid::setupPyramid,
  //                                                         pImagePyramidBuffer,
  //                                                         pColorImageRGB,
  //                                                         m_nMeshLevels));
  //     dataInBuffer = true;

  //   }

  // prepare data in buffer
  pImagePyramid->setupPyramid(pColorImageRGB, m_nMeshLevels);
  // get new data from buffer
  pImagePyramid->updateData();

  TOCK("imagePreprocessing");

  if(trackerSettings.useFeatureImages && featureSettings.featureTermWeight > 0)
    {

      TICK("featurePreprocessing");

      char buffer[BUFFER_SIZE];
      sprintf(buffer, featureSettings.keyNameFormat.c_str(), currentFrameNo);

      // if(featureThread == NULL){
      //     featureThread = new boost::thread(boost::bind(&FeaturePyramid::setupPyramid,
      //                                                   pFeaturePyramid,
      //                                                   string(buffer)));
      //     featureThread->join();
      //   }
      // else{
      //   featureThread->join();
      //   delete featureThread;
      //   featureThread = new boost::thread(boost::bind(&FeaturePyramid::setupPyramid,
      //                                                 pFeaturePyramid,
      //                                                 string(buffer)));
      // }

      // prepare data in buffer
      pFeaturePyramid->setupPyramid(string(buffer));
      // get new data from buffer
      pFeaturePyramid->updateData();

      AttachFeaturesToMeshPyramid();
      templateMeshPyramid.swapFeatures();

      TOCK("featurePreprocessing");

    }


  int numOptimizationLevels = pStrategy->numOptimizationLevels;

  // how many levels to do optimization on ?
  for(int i = numOptimizationLevels - 1; i >= 0; --i)
    {

      long long int ii = i;

      TICK( "trackingTimeLevel" + std::to_string(ii) );

      currLevel = i;
      // start tracking
      // create the optimization problem we are trying to solve
      // should make problem a member of DeformNRSFMTracker to
      // avoid the same memory allocation every frame, could take
      // seconds to allocate memory for each frame
      ceres::Problem problem;
      // ceres::Problem& problem = problemWrapper.getProblem(i);
      // useProblemWrapper = true;

      TICK( "trackingTimeLevel" + std::to_string(ii)  + "::ProblemSetup");
      EnergySetup(problem);
      TOCK( "trackingTimeLevel" + std::to_string(ii)  + "::ProblemSetup");

      TICK( "trackingTimeLevel" + std::to_string(ii)  + "::ProblemMinimization");
      EnergyMinimization(problem);
      TOCK( "trackingTimeLevel" + std::to_string(ii)  + "::ProblemMinimization");

      if(trackerSettings.useRGBImages && trackerSettings.weightPhotometric > 0)
        {
          TICK( "trackingTimeLevel" + std::to_string(ii)  + "::RemoveDataTermResidual");
          //remove dataTermResidualBlocks from previous frame
          for(int residualID = 0; residualID < dataTermResidualBlocks.size(); ++residualID)
            problem.RemoveResidualBlock(dataTermResidualBlocks[ residualID ]);
          dataTermResidualBlocks.resize(0);
          TOCK( "trackingTimeLevel" + std::to_string(ii)  + "::RemoveDataTermResidual");
        }

      if(trackerSettings.useFeatureImages && featureSettings.featureTermWeight > 0)
        {
          TICK( "trackingTimeLevel" + std::to_string(ii)  + "::RemoveFeatureTermResidual");
          // remove featureTermResidualBlocks from previous frame
          // be careful if we want to use multi-threading

          for(int residualID = 0; residualID < featureTermResidualBlocks.size(); ++residualID)
            problem.RemoveResidualBlock(featureTermResidualBlocks[ residualID ]);
          featureTermResidualBlocks.resize(0);
          TOCK( "trackingTimeLevel" + std::to_string(ii)  + "::RemoveFeatureTermResidual");
        }

      // at this point we've finished the optimization on level i
      // now we need to update all the results and propagate the optimization
      // results to next level if necessary
      // the first step is update the current results

      TICK( "trackingTimeLevel" + std::to_string(ii)  + "::UpdateResults");
      UpdateResults();
      TOCK( "trackingTimeLevel" + std::to_string(ii)  + "::UpdateResults");

      //*pOutputInfoRendering = &outputInfoPyramid[i];
      //updateRenderingLevel(pOutputInfoRendering, i);
      TICK( "trackingTimeLevel" + std::to_string(ii)  + "::PropagateMesh");
      PropagateMesh();
      TOCK( "trackingTimeLevel" + std::to_string(ii)  + "::PropagateMesh");

      TOCK( "trackingTimeLevel" + std::to_string(ii) );

    }

  TICK("updateProp");
  // update the results
  updateRenderingLevel(pOutputInfoRendering, 0);
  //*pOutputInfoRendering = &outputInfoPyramid[0];

  // update the top level of propagation result
  outputPropPyramid[m_nMeshLevels-1] = outputInfoPyramid[m_nMeshLevels-1];
  TOCK("updateProp");

  //save data
  // TICK("SavingTime");

  // if(savingThread == NULL)
  // {
  //     savingThread = new boost::thread(boost::bind(&DeformNRSFMTracker::SaveThread, this, pOutputInfoRendering) );
  //     savingThread->join();
  // }else
  // {
  //     savingThread->join();
  //     delete savingThread;
  //     savingThread = new boost::thread(boost::bind(&DeformNRSFMTracker::SaveThread, this, pOutputInfoRendering) );
  // }

  // TOCK("SavingTime");
  // // simply return true;

  SaveThread(pOutputInfoRendering);

  if(trackerSettings.useFeatureImages && featureSettings.featureTermWeight > 0)
    pFeaturePyramid->updatePrev();

  return true;
}

void DeformNRSFMTracker::AddVariableMask(ceres::Problem& problem, baType BA)
{
  switch(BA)
    {
    case BA_MOT:
      {
        problem.SetParameterBlockVariable(&camPose[0]);
        problem.SetParameterBlockVariable(&camPose[3]);
        break;
      }
    case BA_STR:
      {
        vector<double*> parameter_blocks;
        problem.GetParameterBlocks(&parameter_blocks);
        // loop over all the parameter blocks
        // and we set all parameter blocks except camPose[0] and camPose[3] constant
        for(int i = 0; i < parameter_blocks.size(); ++i)
          {
            if(parameter_blocks[i] != &camPose[0]
               && parameter_blocks[i] != &camPose[3])
              problem.SetParameterBlockVariable( parameter_blocks[i] );
          }
        break;
      }
    }
}


void DeformNRSFMTracker::AddConstantMask(ceres::Problem& problem, baType BA)
{
  switch(BA)
    {
    case BA_MOT:
      {
        problem.SetParameterBlockConstant(&camPose[0]);
        problem.SetParameterBlockConstant(&camPose[3]);
        break;
      }
    case BA_STR:
      {
        vector<double*> parameter_blocks;
        problem.GetParameterBlocks(&parameter_blocks);
        // loop over all the parameter blocks
        // and we set all parameter blocks except camPose[0] and camPose[3] constant
        for(int i = 0; i < parameter_blocks.size(); ++i)
          {
            if(parameter_blocks[i] != &camPose[0]
               && parameter_blocks[i] != &camPose[3])
              problem.SetParameterBlockConstant( parameter_blocks[i] );
          }
        break;
      }
    }
}

void DeformNRSFMTracker::UpdateResultsLevel(int level)
{
  PangaeaMeshData& template_mesh = templateMeshPyramid.levels[level];

  MeshDeformation& mesh_trans = meshTransPyramid[level];
  MeshDeformation& prev_mesh_trans = prevMeshTransPyramid[level];

  MeshDeformation& mesh_rot = meshRotPyramid[level];
  MeshDeformation& prev_mesh_rot = prevMeshRotPyramid[level];

  // update output results
  TrackerOutputInfo& output_info = outputInfoPyramid[level];

  long long int ii = level;

  // output result for rendering
  TICK( "updateRenderingLevel" + std::to_string( ii ) );

  UpdateRenderingData(output_info, KK, camPose, template_mesh, mesh_trans);

  // compute normals
  if(trackerSettings.loadMesh)
    output_info.meshData.computeNormalsNeil();
  else
    output_info.meshData.computeNormals();

  TOCK( "updateRenderingLevel" + std::to_string( ii ) );

  vector<bool>& visibility_mask = visibilityMaskPyramid[level];
  // update visibility mask if necessary
  if(trackerSettings.useVisibilityMask)
    {

      TICK( "updateVisbilityMaskLevel" + std::to_string( ii ) );

      if(trackerSettings.useOpenGLMask)
        {
          double tempCamPose[6] = {0,0,0,0,0,0};
          cout << "opengl visibility test" << endl;
          UpdateVisibilityMaskGL(output_info, visibility_mask, KK, tempCamPose, m_nWidth, m_nHeight);
        }
      else
        {
          UpdateVisibilityMask(output_info, visibility_mask, m_nWidth, m_nHeight);
        }

      TOCK( "updateVisbilityMaskLevel" + std::to_string( ii ) );
    }


  // need to update the color diff
  InternalIntensityImageType* color_image_split = pImagePyramid->getColorImageSplit(level);


  UpdateColorDiff(output_info, visibility_mask, color_image_split);

  // update previous deformation
  for(int i = 0; i < mesh_trans.size(); ++i)
    {
      prev_mesh_trans[i][0] = mesh_trans[i][0];
      prev_mesh_trans[i][1] = mesh_trans[i][1];
      prev_mesh_trans[i][2] = mesh_trans[i][2];

      prev_mesh_rot[i][0] = mesh_rot[i][0];
      prev_mesh_rot[i][1] = mesh_rot[i][1];
      prev_mesh_rot[i][2] = mesh_rot[i][2];
    }

}

void DeformNRSFMTracker::UpdateResults()
{
  // a few things need to update
  // update the mesh if we've used any interpolation in the data term
  // update the translation field
  // update the normals of optimized mesh
  // update rendering results

  vector<std::pair<int,int> >& data_pairs =
    pStrategy->optimizationSettings[currLevel].dataTermPairs;
  int num_data_pairs = data_pairs.size();

  for(int i = 0; i < num_data_pairs; ++i)
    {
      std::pair<int, int>& data_pair = data_pairs[i];
      UpdateResultsLevel(data_pair.second);

      cout << "updateResultsLevel " << data_pair.second << endl;

      // if(data_pair.second != data_pair.first)
      // {
      //     UpdateResultsLevel(data_pair.first);
      //     cout << "updateResultsLevel " << data_pair.first << endl;
      // }
    }

}

void DeformNRSFMTracker::PropagateMeshCoarseToFine(int coarse_level, int fine_level)
{
  MeshDeformation& mesh_rot = meshRotPyramid[coarse_level];
  MeshDeformation& mesh_trans = meshTransPyramid[coarse_level];

  MeshDeformation& mesh_rot_fine = meshRotPyramid[fine_level];
  MeshDeformation& mesh_trans_fine = meshTransPyramid[fine_level];

  pair<int, int> meshPair( fine_level, coarse_level );
  MeshNeighbors&  neighbors = meshPropagation.getNeighbors( meshPair );
  MeshWeights& weights = meshPropagation.getWeights( meshPair );

  PangaeaMeshData& template_coarse_mesh = templateMeshPyramid.levels[coarse_level];
  PangaeaMeshData& template_fine_mesh = templateMeshPyramid.levels[fine_level];

  // For Siggraph14 case, we to do propagation from coarse level to next
  // fine level, if rotations are among the optimization variables
  // (arap coeffcient is not zero or rotation is used in data term) we use
  // rotaton to do propagation otherwise do interpolation
  // check if arap coeffcient is zero
  // notice that for dynamicFusion case, this is not consistent with dual quarternion interpolation
  // anyway just for an initialization
  WeightPara& weight_para = pStrategy->weightPara;
  WeightScale& weight_scale = pStrategy->weightScale;

  WeightPara weight_para_level;

  weight_para_level.arapTermWeight = weight_para.arapTermWeight * weight_scale.arapTermScale[currLevel];

  // to be updated, check if rotation is used in data term as well
  if(weight_para_level.arapTermWeight > 0)
    {
      double temp_vertex[3];
      double diff_vertex[3];
      double rot_diff_vertex[3];

      //#pragma omp parallel for
      for(int i = 0; i < template_fine_mesh.numVertices; ++i)
        {
          temp_vertex[0] = 0;
          temp_vertex[1] = 0;
          temp_vertex[2] = 0;
          // find its neighbors in coarse_mesh
          int num_neighbors = neighbors[i].size();
          for(int j = 0; j < num_neighbors; ++j)
            {
              for(int index = 0; index < 3; ++index)
                diff_vertex[index] = template_fine_mesh.vertices[i][index] - template_coarse_mesh.vertices[ neighbors[i][j] ][index];

              ceres::AngleAxisRotatePoint(&mesh_rot[ neighbors[i][j] ][0], diff_vertex, rot_diff_vertex);

              for(int index = 0; index < 3; ++index)
                {
                  temp_vertex[index] += weights[i][j] *
                    (rot_diff_vertex[index] + template_coarse_mesh.vertices[ neighbors[i][j] ][index] + mesh_trans[ neighbors[i][j] ][index]);
                }
            }

          for(int j = 0; j < 3; ++j)
            mesh_trans_fine[i][j] = temp_vertex[j] - template_fine_mesh.vertices[i][j];

          // need to update the rotations of the fine mesh as well
          // compute the rigid transformation between two sets of
          // neighboring points(both are defined on the fine mesh)
          // notice that for siggraph14 optimization, the arap edges
          // are defined on the same level
          //
          vector<double> arap_weights;
          arap_weights.resize( template_fine_mesh.adjVerticesInd[i].size(), 1 );

          computeRot(
                     template_fine_mesh.vertices[i],
                     mesh_trans_fine[i],
                     template_fine_mesh.vertices,
                     mesh_trans_fine,
                     template_fine_mesh.adjVerticesInd[i],
                     arap_weights,
                     mesh_rot_fine[i],
                     true);

        }

    }
  else
    {
      // just do interpolation
      for(int i = 0; i < template_fine_mesh.numVertices; ++i)
        {
          mesh_trans_fine[i][0] = 0;
          mesh_trans_fine[i][1] = 0;
          mesh_trans_fine[i][2] = 0;
          // find its neighbors in coarse_mesh
          int num_neighbors = neighbors[i].size();
          for(int j = 0; j < num_neighbors; ++j)
            {
              for(int index = 0; index < 3; ++index)
                {
                  mesh_trans_fine[i][index] += weights[i][j] * mesh_trans[ neighbors[i][j] ][index];
                }
            }
        }

    }
}

void DeformNRSFMTracker::PropagateMesh()
{

  // propagate the mesh from coarse level to fine level,
  // nothing needs to be done if this is the lowest level
  vector<std::pair<int, int> >& prop_pairs =
    pStrategy->optimizationSettings[currLevel].propPairs;
  int num_prop_pairs = prop_pairs.size();

  for(int k = 0; k < num_prop_pairs; ++k)
    {
      std::pair<int, int>& prop_pair = prop_pairs[k];
      int coarse_level = prop_pair.first;
      int fine_level = prop_pair.second;

      PropagateMeshCoarseToFine(coarse_level, fine_level);

      // after propagation, we update the correponding mesh
      UpdateResultsLevel(fine_level);

      // copy over the mesh right after propagation to outputPropPyramid
      outputPropPyramid[fine_level] = outputInfoPyramid[fine_level];

      cout << "prop pairs" << endl;
      cout << coarse_level << "->" << fine_level << endl;

      cout << "updateResultsLevel " << fine_level << endl;

    }

  // if this is level zero, we propagate the mesh to all levels below
  if( currLevel == 0)
    {
      vector<std::pair<int, int> >& final_pairs = pStrategy->propPairsFinal;
      cout << "final prop" << endl;
      for(int k = 0; k < final_pairs.size(); ++k)
        {
          PropagateMeshCoarseToFine(final_pairs[k].first, final_pairs[k].second);
          UpdateResultsLevel(final_pairs[k].second);
          cout << final_pairs[k].first << "->" << final_pairs[k].second << endl;
        }
    }

}

void DeformNRSFMTracker::AddCostImageProjection(ceres::Problem& problem,
                                                ceres::LossFunction* loss_function,
                                                dataTermErrorType errorType,
                                                PangaeaMeshData& templateMesh,
                                                MeshDeformation& meshTrans,
                                                vector<bool>& visibilityMask,
                                                CameraInfo* pCamera,
                                                Level* pFrame)
{
  for(int i = 0; i < templateMesh.numVertices; ++i){

    if(visibilityMask[i])
      {
        switch(errorType)
          {
          case PE_INTENSITY:

            {
              ResidualImageProjection* pResidual = new ResidualImageProjection(1,
                                                                               &templateMesh.grays[i],
                                                                               &templateMesh.vertices[i][0],
                                                                               pCamera,
                                                                               pFrame,
                                                                               errorType);

              ceres::AutoDiffCostFunction<ResidualImageProjection, 1, 3, 3, 3>* cost_function =
                new ceres::AutoDiffCostFunction<ResidualImageProjection, 1, 3, 3, 3>( pResidual );

              dataTermResidualBlocks.push_back(
                                               problem.AddResidualBlock(
                                                                        cost_function,
                                                                        loss_function,
                                                                        &camPose[0],
                                                                        &camPose[3],
                                                                        &meshTrans[i][0]));
            }

            break;

          case PE_COLOR:
          case PE_DEPTH:
          case PE_DEPTH_PLANE:

            {
              ResidualImageProjection* pResidual = new ResidualImageProjection(1,
                                                                               &templateMesh.colors[i][0],
                                                                               &templateMesh.vertices[i][0],
                                                                               pCamera,
                                                                               pFrame,
                                                                               errorType);

              ceres::AutoDiffCostFunction<ResidualImageProjection, 3, 3, 3, 3>* cost_function =
                new ceres::AutoDiffCostFunction<ResidualImageProjection, 3, 3, 3, 3>( pResidual );

              dataTermResidualBlocks.push_back(
                                               problem.AddResidualBlock(
                                                                        cost_function,
                                                                        loss_function,
                                                                        &camPose[0],
                                                                        &camPose[3],
                                                                        &meshTrans[i][0]));

            }

            break;

          case PE_FEATURE:

            {

              ResidualImageProjectionDynamic* pResidual = new ResidualImageProjectionDynamic(1,
                                                                                             &templateMesh.features[i][0],
                                                                                             &templateMesh.vertices[i][0],
                                                                                             pCamera,
                                                                                             pFrame,
                                                                                             errorType);

              ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionDynamic, 5>* cost_function =
                new ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionDynamic, 5>( pResidual );

              vector<double*> parameter_blocks;
              parameter_blocks.push_back(&camPose[0]);
              parameter_blocks.push_back(&camPose[3]);
              parameter_blocks.push_back( &meshTrans[i][0] );

              // add rotation, translation and vertice parameter block
              cost_function->AddParameterBlock(3);
              cost_function->AddParameterBlock(3);
              cost_function->AddParameterBlock(3);

              cost_function->SetNumResiduals( PE_RESIDUAL_NUM_ARRAY[errorType] );

              dataTermResidualBlocks.push_back(
                                               problem.AddResidualBlock(
                                                                        cost_function,
                                                                        loss_function,
                                                                        parameter_blocks));

              break;

            }


          }


      }

  }

}

void DeformNRSFMTracker::AddCostImageProjectionPatch(ceres::Problem& problem,
                                                     ceres::LossFunction* loss_function,
                                                     dataTermErrorType errorType,
                                                     PangaeaMeshData& templateMesh,
                                                     MeshDeformation& meshTrans,
                                                     vector<bool>& visibilityMask,
                                                     MeshNeighbors& patchNeighbors,
                                                     MeshNeighbors& patchRadii,
                                                     MeshWeights& patchWeights,
                                                     CameraInfo* pCamera,
                                                     Level* pFrame)
{

  for(int i = 0; i < templateMesh.numVertices; ++i)
    {
      if(visibilityMask[i])
        {

          int numNeighbors = patchNeighbors[i].size();

          vector<double*> parameter_blocks;
          for(int j = 0; j < numNeighbors; ++j)
            parameter_blocks.push_back( &(meshTrans[ patchNeighbors[i][j] ][0]) );

          parameter_blocks.push_back( &camPose[0] );
          parameter_blocks.push_back( &camPose[3] );

          ResidualImageProjectionPatch* pResidualPatch = new ResidualImageProjectionPatch(1,
                                                                                          &templateMesh,
                                                                                          pCamera,
                                                                                          pFrame,
                                                                                          numNeighbors,
                                                                                          patchWeights[i],
                                                                                          patchRadii[i],
                                                                                          patchNeighbors[i],
                                                                                          errorType );

          ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionPatch, 5>* cost_function =
            new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionPatch, 5>( pResidualPatch );

          for(int j = 0; j < numNeighbors; ++j)
            cost_function->AddParameterBlock(3);

          cost_function->AddParameterBlock(3);
          cost_function->AddParameterBlock(3);

          cost_function->SetNumResiduals( PE_RESIDUAL_NUM_ARRAY[errorType] );

          dataTermResidualBlocks.push_back(
                                           problem.AddResidualBlock(
                                                                    cost_function,
                                                                    loss_function,
                                                                    parameter_blocks));

          // // I would like to compute residuals myself
          // (*pResidualPatch)(&(parameter_blocks[0]),  &Residuals[ per_residual_num * i ] );

          break;


        }

    }


}

void DeformNRSFMTracker::AddCostImageProjectionCoarse(ceres::Problem& problem,
                                                      ceres::LossFunction* loss_function,
                                                      dataTermErrorType errorType,
                                                      PangaeaMeshData& templateMesh,
                                                      vector<bool>& visibilityMask,
                                                      PangaeaMeshData& templateNeighborMesh,
                                                      MeshDeformation& neighborMeshTrans,
                                                      MeshDeformation& neighborMeshRot,
                                                      MeshNeighbors& neighbors,
                                                      MeshWeights& weights,
                                                      CameraInfo* pCamera,
                                                      Level* pFrame)
{
  double* pValue = NULL;

  for(int i = 0; i < templateMesh.numVertices; ++i)
    {
      if(visibilityMask[i])
        {
          vector<double*> neighborVertices;
          vector<double> neighborWeights;
          vector<double*> parameter_blocks;

          int numNeighbors = neighbors[i].size();
          for(int j = 0; j < numNeighbors; ++j )
            {
              neighborWeights.push_back( weights[i][j] );
              neighborVertices.push_back( &( templateNeighborMesh.vertices[ neighbors[i][j] ][ 0 ] ) );
              parameter_blocks.push_back( &( neighborMeshTrans[ neighbors[i][j] ][0] ) );
            }
          for(int j = 0; j < numNeighbors; ++j )
            parameter_blocks.push_back( &( neighborMeshRot[ neighbors[i][j] ][0] ) );

          parameter_blocks.push_back( &camPose[0] );
          parameter_blocks.push_back( &camPose[3] );

          getValueFromMesh(&templateMesh, errorType, i, pValue);

          ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionCoarse, 5>* cost_function =
            new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionCoarse, 5 >(
                                                                                       new ResidualImageProjectionCoarse(
                                                                                                                         1,
                                                                                                                         pValue,
                                                                                                                         &templateMesh.vertices[i][0],
                                                                                                                         pCamera,
                                                                                                                         pFrame,
                                                                                                                         numNeighbors,
                                                                                                                         neighborWeights,
                                                                                                                         neighborVertices,
                                                                                                                         errorType ) );

          for(int j = 0; j < 2*numNeighbors; ++j)
            cost_function->AddParameterBlock(3);

          cost_function->AddParameterBlock(3);
          cost_function->AddParameterBlock(3);

          cost_function->SetNumResiduals( PE_RESIDUAL_NUM_ARRAY[errorType] );

          dataTermResidualBlocks.push_back(
                                           problem.AddResidualBlock(
                                                                    cost_function,
                                                                    loss_function,
                                                                    parameter_blocks));

        }
    }

}

void DeformNRSFMTracker::AddCostImageProjectionPatchCoarse(ceres::Problem& problem,
                                                           ceres::LossFunction* loss_function,
                                                           dataTermErrorType& errorType,
                                                           PangaeaMeshData& templateMesh,
                                                           vector<bool>& visibilityMask,
                                                           MeshNeighbors& patchNeighbors,
                                                           MeshNeighbors& patchRadii,
                                                           MeshWeights& patchWeights,
                                                           PangaeaMeshData& templateNeighborMesh,
                                                           MeshDeformation& neighborMeshTrans,
                                                           MeshDeformation& neighborMeshRot,
                                                           MeshNeighbors& neighbors,
                                                           MeshWeights& weights,
                                                           CameraInfo* pCamera,
                                                           Level* pFrame)
{

  for(int i = 0; i < templateMesh.numVertices; ++i)
    {
      if(visibilityMask[i])
        {
          // path neighbors
          int numNeighbors;

          //coarse neighbors;
          int numCoarseNeighbors;
          vector<unsigned int> parameterIndices;
          vector<unsigned int> coarseNeighborIndices;
          vector<unsigned int> coarseNeighborBiases;
          vector<double> coarseNeighborWeights;

          vector<double*> parameter_blocks;
          vector<double*> parameter_blocks_rot;

          numNeighbors = patchNeighbors[i].size();

          int bias = 0;
          vector<double*>::iterator iter;
          for(int j = 0; j < numNeighbors; ++j)
            {
              int m = patchNeighbors[i][j];
              int coarseNum = neighbors[ m ].size();

              bias += coarseNum;
              coarseNeighborBiases.push_back( bias );

              for(int k = 0; k < coarseNum; ++k)
                {
                  coarseNeighborIndices.push_back( neighbors[m][k] );
                  coarseNeighborWeights.push_back( weights[m][k] );

                  // if not in parameter_blocks yet
                  double* block = &( neighborMeshTrans[ neighbors[m][k] ][0] );
                  double* block_rot = &( neighborMeshRot[ neighbors[m][k] ][0] );

                  iter = std::find(parameter_blocks.begin(), parameter_blocks.end(), block);
                  if(iter != parameter_blocks.end())
                    {
                      parameter_blocks.push_back( block );
                      parameter_blocks_rot.push_back( block_rot );
                      parameterIndices.push_back( parameter_blocks.size() - 1 );
                    }
                  else
                    parameterIndices.push_back( iter - parameter_blocks.begin() );

                }
            }

          numCoarseNeighbors = parameter_blocks.size();

          for(int k = 0; k < parameter_blocks_rot.size(); ++k)
            parameter_blocks.push_back( parameter_blocks_rot[k] );

          parameter_blocks.push_back( &camPose[0] );
          parameter_blocks.push_back( &camPose[3] );

          ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionPatchCoarse, 5>* cost_function =
            new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionPatchCoarse, 5 >(
                                                                                            new ResidualImageProjectionPatchCoarse(
                                                                                                                                   1,
                                                                                                                                   &templateMesh,
                                                                                                                                   &templateNeighborMesh,
                                                                                                                                   pCamera,
                                                                                                                                   pFrame,
                                                                                                                                   numNeighbors,
                                                                                                                                   numCoarseNeighbors,
                                                                                                                                   patchWeights[i],
                                                                                                                                   patchRadii[i],
                                                                                                                                   patchNeighbors[i],
                                                                                                                                   parameterIndices,
                                                                                                                                   coarseNeighborIndices,
                                                                                                                                   coarseNeighborBiases,
                                                                                                                                   coarseNeighborWeights,
                                                                                                                                   errorType ) );

          for(int j = 0; j < 2*numCoarseNeighbors; ++j)
            cost_function->AddParameterBlock(3);

          cost_function->AddParameterBlock(3);
          cost_function->AddParameterBlock(3);

          cost_function->SetNumResiduals( PE_RESIDUAL_NUM_ARRAY[errorType] );

          dataTermResidualBlocks.push_back(
                                           problem.AddResidualBlock(
                                                                    cost_function,
                                                                    loss_function,
                                                                    parameter_blocks));

        }

    }

}

void DeformNRSFMTracker::AddPhotometricCostNew(ceres::Problem& problem,
                                               ceres::LossFunction* loss_function,
                                               dataTermErrorType errorType)
{
  // add photometric cost
  // there are two different cases
  vector<std::pair<int,int> >& data_pairs =
    pStrategy->optimizationSettings[currLevel].dataTermPairs;

  int num_data_pairs = data_pairs.size();

  CameraInfo* pCamera;
  Level* pFrame;

  for(int k = 0; k < num_data_pairs; ++k)
    {
      std::pair<int, int>& data_pair = data_pairs[k];

      if(errorType == PE_FEATURE || errorType == PE_FEATURE_NCC){
        pCamera = &pFeaturePyramid->getCameraInfo(data_pair.first);
        pFrame = &pFeaturePyramid->getCurrFeatureLevel(data_pair.first);
      }else{
        pCamera = &pImagePyramid->getCameraInfo(data_pair.first);
        pFrame = &pImagePyramid->getImageLevel(data_pair.first);
      }

      cout << "camera width and height " << pCamera->width << " " << pCamera->height << endl;

      cout << "dataTerm pair" << endl;
      cout << data_pair.first << "->" << data_pair.second << endl;

      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[ data_pair.first ];
      MeshDeformation& meshTrans = meshTransPyramid[ data_pair.first ];

      vector<bool>& visibilityMask = visibilityMaskPyramid[ data_pair.first ];

      MeshNeighbors& patchNeighbors = meshPropagation.getPatchNeighbors( data_pair.first );
      MeshWeights& patchWeights = meshPropagation.getPatchWeights( data_pair.first );
      MeshNeighbors& patchRadii = meshPropagation.getPatchRadii( data_pair.first );

      if(data_pair.first == data_pair.second)
        {
          switch(errorType)
            {
            case PE_INTENSITY:
            case PE_COLOR:
            case PE_FEATURE:
              AddCostImageProjection(problem,
                                     loss_function,
                                     errorType,
                                     templateMesh,
                                     meshTrans,
                                     visibilityMask,
                                     pCamera,
                                     pFrame);
              break;
            case PE_NCC:
            case PE_COLOR_NCC:
            case PE_FEATURE_NCC:
              AddCostImageProjectionPatch(problem,
                                          loss_function,
                                          errorType,
                                          templateMesh,
                                          meshTrans,
                                          visibilityMask,
                                          patchNeighbors,
                                          patchRadii,
                                          patchWeights,
                                          pCamera,
                                          pFrame);
              break;
            }
        }
      else{

        PangaeaMeshData& templateNeighborMesh = templateMeshPyramid.levels[ data_pair.second ];
        MeshDeformation& neighborMeshTrans = meshTransPyramid[ data_pair.second ];
        MeshDeformation& neighborMeshRot = meshRotPyramid[ data_pair.second ];

        pair<int, int> meshPair( data_pair.first, data_pair.second );
        MeshNeighbors&  neighbors = meshPropagation.getNeighbors( meshPair );
        MeshWeights& weights = meshPropagation.getWeights( meshPair );

        switch(errorType)
          {

          case PE_INTENSITY:
          case PE_COLOR:
          case PE_FEATURE:

            AddCostImageProjectionCoarse(problem,
                                         loss_function,
                                         errorType,
                                         templateMesh,
                                         visibilityMask,
                                         templateNeighborMesh,
                                         neighborMeshTrans,
                                         neighborMeshRot,
                                         neighbors,
                                         weights,
                                         pCamera,
                                         pFrame);

            break;

          case PE_NCC:
          case PE_COLOR_NCC:
          case PE_FEATURE_NCC:

            AddCostImageProjectionPatchCoarse(problem,
                                              loss_function,
                                              errorType,
                                              templateMesh,
                                              visibilityMask,
                                              patchNeighbors,
                                              patchRadii,
                                              patchWeights,
                                              templateNeighborMesh,
                                              neighborMeshTrans,
                                              neighborMeshRot,
                                              neighbors,
                                              weights,
                                              pCamera,
                                              pFrame);
            break;

          }

      }
    }

}

void DeformNRSFMTracker::AddPhotometricCost(ceres::Problem& problem,
                                            ceres::LossFunction* loss_function,
                                            dataTermErrorType errorType)
{
  // add photometric cost
  // there are two different cases
  vector<std::pair<int,int> >& data_pairs =
    pStrategy->optimizationSettings[currLevel].dataTermPairs;

  int num_data_pairs = data_pairs.size();

  //TICK("SetupPhotometricCost" + std::to_string( currLevel ) );

  for(int k = 0; k < num_data_pairs; ++k)
    {
      std::pair<int, int>& data_pair = data_pairs[k];

      CameraInfo* pCamera = &pImagePyramid->getCameraInfo(data_pair.first);
      ImageLevel* pFrame = &pImagePyramid->getImageLevel(data_pair.first);

      cout << "camera width and height " << pCamera->width << " " << pCamera->height << endl;

      cout << "dataTerm pair" << endl;
      cout << data_pair.first << "->" << data_pair.second << endl;

      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[ data_pair.first ];
      MeshDeformation& meshTrans = meshTransPyramid[ data_pair.first ];

      vector<bool>& visibilityMask = visibilityMaskPyramid[ data_pair.first ];

      MeshNeighbors& patchNeighbors = meshPropagation.getPatchNeighbors( data_pair.first );
      MeshWeights& patchWeights = meshPropagation.getPatchWeights( data_pair.first );
      MeshNeighbors& patchRadii = meshPropagation.getPatchRadii( data_pair.first );

      //
      int per_residual_num = PE_RESIDUAL_NUM_ARRAY[ errorType ];

      vector<double> Residuals;
      Residuals.resize( per_residual_num * templateMesh.numVertices );

      if(data_pair.first == data_pair.second )
        {

            for(int i = 0; i < templateMesh.numVertices; ++i)
              {
                if(visibilityMask[i])
                  {
                    switch(errorType)
                      {
                      case PE_INTENSITY:
                        {
                          ResidualImageProjection* pResidual = new ResidualImageProjection(1,
                                                                                           &templateMesh.grays[i],
                                                                                           &templateMesh.vertices[i][0],
                                                                                           pCamera,
                                                                                           pFrame,
                                                                                           errorType);

                          ceres::AutoDiffCostFunction<ResidualImageProjection, 1, 3, 3, 3>* cost_function =
                            new ceres::AutoDiffCostFunction<ResidualImageProjection, 1, 3, 3, 3>( pResidual );

                          dataTermResidualBlocks.push_back(
                                                           problem.AddResidualBlock(
                                                                                    cost_function,
                                                                                    loss_function,
                                                                                    &camPose[0],
                                                                                    &camPose[3],
                                                                                    &meshTrans[i][0]));
                        }

                        break;

                      case PE_COLOR:
                        {
                          ResidualImageProjection* pResidual = new ResidualImageProjection(1,
                                                                                           &templateMesh.colors[i][0],
                                                                                           &templateMesh.vertices[i][0],
                                                                                           pCamera,
                                                                                           pFrame,
                                                                                           errorType);

                          ceres::AutoDiffCostFunction<ResidualImageProjection, 3, 3, 3, 3>* cost_function =
                            new ceres::AutoDiffCostFunction<ResidualImageProjection, 3, 3, 3, 3>( pResidual );

                          dataTermResidualBlocks.push_back(
                                                           problem.AddResidualBlock(
                                                                                    cost_function,
                                                                                    loss_function,
                                                                                    &camPose[0],
                                                                                    &camPose[3],
                                                                                    &meshTrans[i][0]));

                        }

                        break;

                      case PE_NCC:
                      case PE_COLOR_NCC:
                        {
                          // in patchNeighbors, point itself is counted as a neighbor of itself
                          // patch based photometric error
                          int numNeighbors = patchNeighbors[i].size();

                          vector<double*> parameter_blocks;
                          for(int j = 0; j < numNeighbors; ++j)
                            parameter_blocks.push_back( &(meshTrans[ patchNeighbors[i][j] ][0]) );

                          parameter_blocks.push_back( &camPose[0] );
                          parameter_blocks.push_back( &camPose[3] );

                          ResidualImageProjectionPatch* pResidualPatch = new ResidualImageProjectionPatch(1,
                                                                                                          &templateMesh,
                                                                                                          pCamera,
                                                                                                          pFrame,
                                                                                                          numNeighbors,
                                                                                                          patchWeights[i],
                                                                                                          patchRadii[i],
                                                                                                          patchNeighbors[i],
                                                                                                          errorType );

                          ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionPatch, 5>* cost_function =
                            new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionPatch, 5>( pResidualPatch );

                          for(int j = 0; j < numNeighbors; ++j)
                            cost_function->AddParameterBlock(3);

                          cost_function->AddParameterBlock(3);
                          cost_function->AddParameterBlock(3);

                          cost_function->SetNumResiduals( per_residual_num  );

                          dataTermResidualBlocks.push_back(
                                                           problem.AddResidualBlock(
                                                                                    cost_function,
                                                                                    loss_function,
                                                                                    parameter_blocks));

                          // I would like to compute residuals myself
                          (*pResidualPatch)(&(parameter_blocks[0]),  &Residuals[ per_residual_num * i ] );

                        }

                        break;

                      }
                  }
              }

            // output the residuals please
            double data_term_residual_sum = 0;

            for(int i = 0; i < templateMesh.numVertices; ++i)
              {
                if(visibilityMask[i])
                  {
                    data_term_residual_sum += Residuals[i] * Residuals[i];
                  }
              }

            cout << "ncc data term cost test: " << data_term_residual_sum << endl;

          }

      else
        {

          // if(errorType == PE_NCC )
          // cerr << " NCC error measure not supported in sparse deformation node case yet! "<< endl;

          // just try propagation strategy first
          PangaeaMeshData& templateNeighborMesh = templateMeshPyramid.levels[ data_pair.second ];
          MeshDeformation& neighborMeshTrans = meshTransPyramid[ data_pair.second ];
          MeshDeformation& neighborMeshRot = meshRotPyramid[ data_pair.second ];

          pair<int, int> meshPair( data_pair.first, data_pair.second );
          MeshNeighbors&  neighbors = meshPropagation.getNeighbors( meshPair );
          MeshWeights& weights = meshPropagation.getWeights( meshPair );

          for(int i = 0; i < templateMesh.numVertices; ++i)
            {
              if(visibilityMask[i])
                {
                  switch(errorType)
                    {
                    case PE_INTENSITY:
                    case PE_COLOR:
                      {
                        // collect neighbors info
                        vector<double*> neighborVertices;
                        vector<double> neighborWeights;
                        vector<double*> parameter_blocks;

                        int numNeighbors = neighbors[i].size();
                        for(int j = 0; j < numNeighbors; ++j )
                          {
                            neighborWeights.push_back( weights[i][j] );
                            neighborVertices.push_back( &( templateNeighborMesh.vertices[ neighbors[i][j] ][ 0 ] ) );
                            parameter_blocks.push_back( &( neighborMeshTrans[ neighbors[i][j] ][0] ) );
                          }
                        for(int j = 0; j < numNeighbors; ++j )
                          parameter_blocks.push_back( &( neighborMeshRot[ neighbors[i][j] ][0] ) );

                        parameter_blocks.push_back( &camPose[0] );
                        parameter_blocks.push_back( &camPose[3] );

                        ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionCoarse, 5>* cost_function =
                          new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionCoarse, 5 >(
                                                                                                     new ResidualImageProjectionCoarse(
                                                                                                                                       1,
                                                                                                                                       errorType == PE_INTENSITY ? &templateMesh.grays[i] : &templateMesh.colors[i][0],
                                                                                                                                       &templateMesh.vertices[i][0],
                                                                                                                                       pCamera,
                                                                                                                                       pFrame,
                                                                                                                                       numNeighbors,
                                                                                                                                       neighborWeights,
                                                                                                                                       neighborVertices,
                                                                                                                                       errorType ) );

                        for(int j = 0; j < 2*numNeighbors; ++j)
                          cost_function->AddParameterBlock(3);

                        cost_function->AddParameterBlock(3);
                        cost_function->AddParameterBlock(3);

                        cost_function->SetNumResiduals( per_residual_num );

                        dataTermResidualBlocks.push_back(
                                                         problem.AddResidualBlock(
                                                                                  cost_function,
                                                                                  loss_function,
                                                                                  parameter_blocks));
                      }

                      break;

                    case PE_NCC:
                    case PE_COLOR_NCC:
                      {
                        // patch neighbors
                        int numNeighbors;

                        // coarse neighbors
                        int numCoarseNeighbors;
                        vector<unsigned int> parameterIndices;
                        vector<unsigned int> coarseNeighborIndices;
                        vector<unsigned int> coarseNeighborBiases;
                        vector<double> coarseNeighborWeights;

                        vector<double*> parameter_blocks;
                        vector<double*> parameter_blocks_rot;

                        numNeighbors = patchNeighbors[i].size();

                        int bias = 0;
                        vector<double*>::iterator iter;
                        for(int j = 0; j < numNeighbors; ++j)
                          {
                            int m = patchNeighbors[i][j];
                            int coarseNum = neighbors[ m ].size();

                            bias += coarseNum;
                            coarseNeighborBiases.push_back( bias );

                            for(int k = 0; k < coarseNum; ++k)
                              {
                                coarseNeighborIndices.push_back( neighbors[m][k] );
                                coarseNeighborWeights.push_back( weights[m][k] );

                                // if not in parameter_blocks yet
                                double* block = &( neighborMeshTrans[ neighbors[m][k] ][0] );
                                double* block_rot = &( neighborMeshRot[ neighbors[m][k] ][0] );

                                iter = std::find(parameter_blocks.begin(), parameter_blocks.end(), block);
                                if(iter != parameter_blocks.end())
                                  {
                                    parameter_blocks.push_back( block );
                                    parameter_blocks_rot.push_back( block_rot );
                                    parameterIndices.push_back( parameter_blocks.size() - 1 );
                                  }
                                else
                                  parameterIndices.push_back( iter - parameter_blocks.begin() );

                              }
                          }

                        numCoarseNeighbors = parameter_blocks.size();

                        for(int k = 0; k < parameter_blocks_rot.size(); ++k)
                          parameter_blocks.push_back( parameter_blocks_rot[k] );

                        parameter_blocks.push_back( &camPose[0] );
                        parameter_blocks.push_back( &camPose[3] );

                        ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionPatchCoarse, 5>* cost_function =
                          new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionPatchCoarse, 5 >(
                                                                                                          new ResidualImageProjectionPatchCoarse(
                                                                                                                                                 1,
                                                                                                                                                 &templateMesh,
                                                                                                                                                 &templateNeighborMesh,
                                                                                                                                                 pCamera,
                                                                                                                                                 pFrame,
                                                                                                                                                 numNeighbors,
                                                                                                                                                 numCoarseNeighbors,
                                                                                                                                                 patchWeights[i],
                                                                                                                                                 patchRadii[i],
                                                                                                                                                 patchNeighbors[i],
                                                                                                                                                 parameterIndices,
                                                                                                                                                 coarseNeighborIndices,
                                                                                                                                                 coarseNeighborBiases,
                                                                                                                                                 coarseNeighborWeights,
                                                                                                                                                 errorType ) );

                        for(int j = 0; j < 2*numCoarseNeighbors; ++j)
                          cost_function->AddParameterBlock(3);

                        cost_function->AddParameterBlock(3);
                        cost_function->AddParameterBlock(3);

                        cost_function->SetNumResiduals( per_residual_num );

                        dataTermResidualBlocks.push_back(
                                                         problem.AddResidualBlock(
                                                                                  cost_function,
                                                                                  loss_function,
                                                                                  parameter_blocks));

                      }

                      break;

                    }
                }
            }

        }
    }

  //TOCK("SetupPhotometricCost" + std::to_string( currLevel ) );
}

void DeformNRSFMTracker::AddTotalVariationCost(ceres::Problem& problem,
                                               ceres::LossFunction* loss_function)
{

  vector<std::pair<int,int> >& tv_pairs =
    pStrategy->optimizationSettings[currLevel].regTermPairs;

  int num_tv_pairs = tv_pairs.size();

  for(int k = 0; k < num_tv_pairs; ++k)
    {
      std::pair<int, int>& tv_pair = tv_pairs[k];

      bool same_level = tv_pair.first == tv_pair.second;

      cout << "tv pair" << endl;
      cout << tv_pair.first << "->" << tv_pair.second << endl;

      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[tv_pair.first];

      MeshDeformation& meshTrans = trackerSettings.usePrevForTemplateInTV ?
        prevMeshTransPyramid[tv_pair.first] : meshTransPyramid[tv_pair.first];
      MeshDeformation& neighborMeshTrans = trackerSettings.usePrevForTemplateInTV ?
        prevMeshTransPyramid[tv_pair.second] : meshTransPyramid[tv_pair.second];

      vector<vector<unsigned int> >& meshNeighbors = same_level ?
        templateMesh.adjVerticesInd : meshPropagation.getNeighbors( tv_pair );
      vector<vector<double> >& meshWeights = meshPropagation.getWeights( tv_pair );

      for(int vertex = 0; vertex < templateMesh.numVertices; ++vertex)
        {
          for(int neighbor = 0; neighbor < meshNeighbors[vertex].size(); ++neighbor)
            {
              double weight = same_level ? 1 : meshWeights[vertex][neighbor];
              if(!same_level || vertex < meshNeighbors[vertex][neighbor])
                problem.AddResidualBlock(
                                         new ceres::AutoDiffCostFunction<ResidualTV, 3, 3, 3>(
                                                                                              new ResidualTV( weight ) ),
                                         loss_function,
                                         &meshTrans[ vertex  ][0],
                                         &neighborMeshTrans[ meshNeighbors[vertex][neighbor] ][0]);

            }
        }
    }

}

void DeformNRSFMTracker::AddRotTotalVariationCost(ceres::Problem& problem,
                                                  ceres::LossFunction* loss_function)
{
  vector<std::pair<int,int> >& tv_pairs =
    pStrategy->optimizationSettings[currLevel].regTermPairs;
  int num_tv_pairs = tv_pairs.size();

  for(int k = 0; k < num_tv_pairs; ++k)
    {
      std::pair<int, int>& tv_pair = tv_pairs[k];

      bool same_level = tv_pair.first == tv_pair.second;

      cout << "rot_tv pair" << endl;
      cout << tv_pair.first << "->" << tv_pair.second << endl;

      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[tv_pair.first];

      vector<vector<double> >& meshRot = trackerSettings.usePrevForTemplateInTV ?
        prevMeshRotPyramid[tv_pair.first] : meshRotPyramid[tv_pair.first];
      vector<vector<double> >& neighborMeshRot = trackerSettings.usePrevForTemplateInTV ?
        prevMeshRotPyramid[tv_pair.second] : meshRotPyramid[tv_pair.second];

      vector<vector<unsigned int> >& meshNeighbors = same_level ?
        templateMesh.adjVerticesInd : meshPropagation.getNeighbors( tv_pair );
      vector<vector<double> >& meshWeights = meshPropagation.getWeights( tv_pair );

      for(int vertex = 0; vertex < templateMesh.numVertices; ++vertex)
        {
          for(int neighbor = 0; neighbor < meshNeighbors[vertex].size(); ++neighbor)
            {
              double weight = same_level ? 1 : meshWeights[vertex][neighbor];
              if(!same_level || vertex < meshNeighbors[vertex][neighbor])
                problem.AddResidualBlock(
                                         new ceres::AutoDiffCostFunction<ResidualRotTV, 3, 3, 3>(
                                                                                                 new ResidualRotTV( weight )),
                                         loss_function,
                                         &meshRot[ vertex  ][0],
                                         &neighborMeshRot[ meshNeighbors[vertex][neighbor] ][0]);
            }
        }
    }

}

void DeformNRSFMTracker::AddARAPCost(ceres::Problem& problem,
                                     ceres::LossFunction* loss_function)
{
  vector<std::pair<int,int> >& arap_pairs =
    pStrategy->optimizationSettings[currLevel].regTermPairs;
  int num_arap_pairs = arap_pairs.size();

  for(int k = 0; k < num_arap_pairs; ++k)
    {
      std::pair<int, int>& arap_pair = arap_pairs[k];

      bool same_level = arap_pair.first == arap_pair.second;

      cout << "arap pair" << endl;
      cout << arap_pair.first << "->" << arap_pair.second << endl;

      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[arap_pair.first];
      PangaeaMeshData& templateNeighborMesh = templateMeshPyramid.levels[arap_pair.second];

      MeshDeformation& meshTrans = meshTransPyramid[arap_pair.first];
      MeshDeformation& neighborMeshTrans = meshTransPyramid[arap_pair.second];

      vector<vector<double> >& meshRot = meshRotPyramid[arap_pair.first];

      vector<vector<unsigned int> >& meshNeighbors = same_level ?
        templateMesh.adjVerticesInd : meshPropagation.getNeighbors( arap_pair );
      vector<vector<double> >& meshWeights = meshPropagation.getWeights( arap_pair );


      for(int vertex = 0; vertex < templateMesh.numVertices; ++vertex)
        {
          //#pragma omp parallel for ordered schedule(dynamic)
          //#pragma omp parallel for
          for(int neighbor = 0; neighbor < meshNeighbors[vertex].size(); ++neighbor)
            {
              double weight = same_level ? 1 : meshWeights[vertex][neighbor];

              // ResidualARAP* pResidualARAP = new ResidualARAP( weight, &templateMesh.vertices[vertex][0],
              //     &templateNeighborMesh.vertices[ meshNeighbors[vertex][neighbor] ][0], true);
              // ceres::AutoDiffCostFunction<ResidualARAP, 3, 3, 3, 3>* pAutoDiffCostFunction =
              //     new ceres::AutoDiffCostFunction<ResidualARAP, 3, 3, 3, 3>(pResidualARAP);

              // //#pragma omp ordered
              // problem.AddResidualBlock(pAutoDiffCostFunction,
              // loss_function,
              // &meshTrans[ vertex  ][0],
              // &neighborMeshTrans[ meshNeighbors[vertex][neighbor] ][0],
              // &meshRot[ vertex ][0]);
              problem.AddResidualBlock(
                                       new ceres::AutoDiffCostFunction<ResidualARAP, 3, 3, 3, 3>(
                                                                                                 new ResidualARAP( weight, &templateMesh.vertices[vertex][0],
                                                                                                                   &templateNeighborMesh.vertices[ meshNeighbors[vertex][neighbor] ][0], true) ),
                                       loss_function,
                                       &meshTrans[ vertex  ][0],
                                       &neighborMeshTrans[ meshNeighbors[vertex][neighbor] ][0],
                                       &meshRot[ vertex ][0]);
            }
        }

    }

}

void DeformNRSFMTracker::AddInextentCost(ceres::Problem& problem,
                                         ceres::LossFunction* loss_function)
{
  vector<std::pair<int,int> >& inextent_pairs =
    pStrategy->optimizationSettings[currLevel].regTermPairs;

  int num_inextent_pairs = inextent_pairs.size();

  for(int k = 0; k < num_inextent_pairs; ++k)
    {
      std::pair<int, int>& inextent_pair = inextent_pairs[k];

      bool same_level = inextent_pair.first == inextent_pair.second;

      cout << "inextent pair" << endl;
      cout << inextent_pair.first << "->" << inextent_pair.second << endl;

      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[inextent_pair.first];
      PangaeaMeshData& templateNeighborMesh = templateMeshPyramid.levels[inextent_pair.second];

      MeshDeformation& meshTrans = meshTransPyramid[inextent_pair.first];
      MeshDeformation& neighborMeshTrans = meshTransPyramid[inextent_pair.second];

      vector<vector<unsigned int> >& meshNeighbors = same_level ?
        templateMesh.adjVerticesInd : meshPropagation.getNeighbors( inextent_pair );
      vector<vector<double> >& meshWeights = meshPropagation.getWeights( inextent_pair );

      for(int vertex = 0; vertex < templateMesh.numVertices; ++vertex)
        {
          for(int neighbor = 0; neighbor < meshNeighbors[vertex].size(); ++neighbor)
            {
              double weight = same_level ? 1 : meshWeights[vertex][neighbor];
              //if(!same_level || vertex < meshNeighbors[vertex][neighbor])
              problem.AddResidualBlock(
                                       new ceres::AutoDiffCostFunction<ResidualINEXTENT, 1, 3, 3>(
                                                                                                  new ResidualINEXTENT( weight )),
                                       loss_function,
                                       &meshTrans[ vertex  ][0],
                                       &neighborMeshTrans[ meshNeighbors[vertex][neighbor] ][0]);
            }
        }
    }
}

void DeformNRSFMTracker::AddDeformationCost(ceres::Problem& problem,
                                            ceres::LossFunction* loss_function)
{
  vector<int>& deform_level_vec =
    pStrategy->optimizationSettings[currLevel].deformTermLevelIDVec;

  int num_deform_levels = deform_level_vec.size();

  for(int k = 0; k < num_deform_levels; ++k)
    {
      int deform_level = deform_level_vec[k];

      cout << "deformation level " << deform_level << endl;

      MeshDeformation& meshTrans = meshTransPyramid[deform_level];
      MeshDeformation& prevMeshTrans = prevMeshTransPyramid[deform_level];

      for(int vertex = 0; vertex < meshTrans.size(); ++vertex)
        {
          problem.AddResidualBlock(
                                   new ceres::AutoDiffCostFunction<ResidualDeform, 3, 3>(
                                                                                         new ResidualDeform(1,  &prevMeshTrans[ vertex ][ 0 ])),
                                   loss_function,
                                   &meshTrans[ vertex ][0]);
        }
    }
}

void DeformNRSFMTracker::AddTemporalMotionCost(ceres::Problem& problem,
                                               double rotWeight, double transWeight)
{
  // cout << "prev motion started:" << endl;

  // for(int i = 0; i < 6; ++i)
  // {
  //     prevCamPose[i] = camPose[i];
  //     cout << camPose[i] << endl;
  // }

  problem.AddResidualBlock(
                           new ceres::AutoDiffCostFunction<ResidualTemporalMotion, 6, 3, 3>(
                                                                                            new ResidualTemporalMotion(prevCamPose, prevCamPose+3,
                                                                                                                       rotWeight, transWeight)),
                           NULL,
                           &camPose[0],
                           &camPose[3]);
}

void DeformNRSFMTracker::EnergySetup(ceres::Problem& problem)
{
  // now we are already to construct the energy
  // photometric term
  WeightPara& weightParaLevel = pStrategy->weightParaVec[currLevel];

  // get parameter weightings
  // cout << "data term weight" << " : " << weightParaLevel.dataTermWeight << endl;
  // cout << "data huber width" << " : " << weightParaLevel.dataHuberWidth << endl;

  long long int ii = currLevel;

  if(trackerSettings.useRGBImages && trackerSettings.weightPhotometric > 0)
    {
      TICK( "SetupDataTermCost" + std::to_string(ii) );

      ceres::LossFunction* pPhotometricLossFunction = NULL;
      if(weightParaLevel.dataHuberWidth)
        {
          pPhotometricLossFunction = new ceres::HuberLoss(
                                                          weightParaLevel.dataHuberWidth);
        }
      ceres::ScaledLoss* photometricScaledLoss = new ceres::ScaledLoss(
                                                                       pPhotometricLossFunction,
                                                                       weightParaLevel.dataTermWeight,
                                                                       ceres::TAKE_OWNERSHIP);

      AddPhotometricCostNew(problem, photometricScaledLoss, PEType);

      TOCK( "SetupDataTermCost" + std::to_string(ii) );
    }

  if(trackerSettings.useFeatureImages && featureSettings.featureTermWeight > 0)
    {
      TICK("SetupFeatureTermCost" + std::to_string(ii));

      ceres::LossFunction* pFeatureLossFunction = NULL;
      if(weightParaLevel.featureHuberWidth)
        {
          pFeatureLossFunction = new ceres::HuberLoss(
                                                      weightParaLevel.featureHuberWidth);

        }
      ceres::ScaledLoss* featureScaledLoss = new ceres::ScaledLoss(
                                                                   pFeatureLossFunction,
                                                                   weightParaLevel.featureTermWeight,
                                                                   ceres::TAKE_OWNERSHIP);
      AddPhotometricCostNew(problem, featureScaledLoss, PEType);

      TOCK("SetupFeatureTermCost" + std::to_string(ii));

    }



  if(!useProblemWrapper || !problemWrapper.getLevelFlag( currLevel ) )
    {
      TICK( "SetupRegTermCost" + std::to_string(ii) );

      RegTermsSetup( problem, weightParaLevel );
      problemWrapper.setLevelFlag( currLevel );

      TOCK( "SetupRegTermCost" + std::to_string(ii) );
    }


}


void DeformNRSFMTracker::RegTermsSetup(ceres::Problem& problem, WeightPara& weightParaLevel)
{
  // totatl variation term
  if(weightParaLevel.tvTermWeight)
    {
      //TICK("SetupTVCost"  + std::to_string( currLevel ) );

      ceres::LossFunction* pTVLossFunction = NULL;

      if(trackerSettings.tvTukeyWidth)
        {
          pTVLossFunction = new ceres::TukeyLoss(trackerSettings.tvTukeyWidth);
        }else if(trackerSettings.tvHuberWidth)
        {
          pTVLossFunction = new ceres::HuberLoss(trackerSettings.tvHuberWidth);
        }
      ceres::ScaledLoss* tvScaledLoss = new ceres::ScaledLoss(
                                                              pTVLossFunction, weightParaLevel.tvTermWeight, ceres::TAKE_OWNERSHIP);

      AddTotalVariationCost(problem, tvScaledLoss);
      //AddTotalVariationCost(problem, NULL);

      //TOCK("SetupTVCost"  + std::to_string( currLevel ) );
    }

  // rotation total variation term
  // arap has to be turned on, otherwise there is no rotation variable
  if(weightParaLevel.arapTermWeight &&  weightParaLevel.tvRotTermWeight)
    {
      //TICK("SetupRotTVCost"  + std::to_string( currLevel ) );

      ceres::LossFunction* pRotTVLossFunction = NULL;

      if(trackerSettings.tvRotHuberWidth)
        {
          pRotTVLossFunction = new ceres::HuberLoss(trackerSettings.tvRotHuberWidth);
        }
      ceres::ScaledLoss* tvRotScaledLoss = new ceres::ScaledLoss(
                                                                 pRotTVLossFunction , weightParaLevel.tvRotTermWeight, ceres::TAKE_OWNERSHIP);

      AddRotTotalVariationCost(problem, tvRotScaledLoss);

      //TOCK("SetupRotTVCost"  + std::to_string( currLevel ) );

    }

  // arap term
  if(weightParaLevel.arapTermWeight)
    {
      //TICK("SetupARAPCost"  + std::to_string( currLevel ) );

      ceres::ScaledLoss* arapScaledLoss = new ceres::ScaledLoss(NULL,
                                                                weightParaLevel.arapTermWeight, ceres::TAKE_OWNERSHIP);
      AddARAPCost(problem, arapScaledLoss);

      //TOCK("SetupARAPCost"  + std::to_string( currLevel ) );
    }

  // inextensibility term
  //    cout << "inextent weight: " << weightParaLevel.inextentTermWeight << endl;
  if(weightParaLevel.inextentTermWeight)
    {
      //TICK("SetupInextentCost"  + std::to_string( currLevel ) );

      ceres::ScaledLoss* inextentScaledLoss = new ceres::ScaledLoss(NULL,
                                                                    weightParaLevel.inextentTermWeight, ceres::TAKE_OWNERSHIP);
      AddInextentCost(problem, inextentScaledLoss);

      //TOCK("SetupInextentCost"  + std::to_string( currLevel ) );
    }

  // deformation term
  //cout << "deform weight: " << weightParaLevel.deformWeight << endl;
  if(weightParaLevel.deformWeight)
    {
      //TICK("SetupDeformationCost"  + std::to_string( currLevel ) );

      ceres::ScaledLoss* deformScaledLoss = new ceres::ScaledLoss(NULL,
                                                                  weightParaLevel.deformWeight, ceres::TAKE_OWNERSHIP);
      AddDeformationCost(problem, deformScaledLoss);

      //TOCK("SetupDeformationCost" + std::to_string( currLevel ) );
    }

  // temporal term
  // cout << "translation and rotation parameter: " << weightParaLevel.transWeight
  //      << " " << weightParaLevel.rotWeight << endl;
  if(weightParaLevel.transWeight || weightParaLevel.rotWeight)
    AddTemporalMotionCost(problem, sqrt(weightParaLevel.rotWeight),
                          sqrt(weightParaLevel.transWeight));
}

void DeformNRSFMTracker::EnergyMinimization(ceres::Problem& problem)
{
  ceres::Solver::Options options;

  // solve the term and get solution
  options.max_num_iterations = trackerSettings.maxNumIterations[currLevel];
  options.linear_solver_type = mapLinearSolver(trackerSettings.linearSolver);
  options.minimizer_progress_to_stdout = trackerSettings.isMinimizerProgressToStdout;

  options.function_tolerance = trackerSettings.functionTolerances[currLevel];
  options.gradient_tolerance = trackerSettings.gradientTolerances[currLevel];
  options.parameter_tolerance = trackerSettings.parameterTolerances[currLevel];
  options.min_relative_decrease = trackerSettings.minRelativeDecreases[currLevel];

  options.initial_trust_region_radius = trackerSettings.initialTrustRegionRadiuses[0];
  options.max_trust_region_radius = trackerSettings.maxTrustRegionRadiuses[currLevel];
  options.min_trust_region_radius = trackerSettings.minTrustRegionRadiuses[currLevel];

  options.num_linear_solver_threads = trackerSettings.numLinearSolverThreads;
  options.num_threads = trackerSettings.numThreads;
  options.max_num_consecutive_invalid_steps = 0;

  options.minimizer_type = mapMinimizerType(trackerSettings.minimizerType);
  options.line_search_direction_type = mapLineSearchDirectionType(trackerSettings.lineSearchDirectionType);
  options.line_search_type = mapLineSearchType(trackerSettings.lineSearchType);
  options.nonlinear_conjugate_gradient_type = mapNonLinearCGType(trackerSettings.nonlinearConjugateGradientType);
  options.line_search_interpolation_type = mapLineSearchInterpType(trackerSettings.lineSearchInterpolationType);

  EnergyCallback energy_callback = EnergyCallback();

  // set this to true if you want to have access to updated parameters inside callback
  options.update_state_every_iteration = true;
  options.callbacks.push_back(&energy_callback);

  ceres::Solver::Summary summary;

  if(BAType == BA_MOTSTR && trackerSettings.doAlternation)
    {
      // fix the structure and optimize the motion
      AddConstantMask(problem, BA_STR); // make structure parameters constant

      options.minimizer_type = ceres::TRUST_REGION;
      ceres::Solve(options, &problem, &summary);

      if(trackerSettings.saveResults)
        {
          ceresOutput << summary.FullReport() << std::endl;
          ceresOutput << "Optimize Motion" << std::endl;
          energy_callback.PrintEnergy(ceresOutput);
        }
      else if(trackerSettings.isMinimizerProgressToStdout)
        {
          std::cout << summary.FullReport() << std::endl;
          std::cout << "Optimize Motion" << std::endl;
        }

      energy_callback.Reset();

      // make the structure variable again after optimization
      AddVariableMask(problem, BA_STR);

      // cout << "printing after motion optimization started" << endl;
      // cout << camPose[0] << " " << camPose[1] << " " << camPose[2] << endl;
      // cout << camPose[3] << " " << camPose[4] << " " << camPose[5] << endl;
      // cout << "printing after motion optimization finished" << endl;

      // optimize the shape
      // fix the motion
      AddConstantMask(problem, BA_MOT);

      options.minimizer_type = mapMinimizerType(trackerSettings.minimizerType);
      ceres::Solve(options, &problem, &summary);

      if(trackerSettings.saveResults)
        {
          ceresOutput << summary.FullReport() << std::endl;
          ceresOutput << "Optimize Shape" << std::endl;
          energy_callback.PrintEnergy(ceresOutput);
        }
      else if(trackerSettings.isMinimizerProgressToStdout)
        {
          std::cout << summary.FullReport() << std::endl;
          std::cout << "Optimize Shape" << std::endl;
        }

      // make the motion parameters variable again after optimization
      AddVariableMask(problem, BA_MOT);
    }
  else
    {
      // add const mask if necessary,
      // if we want to optimie motion or structure only
      AddConstantMask(problem, BAType == BA_MOT ? BA_STR : BA_MOTSTR);
      AddConstantMask(problem, BAType == BA_STR ? BA_MOT : BA_MOTSTR);

      ceres::Solve(options, &problem, &summary);
      if(trackerSettings.saveResults)
        {
          ceresOutput << summary.FullReport() << std::endl;
          ceresOutput << "Optimize Motion and Shape" << std::endl;
          energy_callback.PrintEnergy(ceresOutput);
        }
      else if(trackerSettings.isMinimizerProgressToStdout)
        {
          std::cout << summary.FullReport() << std::endl;
          std::cout << "Optimize Motion and Shape" << std::endl;
        }

      AddVariableMask(problem, BAType == BA_MOT ? BA_STR : BA_MOTSTR);
      AddVariableMask(problem, BAType == BA_STR ? BA_MOT : BA_MOTSTR);

    }

}

bool DeformNRSFMTracker::SaveData()
{
  //create result folder
  bfs::path result_folder(trackerSettings.savePath.c_str());
  cout << trackerSettings.savePath << endl;
  if(!bfs::exists(result_folder))
    {
      if(!bfs::create_directories(result_folder) && !bfs::exists(result_folder))
        {
          std::cerr << "Cannot create result directory: " << result_folder << std::endl;
          return EXIT_FAILURE;
        }
    }

  // save shape
  char buffer[BUFFER_SIZE];
  std::ofstream shapeFile;
  std::stringstream shapeFilePath;
  sprintf(buffer,"shape_%04d.txt",currentFrameNo);
  shapeFilePath << trackerSettings.savePath << buffer;

  shapeFile.open(shapeFilePath.str().c_str(),std::ofstream::trunc);

  PangaeaMeshData& currentMesh = outputInfoPyramid[0].meshData;
  for(int i =0; i < currentMesh.numVertices; ++i)
    {
      shapeFile << currentMesh.vertices[i][0] << " "
                << currentMesh.vertices[i][1] << " "
                << currentMesh.vertices[i][2] << " "
                << std::endl;
    }
  shapeFile.close();

  return true;
}

bool DeformNRSFMTracker::SaveMeshToFile(TrackerOutputInfo& outputInfo)
{
  // create the directory if there isn't one
  if(!bfs::exists(trackerSettings.savePath.c_str()))
    {
      bfs::create_directory(trackerSettings.savePath.c_str());
    }

  // save current mesh to file for results checking afterwards
  char buffer[BUFFER_SIZE];
  std::stringstream meshFile;
  sprintf(buffer, trackerSettings.meshFormat.c_str(), currentFrameNo);
  meshFile << trackerSettings.savePath << buffer;

  PangaeaMeshIO::writeToFile(meshFile.str(), outputInfo.meshData);

  return true;
}

bool DeformNRSFMTracker::SaveMeshPyramid()
{
  // meshPyramid directory
  char buffer[BUFFER_SIZE];
  std::stringstream mesh_pyramid_path;
  std::stringstream mesh_file;

  if(!bfs::exists(trackerSettings.savePath.c_str()))
    {
      bfs::create_directory(trackerSettings.savePath.c_str());
      cout << "creating dir " << trackerSettings.savePath << endl;
    }

  mesh_pyramid_path << trackerSettings.savePath << "/mesh_pyramid/";

  cout << mesh_pyramid_path.str() << endl;

  if(!bfs::exists(mesh_pyramid_path.str().c_str()))
    {
      bfs::create_directory(mesh_pyramid_path.str().c_str());
      cout << "creating dir " << mesh_pyramid_path.str() << endl;
    }

  // save mesh pyramid
  for(int i = 0; i < m_nMeshLevels; ++i)
    {
      sprintf(buffer, trackerSettings.meshPyramidFormat.c_str(), currentFrameNo, i);
      mesh_file << mesh_pyramid_path.str() << buffer;

      PangaeaMeshIO::writeToFile(mesh_file.str(), outputInfoPyramid[i].meshData);

      mesh_file.str("");
      //memset(&buffer[0], 0, sizeof(buffer));
    }

  if(trackerSettings.savePropPyramid)
    {
      // save mesh pyramid
      for(int i = 0; i < m_nMeshLevels; ++i)
        {
          sprintf(buffer, trackerSettings.propPyramidFormat.c_str(), currentFrameNo, i);
          mesh_file << mesh_pyramid_path.str() << buffer;

          PangaeaMeshIO::writeToFile(mesh_file.str(), outputPropPyramid[i].meshData);

          mesh_file.str("");
          //memset(&buffer[0], 0, sizeof(buffer));
        }
    }

  return true;
}

void DeformNRSFMTracker::updateRenderingLevel(TrackerOutputInfo** pOutputInfoRendering,
                                              int nRenderLevel, bool renderType)
{
  if(renderType)
    *pOutputInfoRendering = &(outputPropPyramid[nRenderLevel]);
  else
    *pOutputInfoRendering = &(outputInfoPyramid[nRenderLevel]);
  // // show camera center
  // cout << "camera center of level " << nRenderLevel << " is " << endl;
  // cout << outputInfoPyramid[nRenderLevel].meshData.center[0] << " "
  //      << outputInfoPyramid[nRenderLevel].meshData.center[1] << " "
  //      << outputInfoPyramid[nRenderLevel].meshData.center[2] << " "
  //      << endl;
}

void DeformNRSFMTracker::SaveThread(TrackerOutputInfo** pOutputInfoRendering)
{
  if(trackerSettings.saveResults)
    {
      if(trackerSettings.saveMesh)
        {
          if(trackerSettings.saveMeshPyramid)
            {
              // the current mesh pyramid is outputInfoPyramid
              cout << "save mesh pyramid started " << endl;
              SaveMeshPyramid();
              cout << "save mesh pyramid finished " << endl;
            }
          else
            {
              cout << "save mesh started " << endl;
              // the output mesh has been rotated and translated
              // to align with images
              SaveMeshToFile(**pOutputInfoRendering);
              cout << "save mesh finished " << endl;
            }
        }
      else
        SaveData();
    }
}

void DeformNRSFMTracker::AttachFeaturesToMeshPyramid()
{
  // attach features based on previous frame to the mesh
  // project those visible points based on visibilityMask
  // to the feature image

  for(int i = 0; i < m_nMeshLevels; ++i)
    {

      // mesh from previous frame
      // be careful when trying to use multi-threading, as outputInfoPyramid during
      // the process of tracking, or copy over the previous rotation and translation
      // and use the templateMesh
      PangaeaMeshData& prevMesh = outputInfoPyramid[i].meshData;
      FeatureLevel& featureLevel = pFeaturePyramid->getPrevFeatureLevel(i);
      CameraInfo& camInfo = pFeaturePyramid->getCameraInfo(i);
      vector<bool>& visibilityMask = visibilityMaskPyramid[i];

      // update the features in template mesh
      PangaeaMeshData& templateMesh = templateMeshPyramid.levels[i];
      AttachFeatureToMesh(&prevMesh, &featureLevel, &camInfo, visibilityMask, &templateMesh);

    }
}

void DeformNRSFMTracker::AttachFeatureToMesh(PangaeaMeshData* pMesh,
                                             FeatureLevel* pFeatureLevel,
                                             CameraInfo* pCamera,
                                             vector<bool>& visibilityMask,
                                             PangaeaMeshData* pOutputMesh)
{


  int numVertices = visibilityMask.size();
  pOutputMesh->featuresBuffer.resize(numVertices);

  int numChannels = pFeatureLevel->featureImageVec.size();

  for(int i = 0; i < numVertices; ++i)
    {
      if(visibilityMask[i])
        {
          pOutputMesh->featuresBuffer[i].resize( numChannels );

          getValue(pCamera,
                   pFeatureLevel,
                   &(pMesh->vertices[i][0]),
                   &(pOutputMesh->featuresBuffer[i][0]),
                   PE_FEATURE);

        }

    }

}
