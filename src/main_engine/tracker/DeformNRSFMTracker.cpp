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

DeformNRSFMTracker::DeformNRSFMTracker(TrackerSettings& settings, int width, int height, double K[3][3],
    int startFrame, int numTrackingFrames):
    trackerInitialized(false)
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

}

DeformNRSFMTracker::~DeformNRSFMTracker()
{
    delete pStrategy;
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

    // setting weights of the 0th level of the pyramid
    pStrategy->setWeightParameters(weightPara);

    // setting weighting scales over different levels of the pyramid
    //pStrategy->setWeightScale(trackerSettings.meshVertexNum);
    pStrategy->setWeightScale(templateMeshPyramid.meshPyramidVertexNum);

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
    for(std::map< pair<int, int>, int>::iterator it = meshPropagation.neighborMap.begin(); it != meshPropagation.neighborMap.end(); ++it)
    {
        std::cout << "(" << it->first.first << "," << it->first.second << ")"
                  << " => " << it->second << '\n';
    }

    // imagePyramid will be created during the processing of the first image
    imagePyramid.create(m_nWidth, m_nHeight);
    imagePyramid.setupCameraPyramid(m_nMeshLevels, camInfo);

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

            TICK( "visibilityMask" + std::to_string(i) );
            
            if(trackerSettings.useOpenGLMask)
            {
                double tempCamPose[6] = {0,0,0,0,0,0};
                UpdateVisibilityMaskGL(outputInfoPyramid[i], visibilityMaskPyramid[i], KK, tempCamPose, m_nWidth, m_nHeight);
            }
            else
            {
                UpdateVisibilityMask(outputInfoPyramid[i], visibilityMaskPyramid[i], m_nWidth, m_nHeight);
            }

            TOCK( "visibilityMask" + std::to_string(i) );
            
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

    // update the image pyramid, including images and gradients
    // also depth and normals if there is anything related
    imagePyramid.setupPyramid(pColorImageRGB, m_nMeshLevels);

    int numOptimizationLevels = pStrategy->numOptimizationLevels;

    // how many levels to do optimization on ?
    for(int i = numOptimizationLevels - 1; i >= 0; --i)
    {
        TICK( "trackingTimeLevel" + std::to_string(i) );
        
        currLevel = i;
        // start tracking
        // create the optimization problem we are trying to solve
        // should make problem a member of DeformNRSFMTracker to
        // avoid the same memory allocation every frame, could take
        // seconds to allocate memory for each frame
        ceres::Problem problem;

        TICK( "trackingTimeLevel" + std::to_string(i)  + "::ProblemSetup");
        EnergySetup(problem);
        TOCK( "trackingTimeLevel" + std::to_string(i)  + "::ProblemSetup");

        TICK( "trackingTimeLevel" + std::to_string(i)  + "::ProblemMinimization");
        EnergyMinimization(problem);
        TOCK( "trackingTimeLevel" + std::to_string(i)  + "::ProblemMinimization");

        // at this point we've finished the optimization on level i
        // now we need to update all the results and propagate the optimization
        // results to next level if necessary
        // the first step is update the current results

        TICK( "trackingTimeLevel" + std::to_string(i)  + "::UpdateResults");
        UpdateResults();
        TOCK( "trackingTimeLevel" + std::to_string(i)  + "::UpdateResults");

        //*pOutputInfoRendering = &outputInfoPyramid[i];
        //updateRenderingLevel(pOutputInfoRendering, i);
        TICK( "trackingTimeLevel" + std::to_string(i)  + "::PropagateMesh");
        PropagateMesh();
        TOCK( "trackingTimeLevel" + std::to_string(i)  + "::PropagateMesh");

        TOCK( "trackingTimeLevel" + std::to_string(i) );
        
    }   
    
    // update the results
    updateRenderingLevel(pOutputInfoRendering, 0);
    //*pOutputInfoRendering = &outputInfoPyramid[0];

    // update the top level of propagation result
    outputPropPyramid[m_nMeshLevels-1] = outputInfoPyramid[m_nMeshLevels-1];

    //save data
    TICK("SavingTime");
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
    TOCK("SavingTime");
    // simply return true;
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

    // output result for rendering
    TICK( "updateRenderingLevel" + std::to_string( level ) );
    
    UpdateRenderingData(output_info, KK, camPose, template_mesh, mesh_trans);

    // compute normals
    if(trackerSettings.loadMesh)
    output_info.meshData.computeNormalsNeil();
    else
    output_info.meshData.computeNormals();
    
    TOCK( "updateRenderingLevel" + std::to_string( level ) );

    vector<bool>& visibility_mask = visibilityMaskPyramid[level];
    // update visibility mask if necessary
    if(trackerSettings.useVisibilityMask)
    {
        
        TICK( "updateVisbilityMaskLevel" + std::to_string( level ) );
        
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

        TOCK( "updateVisbilityMaskLevel" + std::to_string( level ) );
    }

    // need to update the color diff
    InternalIntensityImageType* color_image_split =
        imagePyramid.levels[level].colorImageSplit;
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

void DeformNRSFMTracker::AddPhotometricCost(ceres::Problem& problem,
    ceres::LossFunction* loss_function, dataTermErrorType errorType)
{
    // add photometric cost
    // there are two different cases
    vector<std::pair<int,int> >& data_pairs =
        pStrategy->optimizationSettings[currLevel].dataTermPairs;
    
    int num_data_pairs = data_pairs.size();
    
    for(int k = 0; k < num_data_pairs; ++k)
    {
        std::pair<int, int>& data_pair = data_pairs[k];

        CameraInfo* pCamera = &imagePyramid.camInfoLevels[data_pair.first];
        ImageLevel* pFrame = &imagePyramid.levels[data_pair.first];

        cout << "camera width and height " << pCamera->width << " " << pCamera->height << endl;

        cout << "dataTerm pair" << endl;
        cout << data_pair.first << "->" << data_pair.second << endl;

        PangaeaMeshData& templateMesh = templateMeshPyramid.levels[ data_pair.first ];
        MeshDeformation& meshTrans = meshTransPyramid[ data_pair.first ];

        vector<bool>& visibilityMask = visibilityMaskPyramid[ data_pair.first ];
        
        if(data_pair.first == data_pair.second )
        {
            for(int i = 0; i < templateMesh.numVertices; ++i)
            {
                if(visibilityMask[i])
                {
                    switch(errorType)
                    {
                        case PE_INTENSITY:
                            
                            problem.AddResidualBlock(
                                new ceres::AutoDiffCostFunction<ResidualImageProjection, 1, 3, 3, 3>(
                                    new ResidualImageProjection(1, &templateMesh.grays[i], &templateMesh.vertices[i][0],
                                        pCamera, pFrame, errorType)),
                                loss_function, &camPose[0], &camPose[3], &meshTrans[i][0]);
                            
                            break;

                        case PE_COLOR:

                            problem.AddResidualBlock(
                                new ceres::AutoDiffCostFunction<ResidualImageProjection, 3, 3, 3, 3>(
                                    new ResidualImageProjection(1, &templateMesh.colors[i][0], &templateMesh.vertices[i][0],
                                        pCamera, pFrame, errorType)),
                                loss_function, &camPose[0], &camPose[3], &meshTrans[i][0]);

                            break;
                    }
                }
            }
        }
        else
        {
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

                    ceres::DynamicAutoDiffCostFunction<ResidualImageProjectionDeform, 5>* cost_function =
                        new ceres::DynamicAutoDiffCostFunction< ResidualImageProjectionDeform, 5 >(
                            new ResidualImageProjectionDeform(
                                1,
                                &templateMesh.grays[i],
                                &templateMesh.vertices[i][0],
                                pCamera,
                                pFrame,
                                numNeighbors,
                                neighborWeights,
                                neighborVertices,
                                errorType ) );

                    for(int j = 0; j < 2*numNeighbors; ++j)
                    cost_function->AddParameterBlock(3);
                    
                    switch(errorType)
                    {
                        case PE_INTENSITY:
                            
                            cost_function->SetNumResiduals(1);
                            problem.AddResidualBlock(
                                cost_function,
                                loss_function,
                                parameter_blocks);
                            break;

                        case PE_COLOR:
                            
                            cost_function->SetNumResiduals(3);
                            problem.AddResidualBlock(
                                cost_function,
                                loss_function,
                                parameter_blocks);
                            break;
                    }
                }
            }
            
        }
    }
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
            for(int neighbor = 0; neighbor < meshNeighbors[vertex].size(); ++neighbor)
            {
                double weight = same_level ? 1 : meshWeights[vertex][neighbor];
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
    if(currentFrameNo != startFrameNo)
    {
        //
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
    WeightPara& weightPara = pStrategy->weightPara;
    WeightScale& weightScale = pStrategy->weightScale;

    WeightPara weightParaLevel;

    // get parameter weightings
    if(currLevel == 0)
    {
        weightParaLevel = weightPara;
    }
    else
    {
        weightParaLevel.dataTermWeight = weightPara.dataTermWeight *
            weightScale.dataTermScale[currLevel];
        weightParaLevel.tvTermWeight = weightPara.tvTermWeight *
            weightScale.tvTermScale[currLevel];
        weightParaLevel.tvRotTermWeight = weightPara.tvRotTermWeight *
            weightScale.tvTermScale[currLevel];
        weightParaLevel.arapTermWeight = weightPara.arapTermWeight *
            weightScale.arapTermScale[currLevel];
        weightParaLevel.inextentTermWeight = weightPara.inextentTermWeight *
            weightScale.inextentTermScale[currLevel];
        weightParaLevel.deformWeight = weightPara.deformWeight *
            weightScale.deformTermScale[currLevel];

        // always the same dataHuberWidth and tvHuberWidth
        weightParaLevel.dataHuberWidth = weightPara.dataHuberWidth;
        weightParaLevel.tvHuberWidth = weightPara.tvHuberWidth;
        weightParaLevel.tvRotHuberWidth = weightPara.tvRotHuberWidth;

        // rotWeight and transWeight
        weightParaLevel.rotWeight = weightPara.rotWeight *
            weightScale.rotScale[currLevel];
        weightParaLevel.transWeight = weightPara.transWeight *
            weightScale.transScale[currLevel];

    }

    // cout << "data term weight" << " : " << weightParaLevel.dataTermWeight << endl;
    // cout << "data huber width" << " : " << weightParaLevel.dataHuberWidth << endl;

    if(weightParaLevel.dataTermWeight)
    {
        ceres::LossFunction* pPhotometricLossFunction = NULL;
        if(weightParaLevel.dataHuberWidth)
        {
            pPhotometricLossFunction = new ceres::HuberLoss(
                weightParaLevel.dataHuberWidth);
        }
        ceres::ScaledLoss* photometricScaledLoss = new ceres::ScaledLoss(
            pPhotometricLossFunction, weightParaLevel.dataTermWeight,
            ceres::TAKE_OWNERSHIP);

        AddPhotometricCost(problem, photometricScaledLoss, PEType);
        //AddPhotometricCost(problem, NULL, PEType);
    }

    // totatl variation term
    if(weightParaLevel.tvTermWeight)
    {
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
    }

    // rotation total variation term
    // arap has to be turned on, otherwise there is no rotation variable
    if(weightParaLevel.arapTermWeight &&  weightParaLevel.tvRotTermWeight)
    {
        ceres::LossFunction* pRotTVLossFunction = NULL;

        if(trackerSettings.tvRotHuberWidth)
        {
            pRotTVLossFunction = new ceres::HuberLoss(trackerSettings.tvRotHuberWidth);
        }
        ceres::ScaledLoss* tvRotScaledLoss = new ceres::ScaledLoss(
            pRotTVLossFunction , weightParaLevel.tvRotTermWeight, ceres::TAKE_OWNERSHIP);

        AddRotTotalVariationCost(problem, tvRotScaledLoss);

    }

    // arap term
    if(weightParaLevel.arapTermWeight)
    {
        ceres::ScaledLoss* arapScaledLoss = new ceres::ScaledLoss(NULL,
            weightParaLevel.arapTermWeight, ceres::TAKE_OWNERSHIP);
        AddARAPCost(problem, arapScaledLoss);
    }

    // inextensibility term
    //    cout << "inextent weight: " << weightParaLevel.inextentTermWeight << endl;
    if(weightParaLevel.inextentTermWeight)
    {
        ceres::ScaledLoss* inextentScaledLoss = new ceres::ScaledLoss(NULL,
            weightParaLevel.inextentTermWeight, ceres::TAKE_OWNERSHIP);
        AddInextentCost(problem, inextentScaledLoss);
    }

    // deformation term
    //cout << "deform weight: " << weightParaLevel.deformWeight << endl;
    if(weightParaLevel.deformWeight)
    {
        ceres::ScaledLoss* deformScaledLoss = new ceres::ScaledLoss(NULL,
            weightParaLevel.deformWeight, ceres::TAKE_OWNERSHIP);
        AddDeformationCost(problem, deformScaledLoss);
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
    // solve the term and get solution
    ceres::Solver::Options options;
    options.max_num_iterations = trackerSettings.maxNumIterations[currLevel];
    options.linear_solver_type = mapLinearSolver(trackerSettings.linearSolver);
    options.minimizer_progress_to_stdout = trackerSettings.isMinimizerProgressToStdout;
    options.function_tolerance = trackerSettings.functionTolerances[currLevel];
    options.gradient_tolerance = trackerSettings.gradientTolerances[currLevel];
    options.parameter_tolerance = trackerSettings.parameterTolerances[currLevel];
    options.initial_trust_region_radius = trackerSettings.initialTrustRegionRadiuses[0];
    options.max_trust_region_radius = trackerSettings.maxTrustRegionRadiuses[currLevel];
    options.min_trust_region_radius = trackerSettings.minTrustRegionRadiuses[currLevel];
    options.min_relative_decrease = trackerSettings.minRelativeDecreases[currLevel];
    options.num_linear_solver_threads = trackerSettings.numLinearSolverThreads;
    options.num_threads = trackerSettings.numThreads;
    options.max_num_consecutive_invalid_steps = 0;

    EnergyCallback energy_callback = EnergyCallback();
    options.update_state_every_iteration = false;
    options.callbacks.push_back(&energy_callback);

    ceres::Solver::Summary summary;

    if(BAType == BA_MOTSTR && trackerSettings.doAlternation)
    {
        // fix the structure and optimize the motion
        AddConstantMask(problem, BA_STR); // make structure parameters constant

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