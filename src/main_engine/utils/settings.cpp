#include "main_engine/utils/settings.h"

void read(const cv::FileNode& node, std::string& value, const char* default_value)
{
    if(!node[default_value].empty())
    node[default_value] >> value;
}

void read(const cv::FileNode& node, vector<vector< pair<int, int> > >& termPair)
{

    cv::FileNodeIterator it = node.begin(), it_end = node.end();
    for( ; it != it_end; ++it)
    {
        vector<int> first_part;
        vector<int> second_part;
        (*it)["first"] >> first_part;
        (*it)["second"] >> second_part;
        vector<pair<int, int> > first_second;
        first_second.resize(first_part.size());
        for(int j = 0; j < first_part.size(); ++j)
        {
            first_second[j].first =  first_part[j];
            first_second[j].second = second_part[j];
        }
        termPair.push_back(first_second);
    }
    
}

void read(const cv::FileNode& node, ImageSourceSettings& settings,
    const ImageSourceSettings& default_settings)
{
    if(node.empty())
    settings = default_settings;
    else
    settings.read(node);
}

void read(const cv::FileNode& node, ShapeLoadingSettings& settings,
    const ShapeLoadingSettings& default_settings)
{
    if(node.empty())
    settings = default_settings;
    else
    settings.read(node);
}

void read(const cv::FileNode& node, MeshLoadingSettings& settings,
    const MeshLoadingSettings& default_settings)
{
    if(node.empty())
    settings = default_settings;
    else
    settings.read(node);
}

void read(const cv::FileNode& node, TrackerSettings& settings,
    const TrackerSettings& default_settings)
{
    if(node.empty())
    settings = default_settings;
    else
    settings.read(node);
}

ImageSourceSettings::ImageSourceSettings()
{
    dataPath = "/home/cvfish/Work/data/pangaea_tracking_data/test/input/";
    imageFormat = "rgb%04d.png";
    intrinsicsFile = "intrinsics.txt";   // probably should consider distortion as well
    width = 1280;
    height = 720;
    startFrame = 1;

    isOrthoCamera = false;

    frameStep = 1;
}

void ImageSourceSettings::read(const cv::FileNode& node)
{
    // read(node, datapath, "datapath");
    // read(node, imageformat, "imageformat");
    // read(node, intrinsicsfile, "intrinsicsfile");

    // dataPath = node["dataPath"];
    // imageFormat = node["imageFormat"];
    // intrinsicsFile = node["intrinsicsFile"];

    if(!node["dataPath"].empty())
    node["dataPath"] >> dataPath;

    if(!node["imageFormat"].empty())
    node["imageFormat"] >> imageFormat;

    if(!node["intrinsicsFile"].empty())
    node["intrinsicsFile"] >> intrinsicsFile;

    if(!node["width"].empty())
    node["width"] >> width;

    if(!node["height"].empty())
    node["height"] >> height;

    if(!node["startFrame"].empty())
    node["startFrame"] >> startFrame;

    if(!node["numFrames"].empty())
    node["numFrames"] >> numFrames;

    if(!node["isOrthoCamera"].empty())
    node["isOrthoCamera"] >> isOrthoCamera;

    // read calibration matrix from intrinsics file
    std::stringstream intrinsicsFileName;
    intrinsicsFileName << dataPath << intrinsicsFile;
    ifstream intrinsicsFileFID(intrinsicsFileName.str().c_str());
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            intrinsicsFileFID >> KK[i][j];
            std::cout << KK[i][j] << " ";
        }
        std::cout << std::endl;
    }

    if(!node["frameStep"].empty())
    node["frameStep"] >> frameStep;

}

ShapeLoadingSettings::ShapeLoadingSettings()
{
    hasGT = false;
    resultsPath = "/home/cvfish/Work/data/pangaea_tracking_data/test/result/";
    shapeFormat = "pnts3D_solTOT_frInd%03d.txt";
    shapeFormatGT = "pnts3D_GT_frInd%03d.txt";
    gtModelFile = "labelsGT_per3Dpnt.txt";
    solModelFile = "labelsSol_per3Dpnt.txt";
    shapeMaskFile = "mask.raw";
    labelColorFile = "colors_labels.txt";
    shapeColMajor = true;
    loadShapeMask = true;
    shapeSamplingScale = 0.25;

    modelNum = 1;

}

void ShapeLoadingSettings::read(const cv::FileNode& node)
{
    if(!node["hasGT"].empty())
    node["hasGT"] >> hasGT;

    if(!node["resultsPath"].empty())
    node["resultsPath"] >> resultsPath;

    if(!node["shapeFormat"].empty())
    node["shapeFormat"] >> shapeFormat;

    if(!node["shapeFormatGT"].empty())
    node["shapeFormatGT"] >> shapeFormatGT;

    if(!node["gtModelFile"].empty())
    node["gtModelFile"] >> gtModelFile;

    if(!node["solModelFile"].empty())
    node["solModelFile"] >> solModelFile;

    if(!node["shapeMaskFile"].empty())
    node["shapeMaskFile"] >> shapeMaskFile;

    if(!node["labelColorFile"].empty())
    node["labelColorFile"] >> labelColorFile;

    if(!node["loadShapeMask"].empty())
    node["loadShapeMask"] >> loadShapeMask;

    if(!node["shapeColMajor"].empty())
    node["shapeColMajor"] >> shapeColMajor;

    if(!node["shapeSamplingScale"].empty())
    node["shapeSamplingScale"] >> shapeSamplingScale;

    if(!node["modelNum"].empty())
    node["modelNum"] >> modelNum;

}

MeshLoadingSettings::MeshLoadingSettings()
{
    meshPath = "/home/cvfish/Work/data/pangaea_tracking_data/test/result/";
    meshFormat = "mesh%04d.obj";
    visibilityMask = true;

    meshLevelFormat = "mesh%04d_level%02d.obj";
    propLevelFormat = "prop_mesh%04d_level%02d.obj";

    loadProp = true;
    fastLoading = true;

    // By default, faces are supposed to be defined clockwise to support the 
	// original implementation of Pangaea
	clockwise = true;
}

void MeshLoadingSettings::read(const cv::FileNode& node)
{
    if(!node["meshPath"].empty())
    node["meshPath"] >> meshPath;

    if(!node["meshFormat"].empty())
    node["meshFormat"] >> meshFormat;

    if(!node["visibilityMask"].empty())
    node["visibilityMask"] >> visibilityMask;

    if(!node["meshLevelFormat"].empty())
    node["meshLevelFormat"] >> meshLevelFormat;

    if(!node["propLevelFormat"].empty())
    node["propLevelFormat"] >> propLevelFormat;

    if(!node["meshLevelList"].empty())
    node["meshLevelList"] >> meshLevelList;

    if(!node["loadProp"].empty())
    node["loadProp"] >> loadProp;

    if(!node["fastLoading"].empty())
    node["fastLoading"] >> fastLoading;

    // Read if faces are defined clockwise or anti-clockwise
	if (!node["clockwise"].empty())
    node["clockwise"] >> clockwise;
}

TrackerSettings::TrackerSettings()
{

    // default value for tracking
    errorType = "gray";
    baType = "motstr";
    
    isRigid = false;
    doAlternation = true;
    updateColor = false;
    useVisibilityMask = false;
    useOpenGLMask = true;
    fastARAP = false;
    onlyDeformDepthPrior = false;
    useXYZ = true;
    isOrthoCamera = false;
    loadMesh = false;
    depth2MeshScale = 1.0;

    weightPhotometric = 500;
    weightTV = 0.5;
    weightRotTV = 0;
    weightDeform = 0;
    weightGradient = 0;
    weightARAP = 0;
    weightINEXTENT = 0;
    weightTransPrior = 0;
    photometricHuberWidth = 0.1;
    tvHuberWidth = 0.2;
    tvRotHuberWidth = 0.2;
    meshScaleUpFactor = 1.0;

    // ceres parameter
    linearSolver = "CG";
    numOptimizationLevels = 3;
    numOptimizationLevelsToDo = 1;
    numLinearSolverThreads = 8;
    numThreads = 8;
    isMinimizerProgressToStdout = true;

    // debugging
    saveResults = false;
    ceresOutputFile = "ceres_output.txt";
    diffFileFormat = "diff%04d.png";
    savePath = "/home/cvfish/Work/data/pangaea_tracking_data/test/tracker/";

    saveMesh = false;
    meshFormat = "mesh%04d.obj";

    saveMeshPyramid = false;
    meshPyramidFormat = "mesh%04d_level%02d.obj";
    propPyramidFormat = "prop_mesh%04d_level%02d.obj";

    savePropPyramid = false;

    showWindow = true;

    // meshPyramid
    meshLevelFormat = "";
    meshPyramidUseRadius = false;

    // 
    useDepthPyramid = false; 
    usePrevForTemplateInTV = false;
    
    tvTukeyWidth = 0;

    //
    minimizerType = "TRUST_REGION";
    lineSearchDirectionType = "LBFGS";
    lineSearchType = "WOLFE";
    nonlinearConjugateGradientType = "FLETCHER_REEVES";
    lineSearchInterpolationType = "CUBIC";

	// By default, faces are supposed to be defined clockwise to support the 
	// original implementation of Pangaea
	clockwise = true;
}

void TrackerSettings::read(const cv::FileNode& node)
{

    if(!node["error_type"].empty())
    node["error_type"] >> errorType;

    if(!node["ba_type"].empty())
    node["ba_type"] >> baType;

    if(!node["mesh_file"].empty())
    node["mesh_file"] >> meshFile;
    
    if(!node["rigid_sequence"].empty())
    node["rigid_sequence"] >> isRigid;

    if(!node["do_alternation"].empty())
    node["do_alternation"] >> doAlternation;

    if(!node["update_color"].empty())
    node["update_color"] >> updateColor;

    if(!node["use_visibility_mask"].empty())
    node["use_visibility_mask"] >> useVisibilityMask;

    if(!node["use_opengl_mask"].empty())
    node["use_opengl_mask"] >> useOpenGLMask;

    if(!node["arap_fast"].empty())
    node["arap_fast"] >> fastARAP;

    if(!node["only_deform_depth_prior"].empty())
    node["only_deform_depth_prior"] >> onlyDeformDepthPrior;

    if(!node["use_xyz"].empty())
    node["use_xyz"] >> useXYZ;

    if(!node["load_mesh"].empty())
    node["load_mesh"] >> loadMesh;

    if(!node["depth2mesh_scale"].empty())
    node["depth2mesh_scale"] >> depth2MeshScale;

    if(!node["photometric_weight"].empty())
    node["photometric_weight"] >> weightPhotometric;

    if(!node["tv_weight"].empty())
    node["tv_weight"] >> weightTV;

    if(!node["rot_tv_weight"].empty())
    node["rot_tv_weight"] >> weightRotTV;

    if(!node["deform_weight"].empty())
    node["deform_weight"] >> weightDeform;

    if(!node["grad_weight"].empty())
    node["grad_weight"] >> weightGradient;

    if(!node["arap_weight"].empty())
    node["arap_weight"] >> weightARAP;

    if(!node["inextent_weight"].empty())
    node["inextent_weight"] >> weightINEXTENT;

    if(!node["trans_weight"].empty())
    node["trans_weight"] >> weightTransPrior;

    if(!node["photometric_huber_width"].empty())
    node["photometric_huber_width"] >> photometricHuberWidth;

    if(!node["tv_huber_width"].empty())
    node["tv_huber_width"] >> tvHuberWidth;

    if(!node["rot_tv_huber_width"].empty())
    node["rot_tv_huber_width"] >> tvRotHuberWidth;

    if(!node["mesh_scale_up_factor"].empty())
    node["mesh_scale_up_factor"] >> meshScaleUpFactor;

    // ceres
    if(!node["linear_solver"].empty())
    node["linear_solver"] >> linearSolver;

    if(!node["numOptimizationLevels"].empty())
    node["numOptimizationLevels"] >> numOptimizationLevels;

    if(!node["numOptimizationLevelsToDo"].empty())
    node["numOptimizationLevelsToDo"] >> numOptimizationLevelsToDo;

    //Read the blur filter size at every pyramid level
    if(!node["blurFilterSize (at each level)"].empty())
    node["blurFilterSize (at each level)"] >> blurFilterSizes;

    if(!node["imagePyramidSamplingFactors (at each level)"].empty())
    node["imagePyramidSamplingFactors (at each level)"] >> imagePyramidSamplingFactors;
    
    //Read the scaling factor for each gradient image at each level
    if(!node["imageGradientsScalingFactor (at each level)"].empty())
    node["imageGradientsScalingFactor (at each level)"] >> imageGradientScalingFactors;

    //Read the number of Levenberg-Marquardt iterations at each optimization level
    if(!node["max_num_iterations (at each level)"].empty())
    node["max_num_iterations (at each level)"] >> maxNumIterations;

    //Read optimizer function tolerance at each level
    if(!node["function_tolerance (at each level)"].empty())
    node["function_tolerance (at each level)"] >> functionTolerances;

    //Read optimizer gradient tolerance at each level
    if(!node["gradient_tolerance (at each level)"].empty())
    node["gradient_tolerance (at each level)"] >> gradientTolerances;

    //Read optimizer parameter tolerance at each level
    if(!node["parameter_tolerance (at each level)"].empty())
    node["parameter_tolerance (at each level)"] >> parameterTolerances;

    //Read optimizer initial trust region at each level
    if(!node["initial_trust_region_radius (at each level)"].empty())
    node["initial_trust_region_radius (at each level)"] >> initialTrustRegionRadiuses;

    //Read optimizer max trust region radius at each level
    if(!node["max_trust_region_radius (at each level)"].empty())
    node["max_trust_region_radius (at each level)"] >> maxTrustRegionRadiuses;

    //Read optimizer min trust region radius at each level
    if(!node["min_trust_region_radius (at each level)"].empty())
    node["min_trust_region_radius (at each level)"] >> minTrustRegionRadiuses;

    //Read optimizer min LM relative decrease at each level
    if(!node["min_relative_decrease (at each level)"].empty())
    node["min_relative_decrease (at each level)"] >> minRelativeDecreases;

    //Read the number of threads for the linear solver
    if(!node["num_linear_solver_threads"].empty())
    node["num_linear_solver_threads"] >> numLinearSolverThreads;

    //Read the number of threads for the jacobian computation
    if(!node["num_threads"].empty())
    node["num_threads"] >> numThreads;

    // debugging
    if(!node["save_results"].empty())
    node["save_results"] >> saveResults;
    if(!node["ceres_output"].empty())
    node["ceres_output"] >> ceresOutputFile;
    if(!node["diff_format"].empty())
    node["diff_format"] >> diffFileFormat;
    if(!node["savePath"].empty())
    node["savePath"] >> savePath;
    if(!node["save_mesh"].empty())
    node["save_mesh"] >> saveMesh;
    if(!node["meshFormat"].empty())
    node["meshFormat"] >> meshFormat;

    if(!node["save_mesh_pyramid"].empty())
    node["save_mesh_pyramid"] >> saveMeshPyramid;
    if(!node["meshPyramidFormat"].empty())
    node["meshPyramidFormat"] >> meshPyramidFormat;

    if(!node["save_prop_pyramid"].empty())
    node["save_prop_pyramid"] >> savePropPyramid;
    if(!node["propPyramidFormat"].empty())
    node["propPyramidFormat"] >> propPyramidFormat;

    if(!node["show_energy"].empty())
    node["show_energy"] >> isMinimizerProgressToStdout;

    // show window or not
    if(!node["show_window"].empty())
    node["show_window"] >> showWindow;

    // mesh level format
    if(!node["mesh_pyramid_file"].empty())
    node["mesh_pyramid_file"] >> meshLevelFormat;
    if(!node["mesh_pyramid_vertex_num"].empty())
    node["mesh_pyramid_vertex_num"] >> meshVertexNum;
    if(!node["mesh_pyramid_neighbor_num"].empty())
    node["mesh_pyramid_neighbor_num"] >> meshNeighborNum;
    if(!node["mesh_pyramid_neighbor_radius"].empty())
    node["mesh_pyramid_neighbor_radius"] >> meshNeighborRadius;
    if(!node["mesh_pyramid_use_radius"].empty())
    node["mesh_pyramid_use_radius"] >> meshPyramidUseRadius;

    if(!node["use_depth_pyramid"].empty())
    node["use_depth_pyramid"] >> useDepthPyramid;

    if(!node["use_prev_for_template_in_tv"].empty())
    node["use_prev_for_template_in_tv"] >> usePrevForTemplateInTV;

    if(!node["tv_tukey_width"].empty())
    node["tv_tukey_width"] >> tvTukeyWidth;

    // arbitary neighbor mesh for all the related terms
    if(!node["data_term_pair"].empty())
    ::read(node["data_term_pair"], dataTermPair);
    // node["data_term_pair"] >> dataTermPair;

    if(!node["reg_term_pair"].empty())
    ::read(node["reg_term_pair"], regTermPair);
    // node["reg_term_pair"] >> regTermPair;

    // added stuff to support line search
    if(!node["minimizer_type"].empty())
    node["minimizer_type"] >> minimizerType;

    if(!node["line_search_direction_type"].empty())
    node["line_search_direction_type"] >> lineSearchDirectionType;

    if(!node["line_search_type"].empty())
    node["line_search_type"] >> lineSearchType;

    if(!node["nonlinear_conjugate_gradient_type"].empty())
    node["nonlinear_conjugate_gradient_type"] >> nonlinearConjugateGradientType;
    
    if(!node["line_search_interpolation_type"].empty())
    node["line_search_interpolation_type"] >> lineSearchInterpolationType;

	// Read if faces are defined clockwise or anti-clockwise
	if (!node["clockwise"].empty())
		node["clockwise"] >> clockwise;
}

ImageSourceType imageSourceType = IMAGESEQUENCE;
TrackingType trackingType = DEFORMNRSFM;

ImageSourceSettings imageSourceSettings;
ShapeLoadingSettings shapeLoadingSettings;
MeshLoadingSettings meshLoadingSettings;
TrackerSettings trackerSettings;
