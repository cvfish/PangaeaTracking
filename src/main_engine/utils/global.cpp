#include "main_engine/utils/global.h"

bool existenceTest(string meshPath, string meshLevelFormat, int frame,
    IntegerContainerType& meshLevelList)
{
    char buffer[BUFFER_SIZE];
    for(int i = 0; i < meshLevelList.size(); ++i)
    {
        std::stringstream meshFile;
        sprintf(buffer, meshLevelFormat.c_str(), frame, meshLevelList[i]);
        meshFile << meshPath << buffer;
        if(!bfs::exists(meshFile.str())){
            cout << "File not existing: " << meshFile.str() << endl;
            return false;
        }
    }
    return true;
}

bool pointInTriangleTest2(double* pointP, double* pointA, double* pointB, double* pointC)
{
    double A, B , D, E, C, F;
    A = pointA[0] - pointC[0];
    B = pointB[0] - pointC[0];
    D = pointA[1] - pointC[1];
    E = pointB[1] - pointC[1];

    C = pointC[0] - pointP[0];
    F = pointC[1] - pointP[1];

    double alpha, beta, gamma;
    alpha = (B*(F)-C*(E))/(A*(E)-B*(D));
    beta  = (A*(F)-C*(D))/(B*(D)-A*(E));
    gamma = 1 - alpha - beta;

    return (alpha >=0 && beta >=0 && gamma >= 0);

}

double triangleArea(double* pointA, double* pointB, double* pointC)
{
    double a = sqrt((pointB[0] - pointC[0])*(pointB[0] - pointC[0]) +
        (pointB[1] - pointC[1])*(pointB[1] - pointC[1]));

    double b = sqrt((pointA[0] - pointC[0])*(pointA[0] - pointC[0]) +
        (pointA[1] - pointC[1])*(pointA[1] - pointC[1]));

    double c = sqrt((pointB[0] - pointA[0])*(pointB[0] - pointA[0]) +
        (pointB[1] - pointA[1])*(pointB[1] - pointA[1]));

    double s = (a + b + c)/2;

    double area = sqrt(s*(s-a)*(s-b)*(s-c));

    return area;

}

bool pointInTriangleTest(double* pointP, double* pointA, double* pointB, double* pointC)
{
    // get the barycentric coordinate
    double AB[3], AC[3], PB[3], PC[3], PA[3];
    double areaABC = triangleArea(pointA, pointB, pointC);
    double areaABP = triangleArea(pointA, pointB, pointP);
    double areaACP = triangleArea(pointA, pointC, pointP);
    double alpha = areaABP / areaABC;
    double beta = areaACP / areaABC;
    double gamma = 1 - alpha - beta;
    return (alpha >=0 && beta >=0 && gamma >= 0);
}

bool visibilityTest(double* vertex, double* center, double* normal,
    double* vertex1, double* vertex2, double* vertex3)
{
    // tell if a point is in front of a triangle face
    double faceDist, pointDist, scale;
    faceDist = center[0]*normal[0] + center[1]*normal[1] + center[2]*normal[2];
    pointDist = vertex[0]*normal[0] + vertex[1]*normal[1] + vertex[2]*normal[2];
    scale = faceDist / pointDist;

    double intersection[3];
    intersection[0] = scale*vertex[0];
    intersection[1] = scale*vertex[1];
    intersection[2] = scale*vertex[2];

    // if the intersection is in front,
    // then the test vertex is occluded by
    // the front face, we give up
    if(intersection[2] < vertex[2])
    return false;
    else
    return true;

}

ImageSourceType mapImageSourceType(std::string const& inString)
{
  cout << inString << endl;

  if (inString == "ImagesBuffer") return ALLIMAGESBUFFER;
  if (inString == "ImageSequence") return IMAGESEQUENCE;
  if (inString == "Video") return VIDEO;
  if (inString == "Camera") return CAMERA;
}

TrackingType mapTrackingType(std::string const& inString)
{
  cout << inString << endl;

  if (inString == "ShapesBuffer") return ALLSHAPESBUFFER;
  if (inString == "ShapeSequence") return SHAPESEQUENCE;
  if (inString == "DeformNRSFMTracker") return DEFORMNRSFM;
  if (inString == "MeshSequence") return MESHSEQUENCE;
  if (inString == "MeshPyramid") return MESHPYRAMID;
  if (inString == "MeshBuffer") return MESHBUFFER;
}

FeatureType mapFeatureType(std::string const& inString)
{
  cout << inString << endl;

  if (inString == "sift") return FT_SIFT;
  if (inString == "bitplane") return FT_BITPLANE;
  if (inString == "gray") return FT_GRAYSCALE;
  if (inString == "color") return FT_COLOR;

}

int typeConvert(string dataType)
{
  if(dataType == "unsigned char") return CV_8U;
  if(dataType == "float") return CV_32F;
  if(dataType == "double") return CV_64F;
}

int typeSize(string dataType)
{
  if(dataType == "unsigned char") return 1;
  if(dataType == "float") return 4;
  if(dataType == "double") return 8;
}

// points1, points2,
void computeRot(vector<double>& template_vextex, vector<double>& vertex,
    vector<vector<double> >& template_nbor_vertices, vector<vector<double > >& nbor_vertices,
    vector<unsigned int>& neighbors, vector<double>& weights,
    vector<double>& output_rot, bool deform)
{
    int num_neighbors = neighbors.size();
    MatrixXd Xt(3, num_neighbors), X(3, num_neighbors);

    Map<VectorXd> w(weights.data(), num_neighbors);
    Eigen::VectorXd w_normalized = w/w.sum();
    Eigen::Vector3d Xt_mean, X_mean;

    for(int i = 0; i < num_neighbors; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            Xt(j,i) = template_vextex[j] - template_nbor_vertices[ neighbors[i] ][j];
            X(j,i) = vertex[j] - nbor_vertices[ neighbors[i] ][j];

            if(deform)
            X(j,i) += Xt(j,i);
        }
    }

    // std::cout << Xt << endl;
    // std::cout << X << endl;

    for(int i=0; i<3; ++i) {
      Xt_mean(i) = (Xt.row(i).array()*w_normalized.transpose().array()).sum();
      X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
    }

    Xt.colwise() -= Xt_mean;
    X.colwise() -= X_mean;

    // std::cout << Xt_mean << endl;
    // std::cout << X_mean << endl;
    // std::cout << Xt << endl;
    // std::cout << X << endl;
    // std::cout << w_normalized << endl;

    Eigen::Matrix3d sigma = Xt * w_normalized.asDiagonal() * X.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d Rot;

    if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
        Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
        Rot = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
    } else {
        Rot = svd.matrixV()*svd.matrixU().transpose();
    }

    // ceres::RotationMatrixToAngleAxis(&Rot(0, 0), &output_rot[0]);

    Eigen::AngleAxisd angleAxis(Rot);
    for(int i = 0; i < 3; ++i)
    output_rot[i] = angleAxis.axis()[i] * angleAxis.angle();

}

// memory logging
void process_mem_usage(double& vm_usage, double& resident_set)
{
   using std::ios_base;
   using std::ifstream;
   using std::string;

   vm_usage     = 0.0;
   resident_set = 0.0;

   // 'file' stat seems to give the most reliable results
   //
   ifstream stat_stream("/proc/self/stat",ios_base::in);

   // dummy vars for leading entries in stat that we don't care about
   //
   string pid, comm, state, ppid, pgrp, session, tty_nr;
   string tpgid, flags, minflt, cminflt, majflt, cmajflt;
   string utime, stime, cutime, cstime, priority, nice;
   string O, itrealvalue, starttime;

   // the two fields we want
   //
   unsigned long vsize;
   long rss;

   stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
               >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
               >> utime >> stime >> cutime >> cstime >> priority >> nice
               >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

   stat_stream.close();

   long page_size_kb = (long)boost::interprocess::mapped_region::get_page_size() / 1024; // in case x86-64 is configured to use 2MB pages
   vm_usage     = vsize / 1024.0;
   resident_set = rss * page_size_kb;
}

// remove file extension
// use '.' to recognize file extension
void remove_file_extension(std::string file, std::string& file_wo_extension)
{
  std::size_t found = file.find('.');
  if(found == std::string::npos)
    file_wo_extension = file;
  else
    file_wo_extension = file.substr(0, found);

}
