#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "hdf5.h"
#include <lmdb.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <set>

#include <GL/glew.h>
#include <GL/glut.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/core/operations.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/mapped_region.hpp>

#ifdef _MSC_VER
#include "third_party/msvc/Stopwatch.h"
#else
#include "third_party/Stopwatch.h"
#endif

#ifdef Success
#undef Success
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/OpenGLSupport>

using namespace std;
using namespace Eigen;

typedef unsigned int uint;
typedef unsigned char uchar;

#define msg cout
#define dbg cout
#define err cerr

typedef double CoordinateType;

typedef unsigned char PixelType;
typedef std::vector< CoordinateType >             CoordinateContainerType;
typedef std::vector< int >             IntegerContainerType;
typedef std::vector<vector< pair<int, int> > > VecVecPairType;
typedef std::vector<vector< vector<int> > > VecVecVecType;
typedef vector<vector<double> > MeshDeformation;

typedef cv::Vec<PixelType,3> Vec3b;
typedef cv::Vec<CoordinateType,3> Vec3d;

typedef cv::Mat_< PixelType > IntensityImageType;
typedef cv::Mat_< CoordinateType > DepthImageType;
typedef cv::Mat_< CoordinateType > FeatureImageType;
typedef DepthImageType InternalIntensityImageType;
typedef cv::Mat_<Vec3b> ColorImageType;
typedef cv::Mat_<Vec3d> InternalColorImageType;

typedef std::vector<ColorImageType> ColorImageContainerType;
typedef std::vector< FeatureImageType > FeatureImageContainerType;

typedef Matrix<CoordinateType, Dynamic, Dynamic, RowMajor> MatrixXCordRow;
typedef Matrix<float, Dynamic, Dynamic, RowMajor> MatrixXfRow;
typedef Matrix<double, Dynamic, Dynamic, RowMajor> MatrixXdRow;

#define btime boost::posix_time
#define bfs boost::filesystem

#define BUFFER_SIZE 1000

enum ImageSourceType{
  ALLIMAGESBUFFER,
  IMAGESEQUENCE,
  VIDEO,
  CAMERA
};

enum TrackingType{
  ALLSHAPESBUFFER,
  SHAPESEQUENCE,
  MESHSEQUENCE,
  MESHPYRAMID,
  MESHBUFFER,
  DEFORMNRSFM
};

enum FeatureType{
  FT_SIFT,
  FT_BITPLANE,
  FT_GRAYSCALE,
  FT_COLOR
};

ImageSourceType mapImageSourceType(std::string const& inString);

TrackingType mapTrackingType(std::string const& inString);

FeatureType mapFeatureType(std::string const& inString);

int typeConvert(string dataType);

int typeSize(string dataType);

#define SafeAllocArray(x,size) x = new unsigned char[size]; memset(x, 0, size);
#define SafeAllocArrayType(x,size,tp) x = new tp[size]; memset(x, 0, size*sizeof(tp));

#define SafeResetArray(x,size) if(x) { memset(x,0,size); }
#define SafeResetArrayType(x,size,tp) if(x) { memset(x,0, (size*sizeof(tp))); }

#define SafeDeleteArray(x) if(x) {delete []x; x = NULL; }

bool existenceTest(string meshPath, string meshLevelFormat,
                   int frame, IntegerContainerType& meshLevelList);

bool pointInTriangleTest2(double* pointP, double* pointA, double* pointB, double* pointC);

double triangleArea(double* pointA, double* pointB, double* pointC);

bool visibilityTest(double* vertex, double* center, double* normal,
                    double* vertex1, double* vertex2, double* vertex3);

void computeRot(vector<double>& template_vextex, vector<double>& vertex,
                vector<vector<double> >& template_nbor_mesh, vector<vector<double > >& nbor_mesh,
                vector<unsigned int>& neighbors, vector<double>& weights,
                vector<double>& output_rot, bool deform = false);

// template functions
template<typename FloatType>
FloatType mynorm(FloatType* loc)
{
  return sqrt(loc[0]*loc[0]+loc[1]*loc[1]+loc[2]*loc[2]);
}

template<typename FloatType>
void flipnorm(FloatType* normals, int num)
{
  for(int i = 0; i < num; ++i)
    normals[i] *= -1;
}

template<typename FloatType>
void compnorm(FloatType* ver1,  FloatType* ver2, FloatType* ver3, FloatType* location,
              bool clockwise)
{
  // compute normals assume that the normal at each point
  //   is defined by the triangle consisting of the previous two
  //   points + current point.
  //   (p1-p3) x (p1-p2)

	FloatType norm[3];
	FloatType a[3];
	FloatType b[3];

	if (clockwise)
    {
      a[0] = ver1[0] - ver3[0];
      a[1] = ver1[1] - ver3[1];
      a[2] = ver1[2] - ver3[2];

      b[0] = ver1[0] - ver2[0];
      b[1] = ver1[1] - ver2[1];
      b[2] = ver1[2] - ver2[2];
    }
	else	// Anti-clockwsie
    {
      a[0] = ver1[0] - ver2[0];
      a[1] = ver1[1] - ver2[1];
      a[2] = ver1[2] - ver2[2];

      b[0] = ver1[0] - ver3[0];
      b[1] = ver1[1] - ver3[1];
      b[2] = ver1[2] - ver3[2];
    }

  norm[0] = a[1]*b[2] - a[2]*b[1];
  norm[1] = a[2]*b[0] - a[0]*b[2];
  norm[2] = a[0]*b[1] - a[1]*b[0];

	if (norm[1] * norm[1] + norm[2] * norm[2] + norm[0] * norm[0] != FloatType(0))
    {
      FloatType temp = FloatType(1.0f) / sqrt(norm[1] * norm[1] + norm[2] * norm[2] + norm[0] * norm[0]);
      norm[0] *= temp;
      norm[1] *= temp;
      norm[2] *= temp;
    }

  location[0] = norm[0];
  location[1] = norm[1];
  location[2] = norm[2];
}

template<typename FloatType>
bool checksize(FloatType* location1, FloatType* location2, FloatType* location3, GLfloat thresh)
{
  if((abs(location1[0] - location2[0]) + abs(location1[0] - location3[0]) > thresh*100) ||
     (abs(location1[1] - location2[1]) + abs(location1[1] - location3[1]) > thresh*100) ||
     (abs(location1[2] - location2[2]) + abs(location1[2] - location3[2]) > thresh*100))
    return false;
  else
    return true;
}

// template linear interpolation
template< typename TPixel >
void LinearInitAxis( TPixel x, int size,
                     int *x1, int *x2,
                     TPixel *dx )
{
  const int ix = static_cast<int>(x);
  if (ix < 0) {
    *x1 = 0;
    *x2 = 0;
    *dx = 1.0;
  } else if (ix > size - 2) {
    *x1 = size - 1;
    *x2 = size - 1;
    *dx = 1.0;
  } else {
    *x1 = ix;
    *x2 = ix + 1;
    *dx = *x2 - x;
  }
}

// Linear interpolation
template< typename T, class TImage >
void SampleLinear( const TImage & intensityImage,
                   typename TImage::value_type y,
                   typename TImage::value_type x, T* sample )
{
  typedef TImage ImageType;
  typedef typename ImageType::value_type PixelType;

  int x1, y1, x2, y2;
  PixelType dx, dy;

  // Take the upper left corner as integer pixel positions.
  /* x -= 0.5; */
  /* y -= 0.5; */

  // x -= 1.0;
  // y -= 1.0;

  LinearInitAxis(y, intensityImage.rows, &y1, &y2, &dy);
  LinearInitAxis(x, intensityImage.cols,  &x1, &x2, &dx);

  //Sample intensity
  const T im11 = T(intensityImage(y1, x1));
  const T im12 = T(intensityImage(y1, x2));
  const T im21 = T(intensityImage(y2, x1));
  const T im22 = T(intensityImage(y2, x2));

  sample[0] =(     dy  * ( dx * im11 + (1.0 - dx) * im12 ) +
                   (1 - dy) * ( dx * im21 + (1.0 - dx) * im22 ));

}

/// Linear interpolation.
template< typename T, class TImage >
void SampleLinear( const TImage & intensityImage,
                   const TImage & intensityGradientX,
                   const TImage & intensityGradientY,
                   typename TImage::value_type y,
                   typename TImage::value_type x, T* sample )
{
  typedef TImage ImageType;
  typedef typename ImageType::value_type PixelType;

  int x1, y1, x2, y2;
  PixelType dx, dy;

  // Take the upper left corner as integer pixel positions.
  /* x -= 0.5; */
  /* y -= 0.5; */

  // x -= 1.0;
  // y -= 1.0;

  LinearInitAxis(y, intensityImage.rows, &y1, &y2, &dy);
  LinearInitAxis(x, intensityImage.cols,  &x1, &x2, &dx);

  //Sample intensity
  const T im11 = T(intensityImage(y1, x1));
  const T im12 = T(intensityImage(y1, x2));
  const T im21 = T(intensityImage(y2, x1));
  const T im22 = T(intensityImage(y2, x2));

  sample[0] =(     dy  * ( dx * im11 + (1.0 - dx) * im12 ) +
                   (1 - dy) * ( dx * im21 + (1.0 - dx) * im22 ));

  //Sample gradient x
  const T gradx11 = T(intensityGradientX(y1, x1));
  const T gradx12 = T(intensityGradientX(y1, x2));
  const T gradx21 = T(intensityGradientX(y2, x1));
  const T gradx22 = T(intensityGradientX(y2, x2));

  sample[1] =(     dy  * ( dx * gradx11 + (1.0 - dx) * gradx12 ) +
                   (1 - dy) * ( dx * gradx21 + (1.0 - dx) * gradx22 ));

  //Sample gradient y
  const T grady11 = T(intensityGradientY(y1, x1));
  const T grady12 = T(intensityGradientY(y1, x2));
  const T grady21 = T(intensityGradientY(y2, x1));
  const T grady22 = T(intensityGradientY(y2, x2));

  sample[2] =(     dy  * ( dx * grady11 + (1.0 - dx) * grady12 ) +
                   (1 - dy) * ( dx * grady21 + (1.0 - dx) * grady22 ));
}


// memory logging
void process_mem_usage(double& vm_usage, double& resident_set);

// remove file extension
// use '.' to recognize file extension
void remove_file_extension(std::string file, std::string& file_wo_extension);

#endif
