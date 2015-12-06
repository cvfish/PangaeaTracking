#PangaeaTracking for MSVC

This is Microsoft Visual Studio project to compile PangaeaTracking on Windows.

1. The following libraries will be required to be compiled in your machine:

  - OpenCV
      available at https://github.com/Itseez/opencv
  - Ceres Solver
      available at https://ceres-solver.googlesource.com/ceres-solver/
  - SuiteSparse
      compiled from Jose Luis Blanco-Claraco's msvc project: https://github.com/jlblancoc/suitesparse-metis-for-windows
  - glog
      available at https://github.com/google/glog
  - Eigen
      available at http://eigen.tuxfamily.org/
  - Boost
      available at http://www.boost.org/
  - freeGLUT
      available at http://freeglut.sourceforge.net/
  - GLEW
      available at http://glew.sourceforge.net/
  - wxWidgets
      available at https://www.wxwidgets.org

2. Open the msvc project solution 'PangaeaTracking.sln' and make sure the following header directories are correct in each project. Change environmental variables and/or directories if required:

  $(OPENCV_PATH)\include
  $(EIGEN_PATH)
  $(EIGEN_PATH)\unsupported
  $(BOOST_ROOT)
  $(SUITESPARSE_PATH)\include
  $(CERES_PATH)\include
  $(GLOG_PATH)\src\windows
  $(GLUT_ROOT)\include
  $(GLEW_ROOT)\include

  These two are only required for the GUI app:
  $(WXWIN)\include\msvc  
  $(WXWIN)\include

3. Build 'MainEngine'

4. Check for the 'ConsoleApp' and 'GUIApp' projects that the following library directories are correct. Change the environmental variables and/or directories if required. This example was made for x64 so change it to x86 if that is your case. Also, this is for Release mode so if you want to compile it in Debug Mode make the required modifications.

  $(SolutionDir)\x64\Release
  $(GLUT_ROOT)\lib
  $(GLEW_ROOT)\lib
  $(BOOST_ROOT)\stage\lib
  $(SUITESPARSE_PATH)\lib64\lapack_blas_windows
  $(SUITESPARSE_PATH)\lib64
  $(OPENCV_PATH)\x64\vc12\lib
  $(CERES_PATH)\lib
  $(GLOG_PATH)\x64\Release

  This one is only required for the GUI app:
  $(WXWIN)\lib\vc_x64_lib

5. This are the list of libraries that have to be linked to the project. Here are shown the ones for the Release mode. Change the name of the libraries to your case when required:

  - Main Engine (this is generated after compiling the MainEngine project) 
      MainEngine.lib

  - OpenCV
      opencv_core300.lib
      opencv_highgui300.lib
      opencv_imgcodecs300.lib
      opencv_imgproc300.lib
  
  - SuiteSparse
      libblas.libliblapack.lib
      libamdd.lib
      libbtfd.lib
      libcamd.lib
      libccolamd.lib
      libcholmod.lib
      libcolamd.lib
      libcxsparse.lib
      libklu.lib
      libldl.lib
      libspqr.lib
      libumfpack.lib
      suitesparseconfig.lib
  
  - glog
      libglog.lib
  
  - Ceres Solver
      ceres.lib
  
  - freeGLUT
      freeglut.lib
      GLU32.lib
      OPENGL32.lib
  
  - GLEW
      glew32.lib
  
  - Windows socket (only required for 'StopWatch.h' but not really used)
      Ws2_32.lib

  - wxWidgets
      wxmsw30u_core.lib
      wxbase30u.lib
      wxmsw30u_gl.lib
      wxexpat.lib
      wxjpeg.lib
      wxpng.lib
      wxregexu.lib
      wxtiff.lib
      wxzlib.lib
      winmm.lib
      comctl32.lib
      rpcrt4.lib
      wsock32.lib
      odbc32.lib
  
  - Boost
      libboost_filesystem-vc120-mt-1_59.lib

6. Run ConsoleApp and GUIApp with a modified configuration file to your data and result directories.