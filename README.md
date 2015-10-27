#PangaeaTracking

This is the implementation of the following ICCV2015 paper:

Direct, Dense, and Deformable: Template-Based Non-Rigid 3D Reconstruction from RGB Video  
Rui Yu, Chris Russell, Neill D. F. Campbell, Lourdes Agapito

This github repository is maintained by Rui Yu (R.Yu@cs.ucl.ac.uk)  
Contact me if you have any questions.

#1. Building the System

###1.1 Requirements

PangaeaTracking has been tested in Ubuntu 14.04 only. Several 3rd party libraries are needed for compiling PangaeaTracking.

  - OpenGL / GLU / GLEW / X11   
```
   sudo apt-get install libgl1-mesa-dev
   sudo libglu1-mesa-dev
   sudo apt-get install libglew1.8 libglew-dev
   apt-get install libx11-dev
```

  - OPENCV (e.g. version 2.4.8 or 3.0)   
    available at http://opencv.org/

  - Ceres Solver
    available at http://ceres-solver.org/   

  - wxWidgets
    available at https://www.wxwidgets.org/   

  - Boost   
    available at http://www.boost.org/

###1.2 Build Process

  To compile the system, do the following:

```
  ./build.sh
```

#2. Data

One example sequence is available at  

#3. Examples

After building PangaeaTracking and preparing the data, you are ready to run the scripts in examples folder.


------