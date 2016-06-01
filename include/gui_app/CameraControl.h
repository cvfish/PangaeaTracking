#ifndef _CAMERACONTROL_H
#define _CAMERACONTROL_H

#include "main_engine/utils/global.h"
#include "gui_app/Matrices.h"

//include <gl\glut.h>           // Need to include it here because the GL* types are required
const float PI=3.1415926535897932384626433832795;
const float PIdiv180=(PI/180.0);

class CCameraControl
{
public:

  float camAngleX;
  float camAngleY;
  float camAngleZ;
  float camTransX;
  float camTransY;
  float camTransZ;

  Matrix4 matrixView;     // for camera
  Matrix4 matrixModel;    // for object
  Matrix4 matrixModelView;

  Matrix4 matrixModelRotation; // rotation

  Vector3 center;
  Vector4 cameraLoc;

public:

  CCameraControl();       //inits the values (Position: (0|0|0) Target: (0|0|-1) )
  void Reset();    // resets values to init
  void setObjectCenter(double objectCenter[3]);
  // anyway, we need to computed the reprojections of 3d points
  // void setTrackingPose(CoordinateType trackingRot[9],
  //     trackingTrans[3]);
  const float* getModelViewMatrix();


};

#endif
