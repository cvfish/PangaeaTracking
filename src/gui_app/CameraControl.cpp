#include "math.h"
#include <iostream>

#include "gui_app/CameraControl.h"

CCameraControl::CCameraControl()
{
  rotX = 0;
  rotY = 0;

  Reset();
}

void CCameraControl::Reset()
{

  camAngleX = 0;
  camAngleY = 0;
  camAngleZ = 0;
  camTransX = 0;
  camTransY = 0;
  camTransZ = 0;

  matrixView.identity();
  matrixModel.identity();
  matrixModelRotation.identity();

  cameraLoc.x = 0;
  cameraLoc.y = 0;
  cameraLoc.z = 0;

}

void CCameraControl::setObjectCenter(double objectCenter[3])
{
  center[0] = objectCenter[0];
  center[1] = objectCenter[1];
  center[2] = objectCenter[2];
}

const float* CCameraControl::getModelViewMatrix()
{
  // transformation in normal order, different from
  // opengl operations, when there is no rotation,
  // we will transform P(x,y,z) to P(x,-y,-z)
  // after modelview matrix transformation
  matrixView.identity();

  matrixView.translate(camTransX+center[0],
                       -camTransY+center[1],
                       -camTransZ+center[2]);

  matrixView.rotateX(180);

  matrixModel.identity();
  matrixModel.translate(-center);

  matrixModel.rotateX(rotX);
  matrixModel.rotateY(rotY);

  // matrixModel.rotateZ(camAngleZ);
  // matrixModel.rotateX(camAngleX);
  // matrixModel.rotateY(camAngleY);
  // matrixModelView = matrixView*matrixModel;

  matrixModelView = matrixView * matrixModelRotation * matrixModel;

  Matrix4 matrixModelViewInv;
  matrixModelViewInv = matrixModelView;
  matrixModelViewInv.invertAffine();
  Vector4 temp(0,0,0,1);
  cameraLoc = matrixModelViewInv * temp;

  return matrixModelView.getTranspose();

}
