#ifndef _BASIC_GLPANE_
#define _BASIC_GLPANE_

#include "gui_app/CameraControl.h"
#include "gui_app/MainFrame.h"

class controlPanel;

class BasicGLPane : public wxGLCanvas
{
public:

  BasicGLPane(wxWindow* parent, int* args);
  ~BasicGLPane();

  void OnPaint(wxPaintEvent &event);
  void resized(wxSizeEvent& evt);

  int getWidth();
  int getHeight();
  void setImageHeight(int height);
  void setImageWidth(int width);
  void setIntrinsicMatrix(double K[3][3]);
  void setProjectionMatrix(double zNear=0.1, double zFar=20000);

  void render(wxPaintEvent& evt);

  // events
  void mouseMoved(wxMouseEvent& evt);
  void mouseWheelMoved(wxMouseEvent& evt);

  void initLighting(bool spot,GLfloat ambient,GLfloat spot1,GLfloat spot2,GLfloat spot3);

  void pangaeaMeshRender(TrackerOutputInfo& outputInfo, bool showTexture, bool
                         showGT, bool showEST, bool threshOn, GLfloat thresh);
  void meshRender(PangaeaMeshData& mesh, bool showTexture, bool isGT, bool threshOn, GLfloat thresh);

  void pangaeaPointRender(TrackerOutputInfo& outputInfo, bool showTexture, bool
                          showGT, bool showEST, bool threshOn, GLfloat thresh);
  void pointRender(PangaeaMeshData& mesh, bool showTexture, bool isGT, bool threshOn, GLfloat thresh);

  void renderHelper(PangaeaMeshData& mesh, int faceID, bool showTexture, bool isGT);
  void renderPoint(PangaeaMeshData& mesh, int pointID, bool showTexture, bool isGT);

  wxGLContext* m_pGLContext;
  CCameraControl* m_pCameraControl;
  double KK[3][3]; //camera intrinsics
  float PMatrix[4][4]; //projection matrix for opengl, column major order

  //    controlPanel* m_pControlPanel;
  controlPanel* m_pControlPanel;
  MainFrame* m_pMainFrame;

  float m_nX;
  float m_nY;

  bool m_bLeftDragging;
  bool m_bRightDragging;
  bool m_bMidDragging;

  bool m_bIsOrthoCamera;
  bool m_IsColMajor;

  int m_nImageHeight;
  int m_nImageWidth;

private:

  DECLARE_EVENT_TABLE();

};

#endif
