//#include "main_engine/utils/global.h"

#include "gui_app/BasicGLPane.h"
#include "gui_app/controlPanel.h"

// #include "gui_app/PangaeaTracking.h"

DECLARE_APP(PangaeaTracking)

BasicGLPane::BasicGLPane(wxWindow* parent, int* args):
wxGLCanvas(parent, wxID_ANY, args, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE)
{
  m_pMainFrame = wxGetApp().m_pMainFrame;
  m_pControlPanel = m_pMainFrame->m_pControlPanel;

  m_pGLContext = new wxGLContext(this);
  SetBackgroundStyle(wxBG_STYLE_CUSTOM);

  m_pCameraControl = new CCameraControl;
  m_pCameraControl->setObjectCenter(m_pMainFrame->center);
}

BasicGLPane::~BasicGLPane(void)
{
  delete m_pCameraControl;
  delete m_pGLContext;
}

void BasicGLPane::OnPaint(wxPaintEvent &event)
{
  wxPaintDC dc(this);
  render(event); //This is your own render stuff
  // Refresh(false);
}

void BasicGLPane::mouseWheelMoved(wxMouseEvent& event)
{
  float delta = event.GetWheelRotation()*0.1;
  m_pCameraControl->camTransZ -= delta;
  Refresh(false);
}

void BasicGLPane::mouseMoved(wxMouseEvent& event)
{

  float deltax = -(event.GetX() - m_nX);
  float deltay = (event.GetY() - m_nY);

  if(event.LeftIsDown())
    {
      if(!m_bLeftDragging)
        {
          m_bLeftDragging = true;
        }
      else
        {
          m_pCameraControl->camAngleX += deltay*0.5;
          m_pCameraControl->camAngleY += deltax*0.5;
          Refresh(false);
        }
    }
  else if(event.MiddleIsDown())
    {
      if(!m_bMidDragging)
        {
          m_bMidDragging = true;
        }
      else
        {
          m_pCameraControl->camAngleZ -= deltax*0.5;
          Refresh(false);
        }
    }
  else if(event.RightIsDown())
    {
      if(!m_bRightDragging)
        {
          m_bRightDragging = true;
        }
      else
        {
          m_pCameraControl->camTransX += deltax;
          m_pCameraControl->camTransY -= deltay;
          Refresh(false);
        }
    }

  m_nX = event.GetX();
  m_nY = event.GetY();

}

void BasicGLPane::resized(wxSizeEvent& evt)
{
  Refresh();
}

int BasicGLPane::getWidth()
{
  return GetSize().x;
}

int BasicGLPane::getHeight()
{
  return GetSize().y;
}

void BasicGLPane::setImageWidth(int width)
{
  m_nImageWidth = width;
}

void BasicGLPane::setImageHeight(int height)
{
  m_nImageHeight = height;
}

void BasicGLPane::setIntrinsicMatrix(double K[3][3])
{
  for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
        {
          KK[i][j] = K[i][j];
        }
    }

}

void BasicGLPane::setProjectionMatrix(double zNear, double zFar)
{
  // check http://www.songho.ca/opengl/gl_projectionmatrix.html
  // about how to convert camera calibration matrix to OpenGL
  // projection matrix
  double au = KK[0][0];
  double u0 = KK[0][2];
  double av = KK[1][1];
  double v0 = KK[1][2];

  double W = m_nImageWidth;
  double H = m_nImageHeight;

  if(u0 != 0)  // perspective camera
    {
      // save data column major order
      PMatrix[0][0] = 2 * au / W;
      PMatrix[1][1] = 2 * av / H;

      PMatrix[2][0] = (1.0 - (2 * u0 / W) );
      PMatrix[2][1] = -(1.0 - (2 * v0 / H) );

      PMatrix[2][2] = (zNear + zFar) / (zNear - zFar);
      PMatrix[3][2] = (2 * zNear * zFar) / (zNear - zFar);

      PMatrix[2][3] = -1.0;
    }
  else
    {
      PMatrix[0][0] = 2/W;
      PMatrix[1][1] = 2/H;

      PMatrix[3][0] = -1;
      PMatrix[3][1] = 1;

      zNear = -1000;

      PMatrix[2][2] = 2 / (zNear - zFar);
      PMatrix[3][2] = (zNear + zFar) / (zNear - zFar);

      PMatrix[3][3] = 1;
    }


}

void BasicGLPane::render(wxPaintEvent& evt)
{
  wxGLCanvas::SetCurrent(*m_pGLContext);
  wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // viewport and projection matrix
  if(m_pControlPanel->m_bWhiteBackground)
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Black Background
  else
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
  glClearDepth(1.0f); // Depth Buffer Setup
  glEnable(GL_DEPTH_TEST); // Enables Depth Testing
  glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_COLOR_MATERIAL);

  if(!m_pControlPanel->m_pRenderWndCheckBox->GetValue())
    glViewport(0, 0, getWidth(), getHeight());
  else
    glViewport(0,0, m_nImageWidth, m_nImageHeight);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glLoadMatrixf(&PMatrix[0][0]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glLoadMatrixf(m_pCameraControl->getModelViewMatrix());


  if(m_pControlPanel->m_bTurnOffLighting)
    glDisable(GL_LIGHTING);
  else
    initLighting(m_pControlPanel->m_bSpotlightOn,
                 m_pControlPanel->m_nAmbientLight/500.0,
                 m_pCameraControl->cameraLoc.x,
                 m_pCameraControl->cameraLoc.y,
                 m_pCameraControl->cameraLoc.z);

  // if(m_pControlPanel->m_bShowSurface)
  // pangaeaMeshRender(m_pMainFrame->outputInfo,
  //     m_pControlPanel->m_bShowTexture?1:0,
  //     m_pControlPanel->m_bShowGT,
  //     m_pControlPanel->m_bShowEST,
  //     m_pControlPanel->m_bDynamicThreshOn,
  //     m_pControlPanel->m_nMaxPolyScale);

  // if(m_pControlPanel->m_bShowPoints)
  // pangaeaPointRender(m_pMainFrame->outputInfo,
  //     m_pControlPanel->m_bShowTexture?1:0,
  //     m_pControlPanel->m_bShowGT,
  //     m_pControlPanel->m_bShowEST,
  //     m_pControlPanel->m_bDeletePoints,
  //     m_pControlPanel->m_nMaxPolyScale);

  //update mesh level
  if(m_pControlPanel->m_nRenderLevel != (*m_pMainFrame->pOutputInfo).nRenderLevel)
    {
      m_pControlPanel->m_nRenderLevel = (*m_pMainFrame->pOutputInfo).nRenderLevel;
      m_pControlPanel->m_pPyramidLevelSlider->SetValue(m_pControlPanel->m_nRenderLevel);
      m_pControlPanel->m_pPyramidLevelText->SetLabel(
                                                     wxString::Format(wxT("Pyramid Level %d"), m_pControlPanel->m_nRenderLevel));
    }

  if(m_pControlPanel->m_bShowSurface)
    pangaeaMeshRender(*m_pMainFrame->pOutputInfo,
                      m_pControlPanel->m_bShowTexture?1:0,
                      m_pControlPanel->m_bShowGT,
                      m_pControlPanel->m_bShowEST,
                      m_pControlPanel->m_bDynamicThreshOn,
                      m_pControlPanel->m_nMaxPolyScale);

  if(m_pControlPanel->m_bShowPoints)
    pangaeaPointRender(*m_pMainFrame->pOutputInfo,
                       m_pControlPanel->m_bShowTexture?1:0,
                       m_pControlPanel->m_bShowGT,
                       m_pControlPanel->m_bShowEST,
                       m_pControlPanel->m_bDeletePoints,
                       m_pControlPanel->m_nMaxPolyScale);

  glPopMatrix();
  glFlush();
  SwapBuffers();
}

void BasicGLPane::initLighting(bool spot,GLfloat ambient,GLfloat spot1,GLfloat spot2,GLfloat spot3)
{
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  GLfloat zeros[]={0,0,0,0};
  GLfloat light_ambient[] = {ambient, ambient, ambient, 1.0};

  GLfloat orientation[] = {spot1,spot2,spot3,1};
  GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mat_shininess[] = { 50.0 };
  //  GLfloat intense[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat intense[] = { 0.5, 0.5, 0.5, 0.5 };
  glShadeModel(GL_SMOOTH);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  if(spot)
    glLightfv(GL_LIGHT0, GL_DIFFUSE,intense);
  else
    glLightfv(GL_LIGHT0, GL_DIFFUSE,zeros);

  glLightfv(GL_LIGHT0, GL_POSITION, orientation);
  glLightfv(GL_LIGHT0, GL_SPECULAR,zeros);

  glLightfv(GL_LIGHT0, GL_POSITION, orientation);
  glLightfv(GL_LIGHT0, GL_SPECULAR,zeros);
  glEnable(GL_COLOR_MATERIAL);

  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );

}

void BasicGLPane::pangaeaMeshRender(TrackerOutputInfo& outputInfo, bool showTexture, bool
                                    showGT, bool showEST, bool threshOn, GLfloat thresh)
{
  if(showGT)
    {
      if(m_pControlPanel->m_bFeatDiffMode)
        meshRender(outputInfo.meshDataFeatDiffGT, showTexture, true, threshOn, thresh);
      else{
        if(m_pControlPanel->m_bColorDiffMode)
          meshRender(outputInfo.meshDataColorDiffGT, showTexture, true, threshOn, thresh);
        else
          meshRender(outputInfo.meshDataGT, showTexture, true, threshOn, thresh);
      }
    }

  if(showEST)
    {
      if(m_pControlPanel->m_bFeatDiffMode)
        meshRender(outputInfo.meshDataFeatDiff, showTexture, false, threshOn, thresh);
      else{
        if(m_pControlPanel->m_bColorDiffMode)
          meshRender(outputInfo.meshDataColorDiff, showTexture, false, threshOn, thresh);
        else
          meshRender(outputInfo.meshData, showTexture, false, threshOn, thresh);
      }
    }
}

void BasicGLPane::pangaeaPointRender(TrackerOutputInfo& outputInfo, bool showTexture, bool
                                     showGT, bool showEST, bool deletePoints, GLfloat thresh)
{
  if(showGT)
    {
      if(m_pControlPanel->m_bFeatDiffMode)
        pointRender(outputInfo.meshDataFeatDiffGT, showTexture, true, deletePoints, thresh);
      else{
        if(m_pControlPanel->m_bColorDiffMode)
          pointRender(outputInfo.meshDataColorDiffGT, showTexture, true, deletePoints, thresh);
        else
          pointRender(outputInfo.meshDataGT, showTexture, true, deletePoints, thresh);
      }
    }

  if(showEST)
    {
      if(m_pControlPanel->m_bFeatDiffMode)
        pointRender(outputInfo.meshDataFeatDiff, showTexture, false, deletePoints, thresh);
      else{
        if(m_pControlPanel->m_bColorDiffMode)
          pointRender(outputInfo.meshDataColorDiff, showTexture, false, deletePoints, thresh);
        else
          pointRender(outputInfo.meshData, showTexture, false, deletePoints, thresh);
      }
    }
}

void BasicGLPane::meshRender(PangaeaMeshData& mesh, bool showTexture, bool isGT, bool threshOn, GLfloat thresh)
{
  glBegin(GL_TRIANGLES);
  for(int i = 0; i < mesh.numFaces; ++i)
    {
      if(threshOn && !checksize( &mesh.vertices[ mesh.facesVerticesInd[i][0] ][0],
                                 &mesh.vertices[ mesh.facesVerticesInd[i][1] ][0],
                                 &mesh.vertices[ mesh.facesVerticesInd[i][2] ][0],
                                 exp(thresh)))
        continue;
      renderHelper(mesh, i, showTexture, isGT);
    }
  glEnd();

  // if(m_pControlPanel->m_bShowEdges)
  // {
  //      for(int i = 0; i < mesh.numFaces; ++i)
  //      {
  //          glBegin(GL_LINE_LOOP);
  //          // renderHelper(mesh, i, showTexture, isGT);
  //          renderHelper(mesh, i, 0, isGT);
  //          glEnd();
  //      }
  // }

}

void BasicGLPane::pointRender(PangaeaMeshData& mesh, bool showTexture, bool isGT, bool deletePoints, GLfloat thresh)
{
  glPointSize(m_pControlPanel->m_nPntSize);

  for(int i = 0; i < mesh.numFaces; ++i)
    {
      if(m_pControlPanel->m_bShowEdges)
        glBegin(GL_LINE_LOOP);
      else
        glBegin(GL_POINTS);

      if(deletePoints && !checksize( &mesh.vertices[ mesh.facesVerticesInd[i][0] ][0],
                                     &mesh.vertices[ mesh.facesVerticesInd[i][1] ][0],
                                     &mesh.vertices[ mesh.facesVerticesInd[i][2] ][0],
                                     exp(thresh)))
        continue;

      renderHelper(mesh, i, showTexture, isGT);
      glEnd();
    }

}

void BasicGLPane::renderHelper(PangaeaMeshData& mesh, int faceID, bool showTexture, bool isGT)
{

  vector<bool>& visibilityMask = (*m_pMainFrame->pOutputInfo).visibilityMask;
  if( m_pControlPanel->m_bHideOcclusion &&
      (  !visibilityMask[ mesh.facesVerticesInd[faceID][0] ] ||
         !visibilityMask[ mesh.facesVerticesInd[faceID][1] ] ||
         !visibilityMask[ mesh.facesVerticesInd[faceID][2] ]
         )
      )
    return;

  for(int k = 0; k < 3; ++k)
    {
      int offset = mesh.facesVerticesInd[faceID][k];

      if(m_pControlPanel->m_bFlipNorm)
        glNormal3f( -mesh.normals[offset][0],
                    -mesh.normals[offset][1],
                    -mesh.normals[offset][2]);
      else
        glNormal3f( mesh.normals[offset][0],
                    mesh.normals[offset][1],
                    mesh.normals[offset][2]);

      if(m_pControlPanel->m_bOcclusionMode)
        {
          // if(m_pMainFrame->outputInfo.visibilityMask[offset]) //visiblle
          if((*m_pMainFrame->pOutputInfo).visibilityMask[offset]) //visiblle
            glColor3f(1.0, 0.0, 0.0);
          else
            glColor3f(0.0, 0.0, 1.0);
        }
      else
        {
          if(showTexture)
            glColor3f( mesh.colors[offset][0],
                       mesh.colors[offset][1],
                       mesh.colors[offset][2]);
          else
            {
              if(isGT)
                glColor3f( mesh.modelColors[ mesh.modelLabels[offset] ][1],
                           mesh.modelColors[ mesh.modelLabels[offset] ][0],
                           mesh.modelColors[ mesh.modelLabels[offset] ][2]);
              else
                glColor3f( mesh.modelColors[ mesh.modelLabels[offset] ][0],
                           mesh.modelColors[ mesh.modelLabels[offset] ][1],
                           mesh.modelColors[ mesh.modelLabels[offset] ][2]);

              if(m_pControlPanel->m_bShowNormals)
                {
                  //glColor3f(sqrt(mesh.normals[offset][0] / 2.0 + 0.5),
                  //	sqrt(mesh.normals[offset][1] / 2.0 + 0.5),
                  //	sqrt(mesh.normals[offset][2] / 2.0 + 0.5));
                  glColor3f(
                            (mesh.normals[offset][0] + 1.0f) / 2.0,
                            (mesh.normals[offset][1] + 1.0f) / 2.0,
                            (mesh.normals[offset][2] + 1.0f) / 2.0
                            );
                }

              if(!isGT && m_pControlPanel->m_bShowErrorHeatMap && mesh.diffWithGT.size() > 0)
                {
                  glColor3f(
                            mesh.diffWithGT[offset][0],
                            mesh.diffWithGT[offset][1],
                            mesh.diffWithGT[offset][2]
                            );
                }

              // // show the gray edges
              // glColor3f( 0, 0, 0);

            }

        }

      glVertex3f( mesh.vertices[offset][0],
                  mesh.vertices[offset][1],
                  mesh.vertices[offset][2]);
    }

}

BEGIN_EVENT_TABLE(BasicGLPane, wxGLCanvas)
EVT_MOTION(BasicGLPane::mouseMoved)
EVT_SIZE(BasicGLPane::resized)
EVT_MOUSEWHEEL(BasicGLPane::mouseWheelMoved)
EVT_PAINT(BasicGLPane::render)
END_EVENT_TABLE()
