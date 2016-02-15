#ifndef _CONTROL_PANEL_H
#define _CONTROL_PANEL_H

#include "gui_app/MainFrame.h"
class BasicGLPane;

class controlPanel : public wxPanel
{

public:

  controlPanel(wxWindow *parent);
  virtual ~controlPanel();

  void Update(bool fromTracker=false);
  void OnEvent(wxMouseEvent & event);
  void ScrollEvent(wxScrollEvent & event);
  void CommandEvent(wxCommandEvent & event);
  void Reset(wxCommandEvent& event);
  void Spotlight( wxCommandEvent& event);
  // void FlipNorms( wxCommandEvent& event);
  void SaveImage( wxCommandEvent& event);
  void SaveOverlay( wxCommandEvent& event);

  void SaveImageSequence( wxCommandEvent& event);
  void SaveOverlaySequence( wxCommandEvent& event);

  void ShowSequence( wxCommandEvent& event);

  void ProcessOneFrame( wxCommandEvent& event);
  void ProcessWholeSequence( wxCommandEvent& event);
  void StopProcessing( wxCommandEvent& event);

  wxButton *m_pResetButton;
  wxButton *m_pSpotlightButton;
  wxCheckBox *m_pRenderWndCheckBox;
  wxCheckBox *m_pShowGTCheckBox;

  wxCheckBox *m_pShowESTCheckBox;
  wxCheckBox *m_pShowTextureCheckBox;
  wxCheckBox *m_pShowSurfaceCheckBox;
  wxCheckBox *m_pShowEdgesCheckBox;
  wxCheckBox *m_pShowPointsCheckBox;
  wxCheckBox *m_pUseSpotlightCheckBox;
  wxCheckBox *m_pBreakBoundaryCheckBox;
  wxCheckBox *m_pDynamicThreshCheckBox;
  wxCheckBox *m_pDeletePointsCheckBox;
  wxCheckBox *m_pWhiteBackground;
  wxCheckBox *m_pHideOcclusion;
  wxCheckBox *m_pOcclusionMode;
  wxCheckBox *m_pColorDiffMode;
  wxCheckBox *m_pFlipNormsCheckBox;
  wxCheckBox *m_pRenderPropCheckBox;
  wxCheckBox *m_pShowNormalsBox;
  wxCheckBox *m_pShowErrorHeatMap;

  wxStaticText *m_pFrameNumText;
  wxSlider *m_pFrameSlider;
  wxStaticText *m_pMaxPolyScaleText;
  wxSlider *m_pMaxPolyScaleSlider;
  wxStaticText *m_pAmbientLightText;
  wxSlider *m_pAmbientLightSlider;
  wxStaticText *m_pPntSizeText;
  wxSlider *m_pPntSizeSlider;
  wxStaticText *m_pOverlaySizeText;
  wxSlider *m_pOverlaySizeSlider;
  wxStaticText *m_pVisTolRatioText;
  wxSlider *m_pVisTolRatioSlider;
  wxStaticText *m_pPyramidLevelText;
  wxSlider *m_pPyramidLevelSlider;

  int m_nRenderLevel;   // pyramid level rendering
  bool m_nRenderProp;   // render type, whether rendering propagation result

  wxButton *m_pSaveImageButton;
  wxButton *m_pSaveOverlayButton;
  wxButton *m_pSaveSequenceButton;
  wxButton *m_pSaveOverlaySequenceButton;
  wxButton *m_pShowSequenceButton;

  wxButton *m_pRunOneFrameButton;
  wxButton *m_pRunSequenceButton;
  wxButton *m_pStopButton;

  bool m_bStop;

  bool m_bRenderWndOn;
  bool m_bShowGT;
  bool m_bShowEST;
  bool m_bShowTexture;
  bool m_bShowSurface;
  bool m_bShowPoints;
  bool m_bShowEdges;
  bool m_bSpotlightOn;
  bool m_bBreakBoundaryOn;
  bool m_bDynamicThreshOn;
  bool m_bWhiteBackground;
  bool m_bHideOcclusion;
  bool m_bOcclusionMode;
  bool m_bColorDiffMode;
  bool m_bFlipNorm;
  bool m_bRenderProp;
  bool m_bShowNormals;
  bool m_bShowErrorHeatMap;

  BasicGLPane* m_pGLPane; // pointer to the rendering pane
  //wxGLCanvas* m_pGLPane;
  wxPanel* m_pOverlayPane;
  wxPanel* m_pImagePane;

  bool m_bDeletePoints; // if we delete points belong to huge triangles
  bool m_bIsCentered; // if we have translated the camera and object
  int m_nStartFrame;
  int m_nCurrentFrame;
  int m_nFrameStep;

  int m_nFrames;
  float m_nMaxPolyScale;
  float m_nAmbientLight;

  double m_nToleranceRatio;

  int m_nPntSize;
  int m_nOverlaySize;

  MainFrame* m_pMainFrame;

private:

  DECLARE_EVENT_TABLE();

};

enum{
  ID_RESET = 1,
  // ID_SPOTLIGHT,
  // ID_FLIP_NORMALS,
  ID_SAVE_IMAGE,
  ID_SAVE_OVERLAY,
  ID_SAVE_SEQUENCE,
  ID_SAVE_OVERLAY_SEQUENCE,
  ID_SHOW_SEQUENCE,
  ID_RUN_FRAME,
  ID_RUN_SEQUENCE,
  ID_STOP
};

#endif
