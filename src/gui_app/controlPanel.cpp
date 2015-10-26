#include "main_engine/utils/global.h"

#include "gui_app/controlPanel.h"
#include "gui_app/BasicGLPane.h"

#include "gui_app/PangaeaTracking.h"

DECLARE_APP(PangaeaTracking)

BEGIN_EVENT_TABLE(controlPanel, wxPanel)
    EVT_COMMAND_SCROLL(wxID_ANY,controlPanel::ScrollEvent)
    EVT_CHECKBOX(wxID_ANY,controlPanel::CommandEvent)
    EVT_BUTTON(ID_RESET,controlPanel::Reset)
// EVT_BUTTON(ID_SPOTLIGHT,controlPanel::Spotlight)
// EVT_BUTTON(ID_FLIP_NORMALS,controlPanel::FlipNorms)
    EVT_BUTTON(ID_SAVE_IMAGE,controlPanel::SaveImage)
    EVT_BUTTON(ID_SAVE_OVERLAY,controlPanel::SaveOverlay)
    EVT_BUTTON(ID_SAVE_SEQUENCE,controlPanel::SaveImageSequence)
    EVT_BUTTON(ID_SAVE_OVERLAY_SEQUENCE,controlPanel::SaveOverlaySequence)
    EVT_BUTTON(ID_SHOW_SEQUENCE,controlPanel::ShowSequence)
    EVT_BUTTON(ID_RUN_FRAME,controlPanel::ProcessOneFrame)
    EVT_BUTTON(ID_RUN_SEQUENCE,controlPanel::ProcessWholeSequence)
    EVT_BUTTON(ID_STOP,controlPanel::StopProcessing)
END_EVENT_TABLE()

controlPanel::controlPanel(wxWindow *parent)
:wxPanel(parent,wxID_ANY),
    m_bRenderWndOn(false),
    m_bShowGT(false),
    m_bShowEST(true),
    m_bShowTexture(true),
    m_bShowSurface(true),
    m_bShowPoints(false),
    m_bShowEdges(false),
    m_bSpotlightOn(false),
    m_bBreakBoundaryOn(true),
    m_bDynamicThreshOn(true),
    m_bIsCentered(false),
    m_bStop(true),
    m_nCurrentFrame(1),
    m_nStartFrame(1),
    m_bDeletePoints(false),
    m_bWhiteBackground(false),
    m_bHideOcclusion(false),
    m_bOcclusionMode(false),
    m_bColorDiffMode(false),
    m_bFlipNorm(false),
    m_bRenderProp(false),
    m_nRenderLevel(0),
    m_bShowNormals(false),
    m_nPntSize(1)
{

    m_pMainFrame = wxGetApp().m_pMainFrame;
    m_pGLPane = m_pMainFrame->m_pGLPane;
    m_nFrames = m_pMainFrame->m_NumTrackingFrames;
    m_nStartFrame = m_pMainFrame->m_nStartFrame;
    m_nCurrentFrame = m_pMainFrame->m_nCurrentFrame;
    m_nFrameStep = m_pMainFrame->m_nFrameStep;

    // create all the wxwidgets
    int index = 5;
    m_pResetButton = new wxButton(this, ID_RESET, _("Reset Camera"), wxPoint(5, 5));
    index+=40;
    // m_pSpotlightButton = new wxButton(this, ID_SPOTLIGHT, _("Set spotlight"), wxPoint(5, index));
    // index+=40;
    // m_pFlipNormsButton = new wxButton(this, ID_FLIP_NORMALS, _("Flip Normals"), wxPoint(5, index));
    // index+=40;
    m_pRenderWndCheckBox = new wxCheckBox(this,wxID_ANY,_("Render Window"), wxPoint(5,index));
    index+=20;
    m_pShowGTCheckBox = new wxCheckBox(this,wxID_ANY,_("Show Model 1"), wxPoint(5, index));
    index+=20;
    m_pShowESTCheckBox= new wxCheckBox(this,wxID_ANY,_("Show Model 2"), wxPoint(5, index));
    index+=20;
    m_pShowTextureCheckBox = new wxCheckBox(this,wxID_ANY,_("Show Texture"), wxPoint(5, index));
    index+=20;
    m_pShowSurfaceCheckBox = new wxCheckBox(this,wxID_ANY,_("Show Surface"), wxPoint(5, index));
    index+=20;
    m_pShowEdgesCheckBox = new wxCheckBox(this,wxID_ANY,_("Show Edges"), wxPoint(5, index));
    index+=20;
    m_pShowPointsCheckBox = new wxCheckBox(this,wxID_ANY,_("Show Points"), wxPoint(5, index));
    index+=20;
    m_pUseSpotlightCheckBox = new wxCheckBox(this,wxID_ANY,_("Use Spotlight"), wxPoint(5, index));
    index+=20;
    m_pBreakBoundaryCheckBox = new wxCheckBox(this,wxID_ANY,_("Break on Label Boundary"), wxPoint(5, index));
    index+=20;
    m_pDynamicThreshCheckBox = new wxCheckBox(this,wxID_ANY,_("Dynamic threshold"), wxPoint(5, index));
    index+=20;
    m_pDeletePointsCheckBox = new wxCheckBox(this,wxID_ANY,_("Delete Points"), wxPoint(5, index));
    index+=20;
    m_pWhiteBackground = new wxCheckBox(this,wxID_ANY,_("White Background"), wxPoint(5, index));
    index+=20;
    m_pHideOcclusion = new wxCheckBox(this,wxID_ANY,_("Hide Occluded"), wxPoint(5, index));
    index+=20;
    m_pOcclusionMode = new wxCheckBox(this,wxID_ANY,_("Show Occlusion"), wxPoint(5, index));
    index+=20;
    m_pColorDiffMode = new wxCheckBox(this,wxID_ANY,_("Show Color Diff"), wxPoint(5, index));
    index+=20;
    m_pFlipNormsCheckBox = new wxCheckBox(this,wxID_ANY,_("Flip Norms"), wxPoint(5, index));
    index+=20;
    m_pRenderPropCheckBox = new wxCheckBox(this,wxID_ANY,_("Render Propagation"), wxPoint(5, index));
    index+=20;
    m_pShowNormalsBox = new wxCheckBox(this,wxID_ANY,_("Show Normals"), wxPoint(5, index));
    index+=40;

    m_pFrameNumText = new wxStaticText(this,wxID_ANY,_("Frame 1"),wxPoint(5,index));
    index+=20;

    m_pFrameSlider = new wxSlider(this,wxID_ANY, m_nCurrentFrame, m_nStartFrame,
        m_nFrames, wxPoint(5,index-10), wxSize(200,40));

    index+=20;
    m_pMaxPolyScaleText = new wxStaticText(this,wxID_ANY,_("Max Poly Scale"),wxPoint(5,index));
    index+=20;
    m_pMaxPolyScaleSlider = new wxSlider(this, wxID_ANY, 0, -600, 500, wxPoint(5,index-10), wxSize(200,40));
    index+=20;
    m_pAmbientLightText = new wxStaticText(this,wxID_ANY,_("Ambient light"),wxPoint(5,index));
    index+=20;
    m_pAmbientLightSlider = new wxSlider(this, wxID_ANY, 500, 0, 500, wxPoint(5,index-10), wxSize(200,40));
    index+=20;
    m_pPntSizeText = new wxStaticText(this, wxID_ANY,  _("Point size 1"),wxPoint(5,index));
    index+=20;
    m_pPntSizeSlider=new wxSlider(this, wxID_ANY, 1, 1, 10, wxPoint(5,index-10), wxSize(200,40));
    index+=20;
    m_pOverlaySizeText = new wxStaticText(this, wxID_ANY,  _("Overlay size 1"),wxPoint(5,index));
    index+=20;
    m_pOverlaySizeSlider=new wxSlider(this, wxID_ANY, 1, 1, 10, wxPoint(5,index-10), wxSize(200,40));
    index+=20;

    m_pVisTolRatioText = new wxStaticText(this, wxID_ANY, _("Depth Tolerance Ratio 10/1000"), wxPoint(5, index));
    index+=20;
    m_pVisTolRatioSlider = new wxSlider(this, wxID_ANY, 10, 1, 1000, wxPoint(5,index-10), wxSize(200,40));
    index+=40;
    m_pPyramidLevelText = new wxStaticText(this, wxID_ANY, _("Pyramid Level 0"), wxPoint(5, index));
    //    m_pPyramidLevelSlider = new wxSlider(this, wxID_ANY, 0, 0, 3, wxPoint(5, index));
    //index-=10;
    m_pPyramidLevelSlider = new wxSlider(this, wxID_ANY, 0, 0, m_pMainFrame->m_nNumMeshLevels-1,
        wxPoint(120, index), wxSize(50,20));
    index+=30;

    m_nToleranceRatio = m_pVisTolRatioSlider->GetValue() / 1000.0;

    m_pSaveImageButton = new wxButton(this, ID_SAVE_IMAGE, _("Save Image"), wxPoint(5, index));
    index+=30;
    m_pSaveOverlayButton = new wxButton(this, ID_SAVE_OVERLAY, _("Save Overlay"), wxPoint(5, index));
    index+=30;
    m_pSaveSequenceButton = new wxButton(this, ID_SAVE_SEQUENCE,
        _("Save Image Sequence"), wxPoint(5, index));
    index+=30;
    m_pSaveOverlaySequenceButton = new wxButton(this, ID_SAVE_OVERLAY_SEQUENCE,
        _("Save Overlay Sequence"), wxPoint(5, index));
    index+=30;
    m_pShowSequenceButton = new wxButton(this, ID_SHOW_SEQUENCE,
        _("Show Sequence"), wxPoint(5, index));
    index+=30;

    m_pRunOneFrameButton = new wxButton(this, ID_RUN_FRAME, _("Run One Frame"), wxPoint(5, index));
    index+=30;
    m_pRunSequenceButton = new wxButton(this, ID_RUN_SEQUENCE, _("Run Whole Sequence"), wxPoint(5, index));
    index+=30;
    m_pStopButton = new wxButton(this, ID_STOP, _("Stop Processing"), wxPoint(5, index));
    index+=20;


    if(trackingType != DEFORMNRSFM)
    {
        m_pRunOneFrameButton->Enable(false);
        m_pRunSequenceButton->Enable(false);
        m_pStopButton->Enable(false);
    }
    else
    {
        m_pFrameSlider->Enable(false);
        m_pSaveSequenceButton->Enable(false);
        m_pSaveOverlaySequenceButton->Enable(false);
        m_pShowSequenceButton->Enable(false);
    }

    // set the initial value
    m_pRenderWndCheckBox->SetValue(m_bRenderWndOn);
    m_pShowGTCheckBox->SetValue(m_bShowGT);
    m_pShowESTCheckBox->SetValue(m_bShowEST);
    m_pShowTextureCheckBox->SetValue(m_bShowTexture);
    m_pShowSurfaceCheckBox ->SetValue(m_bShowSurface);
    m_pShowPointsCheckBox->SetValue(m_bShowPoints);
    m_pUseSpotlightCheckBox->SetValue(m_bSpotlightOn);
    m_pBreakBoundaryCheckBox->SetValue(m_bBreakBoundaryOn);

    Update();
}

controlPanel::~controlPanel()
{}

void controlPanel::Update(bool fromTracker)
{
    m_bRenderWndOn = m_pRenderWndCheckBox->IsChecked();
    m_bShowGT = m_pShowGTCheckBox->IsChecked();
    m_bShowEST = m_pShowESTCheckBox->IsChecked();
    m_bShowTexture = m_pShowTextureCheckBox->IsChecked();
    m_bShowSurface = m_pShowSurfaceCheckBox->IsChecked();
    m_bShowEdges = m_pShowEdgesCheckBox->IsChecked();
    m_bShowPoints = m_pShowPointsCheckBox->IsChecked();
    m_bSpotlightOn = m_pUseSpotlightCheckBox->IsChecked();
    m_bBreakBoundaryOn = m_pBreakBoundaryCheckBox->IsChecked();
    m_bDynamicThreshOn = m_pDynamicThreshCheckBox->IsChecked();
    m_bDeletePoints = m_pDeletePointsCheckBox->IsChecked();
    m_bWhiteBackground = m_pWhiteBackground->IsChecked();
    m_bHideOcclusion = m_pHideOcclusion->IsChecked();
    m_bOcclusionMode = m_pOcclusionMode->IsChecked();
    m_bColorDiffMode = m_pColorDiffMode->IsChecked();
    m_bFlipNorm = m_pFlipNormsCheckBox->IsChecked();
    m_bShowNormals = m_pShowNormalsBox->IsChecked();

    if(!fromTracker)
    m_nCurrentFrame = m_pFrameSlider->GetValue();
    else
    m_pFrameSlider->SetValue(m_nCurrentFrame);

    m_nMaxPolyScale = m_pMaxPolyScaleSlider->GetValue();
    m_nAmbientLight = m_pAmbientLightSlider->GetValue();
    m_nPntSize = m_pPntSizeSlider->GetValue();
    m_nOverlaySize = m_pOverlaySizeSlider->GetValue();
    m_pOverlaySizeText->SetLabel(wxString::Format(wxT("Overlay Size %d"), m_nOverlaySize));
    m_pPntSizeText->SetLabel(wxString::Format(wxT("Point Size %d"), m_nPntSize));
    m_pFrameSlider->SetLabel(wxString::Format(wxT("Frame %d"), m_nCurrentFrame));

    if(m_nRenderLevel != m_pPyramidLevelSlider->GetValue() ||
        m_bRenderProp != m_pRenderPropCheckBox->IsChecked() )
    {
        //if(m_bRenderProp)
        // cout << "rendering prop" << endl;
        // cout << "rendering optimization" << endl;

        m_nRenderLevel = m_pPyramidLevelSlider->GetValue();
        m_bRenderProp = m_pRenderPropCheckBox->IsChecked();

        m_pMainFrame->UpdateRenderingLevel(m_nRenderLevel, m_bRenderProp);
        m_pPyramidLevelText->SetLabel(
            wxString::Format(wxT("Pyramid Level %d"), m_nRenderLevel));

    }

    if(m_nToleranceRatio*1000 != m_pVisTolRatioSlider->GetValue())
    {
        // update the visibilityMask first based on the m_nToleranceRatio
        m_nToleranceRatio = m_pVisTolRatioSlider->GetValue() / 1000.0;
        m_pMainFrame->UpdateVisibilityMask(m_nToleranceRatio);
        m_pVisTolRatioText->SetLabel(
            wxString::Format(wxT("Depth Tolerance Ratio %d/1000"),
                int(m_nToleranceRatio*1000)));
        cout << m_nToleranceRatio << endl;
    }

    // update the points and normals if necessary
    if(trackingType != DEFORMNRSFM)
    {
        if(m_nCurrentFrame != m_pMainFrame->m_nCurrentFrame)
        {
            int prevFrame = m_pMainFrame->m_nCurrentFrame;
            m_pMainFrame->m_nCurrentFrame = m_nCurrentFrame;
            cout << "previous frame: " << prevFrame << endl;
            cout << "current frame: " << m_nCurrentFrame << endl;
            if(!m_pMainFrame->ProcessOneFrame(m_nCurrentFrame))
            {
                cout << "Processing failed!" << endl;
                m_nCurrentFrame = prevFrame;
                m_pMainFrame->GetInput(prevFrame);
                m_pMainFrame->m_pTrackingEngine->setCurrentFrame(prevFrame);
                m_pMainFrame->m_nCurrentFrame = prevFrame;
                m_pFrameSlider->SetValue(m_nCurrentFrame);
            }else
            cout << "Processing done!" << endl;
        }
    }
    
    m_pFrameNumText->SetLabel(wxString::Format(wxT("Frame %d"),m_nCurrentFrame));

}

void controlPanel::ScrollEvent(wxScrollEvent &event)
{
    Update();
    m_pGLPane->Refresh();
}

void controlPanel::CommandEvent(wxCommandEvent &event)
{
    Update();
    m_pGLPane->Refresh();
}

void controlPanel::Reset( wxCommandEvent& event)
{
    m_pGLPane->m_pCameraControl->Reset();
    wxPaintEvent event2;
    m_pGLPane->render(event2);
}

void controlPanel::SaveImage( wxCommandEvent& event)
{
    m_pMainFrame->SaveImage();
}

void controlPanel::SaveOverlay( wxCommandEvent& event)
{
    m_pMainFrame->SaveOverlay();
}

void controlPanel::SaveImageSequence( wxCommandEvent& event)
{
    for(int  i = m_nStartFrame; i <= m_nFrames; i = i + m_nFrameStep)
    {
        m_pFrameSlider->SetValue(i);
        Update(false);
        wxPaintEvent event2;
        m_pGLPane->render(event2);
        SaveImage(event);
    }
}

void controlPanel::SaveOverlaySequence( wxCommandEvent& event)
{
    for(int  i = m_nStartFrame; i <= m_nFrames; i = i + m_nFrameStep)
    {
        m_pFrameSlider->SetValue(i);
        Update(false);
        SaveOverlay(event);
    }
}

void controlPanel::ShowSequence( wxCommandEvent& event)
{
    m_pMainFrame->m_nTimer.Start(1);
}

void controlPanel::ProcessOneFrame( wxCommandEvent& event)
{
    m_pMainFrame->m_nTimer.StartOnce();
}

void controlPanel::ProcessWholeSequence( wxCommandEvent& event)
{
    m_pMainFrame->m_nTimer.Start(1);
}

void controlPanel::StopProcessing( wxCommandEvent& event)
{
    m_pMainFrame->m_nTimer.Stop();
}
