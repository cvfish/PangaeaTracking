#include "gui_app/MainFrame.h"
#include "gui_app/controlPanel.h"
#include "gui_app/BasicGLPane.h"
#include "gui_app/ImagePanel.h"

#include "third_party/Stopwatch.h"

BEGIN_EVENT_TABLE(MainFrame, wxFrame)
    EVT_TIMER(ID_TIMER, MainFrame::OnTimer)
    EVT_IDLE(MainFrame::OnIdle)
END_EVENT_TABLE()

// MainFrame::MainFrame(const wxString& title, int argc, wxChar* argv[])
//MainFrame::MainFrame(const wxString& title, int argc, char** argv)
MainFrame::MainFrame(const wxString& title, int argc, char* argv[])
:wxFrame(NULL, -1, title, wxPoint(0,0), wxSize(1000,1000)),
    m_nTimer(this, ID_TIMER),
    isTrackingFinished(true)
{
    
    // read configuration file
    ReadConfigurationFile(argc,argv);

    // set up imagesourceEngine and tracking engine
    SetupInputAndTracker();

    wxGetApp().m_pMainFrame = this;

    // splitter the Panel to controlPanel and rightPanel
    wxSplitterWindow* splittermain = new wxSplitterWindow(this, wxID_ANY);
    splittermain->SetSashGravity(0);
    splittermain->SetMinimumPaneSize(210);
    m_pControlPanel = new controlPanel(splittermain);
    wxPanel* rightPanel = new wxPanel(splittermain);
    splittermain->SplitVertically(m_pControlPanel, rightPanel);
    splittermain->SetSashPosition(200);

    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(splittermain, 1, wxEXPAND, 0);
    this->SetSizer(sizer);
    sizer->SetSizeHints(this);

    // split the rightPanel to glPanel and imagePanel
    wxSplitterWindow* splitterRenderImage = new wxSplitterWindow(rightPanel, wxID_ANY);
    splitterRenderImage->SetSashGravity(0.5);

    wxBoxSizer* rightSizer = new wxBoxSizer(wxVERTICAL);
    rightSizer->Add(splitterRenderImage,1,wxEXPAND);
    rightPanel->SetSizer(rightSizer);

    wxPanel* glPanel = new wxPanel(splitterRenderImage);
    wxPanel* imagePanel = new wxPanel(splitterRenderImage);
    splitterRenderImage->SplitVertically(glPanel, imagePanel);

    // do something on glPanel
    int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
    m_pGLPane = new BasicGLPane(glPanel, args);
    m_pGLPane->setImageHeight(m_nHeight);
    m_pGLPane->setImageWidth(m_nWidth);
    m_pGLPane->setIntrinsicMatrix(KK);
    m_pGLPane->setProjectionMatrix();

    // update the opengl panel pointer in control panel
    m_pControlPanel->m_pGLPane = m_pGLPane;

    wxBoxSizer* glSizer = new wxBoxSizer(wxVERTICAL);
    //wxBoxSizer* glSizer = new wxBoxSizer(wxHORIZONTAL);
    glSizer->Add(m_pGLPane,1,wxEXPAND);
    glPanel->SetSizer(glSizer);

    // splitter the imagePanel vertically to inputImagePanel and overlayPanel
    wxSplitterWindow* splitterImage = new wxSplitterWindow(imagePanel, wxID_ANY);
    splitterImage->SetSashGravity(0.5);
    wxPanel* inputImagePanel = new wxPanel(splitterImage);
    wxPanel* overlayPanel = new wxPanel(splitterImage);
    splitterImage->SplitHorizontally(inputImagePanel, overlayPanel);
    // splitterImage->SplitVertically(inputImagePanel, overlayPanel);

    // do something on imagePanel
    // m_pOverlayPane = new wxPanel(inputImagePanel);
    // m_pImagePane = new wxPanel(overlayPanel);
    m_pImagePane = new wxImagePanel(inputImagePanel,m_pColorImageRGB,m_nWidth,m_nHeight, false);
    m_pOverlayPane = new wxImagePanel(overlayPanel,m_pColorImageRGB,m_nWidth,m_nHeight, true);

    m_pOverlayPane->setMainFrame(this);
    m_pImagePane->setMainFrame(this);
    m_pOverlayPane->setNumPnts(m_nWidth*m_nHeight*shapeLoadingSettings.shapeSamplingScale*
        shapeLoadingSettings.shapeSamplingScale);
    m_pImagePane->setNumPnts(m_nWidth*m_nHeight*shapeLoadingSettings.shapeSamplingScale*
        shapeLoadingSettings.shapeSamplingScale);

    // wxBoxSizer* imagePaneSizer = new wxBoxSizer(wxHORIZONTAL);
    // wxBoxSizer* OverlayPaneSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* imagePaneSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* OverlayPaneSizer = new wxBoxSizer(wxVERTICAL);
    imagePaneSizer->Add(m_pImagePane,1,wxEXPAND,0);
    OverlayPaneSizer->Add(m_pOverlayPane,1,wxEXPAND,0);
    inputImagePanel->SetSizer(imagePaneSizer);
    overlayPanel->SetSizer(OverlayPaneSizer);

    // wxBoxSizer* imageSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* imageSizer = new wxBoxSizer(wxVERTICAL);
    imageSizer->Add(splitterImage, 1, wxEXPAND);
    imagePanel->SetSizer(imageSizer);

    if(trackerSettings.showWindow)
    {
        this->SetAutoLayout(true);
        this->Show(true);
        this->Maximize(true);
    }
    else
    {
        // wxCommandEvent dummyEvent;
        // m_pControlPanel->ProcessWholeSequence(dummyEvent);
        cout << "not showing window " << endl;
        while(m_nCurrentFrame <= m_NumTrackingFrames)
        {
            //m_nCurrentFrame++;
            m_nCurrentFrame += m_nFrameStep;
            ProcessOneFrame(m_nCurrentFrame);
        }
    }
}

MainFrame::~MainFrame()
{
    if(m_nTimer.IsRunning())
    m_nTimer.Stop();

    if(threadTracking.get_id() != boost::thread::id())
    threadTracking.join();
}

bool MainFrame::ProcessOneFrame(int nFrame)
{
    // if(trackingType != DEFORMNRSFM && m_pControlPanel->m_nCurrentFrame == nFrame && m_nCurrentFrame == nFrame)
    // return true;
    isTrackingFinished =  false;
    cout << "processing frame: " << nFrame << endl;
    
    // // read input
    // TICK("getInput");
    // if(!GetInput(nFrame))
    // return false;
    // TOCK("getInput");

    // // do tracking
    // TICK("tracking");
    // if(!m_pTrackingEngine->trackFrame(nFrame, m_pColorImageRGB, &pOutputInfo))
    // {
    //     cout << "tracking failed: " << endl;
    //     return false;
    // }
    // TOCK("tracking");
    if(!MainEngine::ProcessOneFrame(nFrame))
    return false;

    // update imagePanel
    TICK("update2DRendering");
    m_pOverlayPane->updateImage(m_pColorImageRGB, m_nWidth, m_nHeight);
    m_pImagePane->updateImage(m_pColorImageRGB, m_nWidth, m_nHeight);
    TOCK("update2DRendering");
    
    isTrackingFinished =  true;

    Stopwatch::getInstance().printAll();
    
    return true;
}

void MainFrame::OnTimer(wxTimerEvent& event)
{
    if(m_nCurrentFrame + m_nFrameStep <= m_NumTrackingFrames)
    threadTracking = boost::thread(&MainFrame::ProcessNextFrame, this);
    else
    m_nTimer.Stop();
}

void MainFrame::OnIdle(wxIdleEvent& event)
{
    // do nothing if we are loading sequencea
    if(trackingType == DEFORMNRSFM || trackingType == MESHPYRAMID)
    {
        m_pControlPanel->m_nCurrentFrame = m_nCurrentFrame;
        m_pControlPanel->Update(true);
        //    cout << "frame number  " << m_pControlPanel->m_nCurrentFrame << endl;
    }
    m_pGLPane->Refresh();
}

void MainFrame::UpdateVisibilityMask(double toleranceRatio)
{
    vector<bool> visibilityMask;
    UpdateVisibilityMaskGL(outputInfo, visibilityMask, KK, outputInfo.camPose,
        m_nWidth, m_nHeight, toleranceRatio);

}

void MainFrame::UpdateRenderingLevel(int nRenderLevel, bool renderType)
{
    // update the rendering level
    if(trackingType == DEFORMNRSFM || trackingType == MESHPYRAMID)
    {
        m_pTrackingEngine->updateRenderingLevel(&pOutputInfo, nRenderLevel, renderType);
    }
}

void MainFrame::SaveImage()
{
    cout<<"save image"<<endl;
    static unsigned int image_index=0;
    GLint l_ViewPort[4];
    glGetIntegerv(GL_VIEWPORT, l_ViewPort);

    for (int i = 0; i != 4; ++i)
    l_ViewPort[i]-=l_ViewPort[i]%4;

    int width=  (l_ViewPort[2] - l_ViewPort[0]);
    int height=(l_ViewPort[3] - l_ViewPort[1]);
    cout<<"l_Viewport[0] is "<<l_ViewPort[0]<<endl;
    cout<<"l_Viewport[1] is "<<l_ViewPort[1]<<endl;
    cout<<"l_Viewport[2] is "<<l_ViewPort[2]<<endl;
    cout<<"l_Viewport[3] is "<<l_ViewPort[3]<<endl;

    glReadBuffer(GL_BACK);
    glFinish();
    glPixelStorei(GL_PACK_ALIGNMENT, 3);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    unsigned char *glBitmapData = new unsigned char [3 * width * height];
    unsigned char *glBitmapData_flip = new unsigned char [3 * width * height];
    //  glReadPixels((GLint)0, (GLint)0, (GLint)width, (GLint)height, GL_RGB, GL_BYTE, glBitmapData);
    glReadPixels((GLint)0, (GLint)0, (GLint)width, (GLint)height, GL_RGB, GL_UNSIGNED_BYTE, glBitmapData);
    for(int i=0;i<height;++i)
    memcpy(&glBitmapData_flip[3*i*width],&glBitmapData[3*(height-i-1)*width],3*width);
    //wxImage img (width, height, glBitmapData, true);
    wxImage img (width, height, glBitmapData_flip, true);
    cout<<"width is "<<width<<endl;
    cout<<"height is "<<height<<endl;
    wxInitAllImageHandlers();
    
    //glReadPixels(l_ViewPort[0], l_ViewPort[1], l_ViewPort[2], l_ViewPort[3],
    //             GL_RGB,GL_UNSIGNED_BYTE,(GLvoid*)&(pixels[0]));
    //wxImage img(width, height, &(pixels[0]),true);
    //img.Mirror(false);
    cout<<"outputed data"<<endl;
    stringstream file;
    file<<setfill('0');
    // file<<path;
    // file<<savefolder;
    file<<"./";
    file<<"Rendering";
    file<<"/";

    if(!bfs::exists(file.str().c_str()))
    {
        bfs::create_directory(file.str().c_str());
    }
    file<<"render"<<setw(4)<< m_nCurrentFrame <<".png";
    //file<<"out"<<setw(3)<<image_index++<<".bmp";
    cout<<"saving as "<<file.str()<<endl;

    wxString mystring(file.str().c_str(),wxConvUTF8);
    cout<<"made mystring"<<endl;
    img.SaveFile(mystring,wxBITMAP_TYPE_PNG);
    //img.SaveFile(mystring,wxBITMAP_TYPE_BMP);
    cout<<"Saved"<<endl;
    // // Second method:

    delete [] glBitmapData;
    delete [] glBitmapData_flip;
}

void MainFrame::SaveOverlay()
{
    // save overlay image
    // create a bitmap(the same size as image)
    int width =  m_nWidth;
    int height = m_nHeight;
    wxBitmap *overlay = new wxBitmap(width,height);

    // Create a memory Device Context
    wxMemoryDC memDC;
    //Tell memDC to write on.
    memDC.SelectObject( *overlay );
    m_pOverlayPane->render(memDC);

    memDC.SelectObject( wxNullBitmap );

    stringstream file;
    file<<setfill('0');
    // file<<path;
    // file<<savefolder;
    file<<"./";
    file<<"Overlay";
    file<<"/";

    if(!bfs::exists(file.str().c_str()))
    {
        bfs::create_directory(file.str().c_str());
    }
    file<<"overlay"<<setw(4)<< m_nCurrentFrame <<".png";
    cout<<"saving as "<<file.str()<<endl;

    wxString mystring(file.str().c_str(),wxConvUTF8);
    overlay->SaveFile(mystring, wxBITMAP_TYPE_PNG, (wxPalette*)NULL );

    delete overlay;
}

