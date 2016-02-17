#ifndef _MAINFRAME_H
#define _MAINFRAME_H

#include "main_engine/MainEngine.h"
#include "gui_app/PangaeaTracking.h"

#include <wx/spinctrl.h>
#include <wx/textctrl.h>
#include <wx/glcanvas.h>
#include <wx/splitter.h>

class controlPanel;
class BasicGLPane;
class wxImagePanel;

class MainFrame: public wxFrame, public MainEngine
{

public:

    // MainFrame(const wxString& title, int argc, wxChar* argv[]);
    //MainFrame(const wxString& title, int argc, char** argv);
    MainFrame(const wxString& title, int argc, char* argv[]);


    ~MainFrame();

    void OnTimer(wxTimerEvent& event);
    void OnIdle(wxIdleEvent& event);

    bool ProcessOneFrame(int nFrame);

    void UpdateVisibilityMask(double toleranceRatio);
    void UpdateRenderingLevel(int nRenderLevel, bool renderType = false);

    void SaveImage();
    void SaveOverlay();

    // void ReadModelInfo();

    controlPanel* m_pControlPanel;
    BasicGLPane* m_pGLPane;

    wxImagePanel* m_pOverlayPane;
    wxImagePanel* m_pImagePane;

    // Image sequence, timer callback
    wxTimer m_nTimer;

    boost::thread threadTracking;
    bool isTrackingFinished;


private:

    DECLARE_EVENT_TABLE()

};

enum{
    ID_TIMER = 100
};

#endif
