#ifndef _PANGAEA_TRACKING_
#define _PANGAEA_TRACKING_

#include <wx/wx.h>

// #include "gui_app/MainFrame.h"
class MainFrame;

class PangaeaTracking: public wxApp
{

public:

    MainFrame* m_pMainFrame;
    char **argv_char;
    int arg_num;

public:

    virtual bool OnInit();
    virtual int OnExit();

};

DECLARE_APP(PangaeaTracking)

#endif