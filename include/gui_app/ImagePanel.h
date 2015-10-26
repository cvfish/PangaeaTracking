#ifndef _WX_IMAGE_PANEL
#define _WX_IMAGE_PANEL

#include <wx/wx.h>
#include <wx/sizer.h>

class MainFrame;

class wxImagePanel : public wxPanel
{
    wxImage image;
    wxBitmap resized;
    int w, h;
    bool doOverlay;
    std::vector<double> overlayPnts;  // x,y coordinate

    MainFrame* pMainFrame;
    int numOverlayPnts;

    bool updated;

public:
    wxImagePanel(wxFrame* parent, wxString file, wxBitmapType format);
    wxImagePanel(wxPanel* parent, unsigned char* pData, int width, int height, bool overlay);
    void updateImage(unsigned char* pData, int width, int height);

    void setMainFrame(MainFrame* pMainFrm);
    void setNumPnts(int numPnts);

    void paintEvent(wxPaintEvent & evt);
    void paintNow();
    void OnSize(wxSizeEvent& event);
    void render(wxDC& dc);

    DECLARE_EVENT_TABLE()

};

#endif