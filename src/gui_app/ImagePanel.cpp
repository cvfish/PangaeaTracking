#include "gui_app/ImagePanel.h"

#include "gui_app/MainFrame.h"
#include "gui_app/controlPanel.h"

BEGIN_EVENT_TABLE(wxImagePanel, wxPanel)
    EVT_PAINT(wxImagePanel::paintEvent)
    EVT_SIZE(wxImagePanel::OnSize)
END_EVENT_TABLE()

wxImagePanel::wxImagePanel(wxFrame* parent, wxString file, wxBitmapType format):
    wxPanel(parent), updated(false)
{
    // load the file... ideally add a check to see if loading was successful
    image.LoadFile(file, format);
    w = -1;
    h = -1;
}

// Initialize from memory
wxImagePanel::wxImagePanel(wxPanel* parent, unsigned char* pData, int width, int height, bool overlay):
    wxPanel(parent), updated(false)
{
    image = wxImage(width, height, pData, true);
    w = width;
    h = height;
    doOverlay = overlay;
}

void wxImagePanel::updateImage(unsigned char* pData, int width, int height)
{
    image = wxImage(width, height, pData, true);
    updated = true;
    Refresh(false);
}

void wxImagePanel::setMainFrame(MainFrame* pMainFrm)
{
    pMainFrame = pMainFrm;
}

void wxImagePanel::setNumPnts(int numPnts)
{
    numOverlayPnts = numPnts;
}

void wxImagePanel::paintEvent(wxPaintEvent & evt)
{
    // depending on your system you may need to look at double-buffered dcs
    wxPaintDC dc(this);
    render(dc);
}


void wxImagePanel::paintNow()
{
    // depending on your system you may need to look at double-buffered dcs
    wxClientDC dc(this);
    render(dc);
}


void wxImagePanel::render(wxDC&  dc)
{
    int neww, newh;
    dc.GetSize( &neww, &newh );

    if(( neww != w || newh != h ) || updated)
    {
        resized = wxBitmap( image.Scale( neww, newh /*, wxIMAGE_QUALITY_HIGH*/ ) );
        w = neww;
        h = newh;
        dc.DrawBitmap( resized, 0, 0, false );
        updated = false;
    }
    else
    {
        dc.DrawBitmap( resized, 0, 0, false );
    }

    if(doOverlay)
    {
        // draw 2d projection points
        wxPen mypen = wxPen(wxColor(wxT("Red")), 1);
        dc.SetPen(mypen);
        int numPnts = (*(pMainFrame->pOutputInfo)).meshProj.size();
        int size = pMainFrame->m_pControlPanel->m_nOverlaySize;
        cout << "overlay size: " << size << endl;
        for(int k = 0; k < numPnts; ++k)
        {
            if((*(pMainFrame->pOutputInfo)).visibilityMask[k])
            {
                for(int i = 0; i < size; ++i)
                {
                    for(int j = 0; j < size; ++j)
                    {
                        dc.DrawPoint(((*(pMainFrame->pOutputInfo)).meshProj[k][0]+i) * w / pMainFrame->m_nWidth,
                            ((*(pMainFrame->pOutputInfo)).meshProj[k][1]+j) * h / pMainFrame->m_nHeight);
                    }
                }
            }
        }

        wxPen occluder_pen = wxPen(wxColor(wxT("Blue")), 1);
        dc.SetPen(occluder_pen);
        for(int k = 0; k < numPnts; ++k)
        {
            if(!(*(pMainFrame->pOutputInfo)).visibilityMask[k])
            {
                for(int i = 0; i < size; ++i)
                {
                    for(int j = 0; j < size; ++j)
                    {
                        dc.DrawPoint(((*(pMainFrame->pOutputInfo)).meshProj[k][0]+i) * w / pMainFrame->m_nWidth,
                            ((*(pMainFrame->pOutputInfo)).meshProj[k][1]+j) * h / pMainFrame->m_nHeight);
                    }
                }
            }
        }
        
    }
}

void wxImagePanel::OnSize(wxSizeEvent& event){
    Refresh();
    //skip the event.
    event.Skip();
}