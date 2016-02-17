#include "gui_app/PangaeaTracking.h"
#include "gui_app/MainFrame.h"

IMPLEMENT_APP(PangaeaTracking)

bool PangaeaTracking::OnInit()
{
    // m_pMainFrame = new MainFrame(wxT("PangaeaTracking Render"), argc, argv);
    // new MainFrame(wxT("PangaeaTracking Render"), argc, argv);
    // argv_char = new char*[argc];
    arg_num = argc;
    SafeAllocArrayType(argv_char, argc, char*);
    for(int i = 0; i < argc; ++i)
    {
        argv_char[i] = new char[1024];
        SafeAllocArrayType(argv_char[i], 1024, char);
        strncpy( argv_char[i],
            (const char*)( (wxString( argv[i] ) ).mb_str(wxConvUTF8) ),
            1023 );
    }

    new MainFrame("PangaeaTracking Render", argc, argv_char);

    return true;
}

int PangaeaTracking::OnExit()
{
    for(int i = 0; i < arg_num; ++i)
    SafeDeleteArray(argv_char[i]);

    SafeDeleteArray(argv_char);

    return 0;
}
