#include "main_engine/MainEngine.h"

int main(int argc, char* argv[])
{
    MainEngine mainEngine;
    mainEngine.ReadConfigurationFile(argc, argv);
    mainEngine.SetupInputAndTracker();
    mainEngine.Run();

    return 0;
}