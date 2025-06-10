// material editor and compiler. creates mtl file that can be used to add custom settings for printer
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

#include "plot_area.h"

int main(int argc, char** argv)
{
    RECT plot = { 0, 0, 800, 800 };
    PlotArea window_area(plot, "Plot area", "PLOT");

    window_area.Show();

    while (window_area.IsRunning())
    {
        window_area.ProcessMessage();
    }

    return 0;
}
