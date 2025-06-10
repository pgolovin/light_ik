#include "plot_area.h"
#include "windowsx.h"
#include <mutex>

PlotArea::PlotArea(const RECT& wnd_position, const std::string& title, const std::string& name)
    : Window(wnd_position, title, name)
{
    
};

void PlotArea::onIdle()
{
}

LRESULT CALLBACK PlotArea::WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_PAINT:
    {
        std::lock_guard<std::mutex> guard(m_guard);
        m_current_positions.clear();
    }
    break;
    case WM_DESTROY:
        PostQuitMessage(0);
        StopMainLoop();
        break;
    case WM_MOUSEMOVE:
        m_last_position.x = GET_X_LPARAM(lParam); 
        m_last_position.y = GET_Y_LPARAM(lParam); 
        {
            HDC hdc = GetDC(GetWindowHandle());
            SetPixel(hdc, m_last_position.x, m_last_position.y, 0x50000000);
            ReleaseDC(GetWindowHandle(), hdc);
        }
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }

    return 0;
}
void PlotArea::Step(int32_t x_increment, int32_t y_increment)
{
    std::lock_guard<std::mutex> guard(m_guard);

    HDC hdc = GetDC(GetWindowHandle());
    SetPixel(hdc, m_last_position.x, m_last_position.y, 0x50000000);
    ReleaseDC(GetWindowHandle(), hdc);
}
