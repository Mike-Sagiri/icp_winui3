#include "pch.h"
#include "App.xaml.h"
#include "MainWindow.xaml.h"

using namespace winrt;
using namespace Microsoft::UI::Xaml;
using namespace Microsoft::UI::Windowing;
using namespace Windows::Graphics;

// To learn more about WinUI, the WinUI project structure,
// and more about our project templates, see: http://aka.ms/winui-project-info.

namespace winrt::icp_winui3::implementation
{
    /// <summary>
    /// Initializes the singleton application object.  This is the first line of authored code
    /// executed, and as such is the logical equivalent of main() or WinMain().
    /// </summary>
    App::App()
    {
        // Xaml objects should not call InitializeComponent during construction.
        // See https://github.com/microsoft/cppwinrt/tree/master/nuget#initializecomponent

#if defined _DEBUG && !defined DISABLE_XAML_GENERATED_BREAK_ON_UNHANDLED_EXCEPTION
        UnhandledException([](IInspectable const&, UnhandledExceptionEventArgs const& e)
        {
            if (IsDebuggerPresent())
            {
                auto errorMessage = e.Message();
                __debugbreak();
            }
        });
#endif
    }
	// For ICONS: 设置窗口图标
    void SetWindowIcon(HWND hwnd)
    {
        HICON hIcon = LoadIcon(
            GetModuleHandle(nullptr),
            MAKEINTRESOURCE(IDI_ICON1)
        );

        SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIcon);
        SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
    }
    void App::CenterWindow(Microsoft::UI::Xaml::Window const& window)
    {
        // 从 Window 拿 HWND
        HWND hwnd{};
        auto native = window.try_as<IWindowNative>();
        if (!native) return;

        native->get_WindowHandle(&hwnd);
        if (!hwnd) return;

		auto&& appWindow = window.AppWindow();

        if (!appWindow) return;

        // 当前窗口尺寸（DPI-aware）
        auto size = appWindow.Size();

        // 当前显示器工作区
        HMONITOR hMon = MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST);
        MONITORINFO mi{};
        mi.cbSize = sizeof(mi);
        GetMonitorInfo(hMon, &mi);

        RECT work = mi.rcWork;

        int x = work.left + (work.right - work.left - size.Width) / 2;
        int y = work.top + (work.bottom - work.top - size.Height) / 2;

        // 关键：用 AppWindow 移动
        appWindow.Move({ x, y });
    }
    /// <summary>
    /// Invoked when the application is launched.
    /// </summary>
    /// <param name="e">Details about the launch request and process.</param>
    void App::OnLaunched([[maybe_unused]] LaunchActivatedEventArgs const& e)
    {
        window = make<MainWindow>();
        window.Activate();
        HWND hwnd{};
        auto windowNative = window.try_as<IWindowNative>();
        windowNative->get_WindowHandle(&hwnd);
        SetWindowIcon(hwnd);
        // 通过 AppWindow 设置窗口大小
        if (auto appWindow = window.AppWindow())
        {
            SizeInt32 size{ 800, 600 };
            appWindow.Resize(size);
            CenterWindow(window);
        }

        // 可选：设置内容根的最小尺寸，防止过小
        if (auto content = window.Content().try_as<FrameworkElement>())
        {
            content.MinWidth(400);
            content.MinHeight(300);
        }
    }
}
