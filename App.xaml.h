#pragma once

#include "App.xaml.g.h"

namespace winrt::icp_winui3::implementation
{
    struct App : AppT<App>
    {
        App();

        void OnLaunched(Microsoft::UI::Xaml::LaunchActivatedEventArgs const&);
        void CenterWindow(Microsoft::UI::Xaml::Window const& window);

    private:
        winrt::Microsoft::UI::Xaml::Window window{ nullptr };
    };
}
