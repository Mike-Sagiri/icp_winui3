#pragma once

#include "MainWindow.g.h"

namespace winrt::icp_winui3::implementation
{
    struct Vec3 {
        double x, y, z;
    };

    using PointCloud = std::vector<Vec3>;

    inline Eigen::Vector3d toEigen(const Vec3& p) {
        return Eigen::Vector3d(p.x, p.y, p.z);
    }

    inline Vec3 fromEigen(const Eigen::Vector3d& v) {
        return Vec3{ v.x(), v.y(), v.z() };
    }
    struct MainWindow : MainWindowT<MainWindow>
    {
        MainWindow()
        {
            // Xaml objects should not call InitializeComponent during construction.
            // See https://github.com/microsoft/cppwinrt/tree/master/nuget#initializecomponent
        }

		void ActionButton_Click(winrt::Windows::Foundation::IInspectable const& sender, winrt::Microsoft::UI::Xaml::RoutedEventArgs const& e);
        void PointCanvas_Draw(
            winrt::Microsoft::Graphics::Canvas::UI::Xaml::CanvasControl const& sender,
            winrt::Microsoft::Graphics::Canvas::UI::Xaml::CanvasDrawEventArgs const& args);
    private:
		bool fileChosen = false;
        bool m_isRunning = false;
        int progress = 0;
        PointCloud m_visCloud;
        std::vector<Vec3> m_visNormals;
        std::mutex m_visMutex;
		std::wstring SelectFolderPath(HWND);
        std::wstring GetCurrentWorkingDirectory();
         // ×Ö·û´®±àÂë¸¨Öú£ºUTF-16 <-> UTF-8
        std::wstring folderPath;
        winrt::fire_and_forget ICP();
        void ReportProgress(double percent);
        void ChangeStatus(std::wstring status);

    };
}

namespace winrt::icp_winui3::factory_implementation
{
    struct MainWindow : MainWindowT<MainWindow, implementation::MainWindow>
    {
    };
}
