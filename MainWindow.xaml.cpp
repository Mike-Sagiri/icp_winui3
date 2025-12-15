#include "pch.h"
#include "MainWindow.xaml.h"
#if __has_include("MainWindow.g.cpp")
#include "MainWindow.g.cpp"
#endif
#include <winrt/Windows.Storage.Pickers.h>
#include <winrt/Windows.Storage.h>
#include <winrt/Microsoft.UI.Interop.h>
#include <Microsoft.UI.Xaml.Window.h>
#include <winrt/Microsoft.UI.Dispatching.h>
#include <winrt/Windows.UI.Core.h>
#include <winrt/Windows.System.h>

using namespace winrt;
using namespace Microsoft::UI::Xaml;
using namespace Microsoft::UI::Windowing;
using namespace Windows::Graphics;
using namespace Windows::Storage;
using namespace Windows::Storage::Pickers;

namespace winrt::icp_winui3::implementation
{

    // ==================== IO：读写 asc 点云 ====================

    bool loadPointCloud(const std::wstring& filename, PointCloud& cloud) {
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            // 原先: std::wcerr
            // 改为：调用方负责传入 hWnd，这里保持静默返回 false
            return false;
        }
        cloud.clear();
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::istringstream iss(line);
            Vec3 p;
            if (!(iss >> p.x >> p.y >> p.z)) {
                continue; // 忽略格式错误行
            }
            cloud.push_back(p);
        }
        // 原先: std::wcout
        return true;
    }

    bool savePointCloud(const std::wstring& filename, const PointCloud& cloud) {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            // 原先: std::wcerr
            return false;
        }
        for (const auto& p : cloud) {
            ofs << p.x << " " << p.y << " " << p.z << "\n";
        }
        // 原先: std::wcout
        return true;
    }

    // ==================== nanoflann 适配器 ====================

    /**
     * 将 PointCloud 适配给 nanoflann：
     * - kdtree_get_point_count
     * - kdtree_get_pt
     */
    struct PointCloudAdaptor {
        const PointCloud& pts;

        PointCloudAdaptor(const PointCloud& points) : pts(points) {}

        inline size_t kdtree_get_point_count() const { return pts.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            if (dim == 0) return pts[idx].x;
            if (dim == 1) return pts[idx].y;
            return pts[idx].z;
        }
        // 可选：提供包围盒，返回 false 表示未提供
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    using KDTree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>,
        PointCloudAdaptor,
        3 /* dim */
    >;

    // ==================== 法向估计：kNN + PCA（用 kd-tree） ====================

    void estimateNormalsPCA(const PointCloud& cloud,
        std::vector<Vec3>& normals,
        int k_neighbors = 20)
    {
        const size_t N = cloud.size();
        normals.resize(N);
        if (N == 0) return;
        if (k_neighbors > (int)N) k_neighbors = (int)N;

        std::cout << "Estimating normals with KD-tree kNN, k=" << k_neighbors << " ...\n";
        

        PointCloudAdaptor adaptor(cloud);
        KDTree_t kdtree(3 /*dim*/, adaptor,
            nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        kdtree.buildIndex();
//#pragma omp parallel
        {
        std::vector<unsigned int> ret_indices(k_neighbors);
        std::vector<double> out_dists_sqr(k_neighbors);
//        #pragma omp parallel for
        for (LONG64 i = 0; i < N; ++i) {
            const Vec3& pi = cloud[i];
            double query_pt[3] = { pi.x, pi.y, pi.z };

            size_t found = kdtree.knnSearch(&query_pt[0], k_neighbors,
                &ret_indices[0], &out_dists_sqr[0]);
            if (found == 0) {
                normals[i] = Vec3{ 0, 0, 1 }; // fallback
                continue;
            }

            // 计算邻域质心
            Eigen::Vector3d centroid(0, 0, 0);
            for (size_t j = 0; j < found; ++j) {
                centroid += toEigen(cloud[ret_indices[j]]);
            }
            centroid /= (double)found;

            // 协方差矩阵
            Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
            for (size_t j = 0; j < found; ++j) {
                Eigen::Vector3d p = toEigen(cloud[ret_indices[j]]) - centroid;
                C += p * p.transpose();
            }
            C /= (double)found;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
            Eigen::Vector3d n = solver.eigenvectors().col(0); // 最小特征值对应的特征向量
            n.normalize();
            normals[i] = fromEigen(n);
        }
       }

        std::cout << "Normals estimated.\n";
    }

    // ==================== 点到面 ICP（使用 kd-tree 最近邻） ====================

    void ICPPointToPlane(
        PointCloud& source,                  // 输入：初始点云；输出：配准后点云
        const PointCloud& target,
        const std::vector<Vec3>& target_normals,
        Eigen::Matrix3d& R_total,
        Eigen::Vector3d& t_total,
        int max_iterations = 20,
        double max_correspondence_dist = 0.5,  // 最大对应距离
        double eps = 1e-6                       // 收敛阈值
    )
    {
        const size_t Ns = source.size();
        const size_t Nt = target.size();
        if (Ns == 0 || Nt == 0) {
            std::cerr << "ICP: empty source or target.\n";
            R_total = Eigen::Matrix3d::Identity();
            t_total = Eigen::Vector3d::Zero();
            return;
        }

        // target kd-tree
        PointCloudAdaptor adaptor(target);
        KDTree_t kdtree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree.buildIndex();

        R_total = Eigen::Matrix3d::Identity();
        t_total = Eigen::Vector3d::Zero();

        std::vector<Eigen::Vector3d> src_transformed(Ns);
        for (size_t i = 0; i < Ns; ++i) {
            src_transformed[i] = toEigen(source[i]);
        }

        for (int iter = 0; iter < max_iterations; ++iter) {
            Eigen::Matrix<double, 6, 6> ATA = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> ATb = Eigen::Matrix<double, 6, 1>::Zero();

            size_t num_corr = 0;
            double max_corr_sq = max_correspondence_dist * max_correspondence_dist;

//#pragma omp parallel
            {
                // 每个线程自己的副本（私有）
                Eigen::Matrix<double, 6, 6> ATA_local = Eigen::Matrix<double, 6, 6>::Zero();
                Eigen::Matrix<double, 6, 1> ATb_local = Eigen::Matrix<double, 6, 1>::Zero();
                size_t num_corr_local = 0;

//#pragma omp for
                for (int i = 0; i < (int)Ns; ++i)
                {
                    Eigen::Vector3d s = src_transformed[i];
                    double query_pt[3] = { s.x(), s.y(), s.z() };

                    unsigned int ret_index;
                    double out_dist_sq;

                    size_t found = kdtree.knnSearch(query_pt, 1, &ret_index, &out_dist_sq);
                    if (found == 0) continue;
                    if (out_dist_sq > max_corr_sq) continue;

                    Eigen::Vector3d q = toEigen(target[ret_index]);
                    Eigen::Vector3d n = toEigen(target_normals[ret_index]);
                    if (n.squaredNorm() < 1e-12) continue;
                    n.normalize();

                    Eigen::Vector3d s_cross_n = s.cross(n);
                    double r = n.dot(s - q);

                    Eigen::Matrix<double, 6, 1> Ai;
                    Ai << s_cross_n, n;
                    double bi = -r;

                    ATA_local += Ai * Ai.transpose();
                    ATb_local += Ai * bi;
                    ++num_corr_local;
                }

                // 合并阶段（极少发生）
//#pragma omp critical
                {
                    ATA += ATA_local;
                    ATb += ATb_local;
                    num_corr += num_corr_local;
                }
            }

            if (num_corr < 6) {
                std::cout << "ICP iter " << iter << ": not enough correspondences (" << num_corr << ")\n";
                break;
            }

            Eigen::Matrix<double, 6, 1> x = ATA.ldlt().solve(ATb);
            Eigen::Vector3d w = x.segment<3>(0);
            Eigen::Vector3d t_inc = x.segment<3>(3);

            double inc_norm = x.norm();
            std::cout << "ICP iter " << iter
                << ", increment norm = " << inc_norm
                << ", corr = " << num_corr << std::endl;

            if (inc_norm < eps) {
                std::cout << "ICP converged (small increment).\n";
                break;
            }

            // 旋转增量
            double theta = w.norm();
            Eigen::Matrix3d R_inc = Eigen::Matrix3d::Identity();
            if (theta > 1e-12) {
                Eigen::Vector3d axis = w / theta;
                Eigen::AngleAxisd aa(theta, axis);
                R_inc = aa.toRotationMatrix();
            }
            else {
                Eigen::Matrix3d W;
                W << 0, -w.z(), w.y(),
                    w.z(), 0, -w.x(),
                    -w.y(), w.x(), 0;
                R_inc = Eigen::Matrix3d::Identity() + W;
            }

            R_total = R_inc * R_total;
            t_total = R_inc * t_total + t_inc;

            for (size_t i = 0; i < Ns; ++i) {
                src_transformed[i] = R_inc * src_transformed[i] + t_inc;
            }
        }

        // 应用最终变换到 source
        for (size_t i = 0; i < Ns; ++i) {
            Eigen::Vector3d v = toEigen(source[i]);
            v = R_total * v + t_total;
            source[i] = fromEigen(v);
        }
    }

    // ==================== 遍历文件夹，多帧 ICP + 合并 ====================

    std::vector<std::wstring> collectAscFiles(const std::wstring& folder) {
        std::vector<std::wstring> files;

        std::wstring pattern = folder;
        if (!pattern.empty()) {
            wchar_t last = pattern.back();
            if (last != L'\\' && last != L'/') {
                pattern += L"\\";
            }
        }
        pattern += L"*.asc";

        WIN32_FIND_DATAW ffd;
        HANDLE hFind = FindFirstFileW(pattern.c_str(), &ffd);
        if (hFind == INVALID_HANDLE_VALUE) {
            return files;
        }
        do {
            if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                std::wstring path = folder;
                wchar_t last = path.empty() ? L'\0' : path.back();
                if (last != L'\\' && last != L'/') {
                    path += L"\\";
                }
                path += ffd.cFileName;
                files.push_back(path);
            }
        } while (FindNextFileW(hFind, &ffd));
        FindClose(hFind);

        std::sort(files.begin(), files.end());
        return files;
    }

    winrt::fire_and_forget MainWindow::ICP()
    {
        winrt::apartment_context ui_thread;
        auto lifetime = get_strong();
        if (m_isRunning)
            co_return;

        m_isRunning = true;
        ActionButton().IsEnabled(false);
        BusyRing().IsActive(true);
        BusyRing().Visibility(Visibility::Visible);
		ProgressBar().Value(0);
		ProgressBar().Visibility(Visibility::Visible);
        co_await winrt::resume_background();
        std::vector<std::wstring> ascFiles = collectAscFiles(folderPath);
        if (ascFiles.empty()) {
            ChangeStatus(L"未找到任何 .asc 文件。");
            co_await ui_thread;
            ActionButton().IsEnabled(true);
            m_isRunning = false;
            // 完成后
            BusyRing().IsActive(false);
            BusyRing().Visibility(Visibility::Collapsed);
            ProgressBar().Value(100);
            ProgressBar().Visibility(Visibility::Collapsed);
            co_return;
        }
        PointCloud accumulatedCloud;
        std::vector<Vec3> accumulatedNormals;

        for (size_t i = 0; i < ascFiles.size(); ++i)
        {
			ChangeStatus(L"处理文件 " + std::to_wstring(i + 1) + L" / " + std::to_wstring(ascFiles.size()) + L": " + ascFiles[i]);
            PointCloud cloud;
            if (!loadPointCloud(ascFiles[i], cloud))
            {
                co_await winrt::resume_foreground(Dispatcher());
                StatusTextBlock().Text(L"无法加载文件: " + ascFiles[i]);
                ActionButton().IsEnabled(true);
                m_isRunning = false;
                // 完成后
                BusyRing().IsActive(false);
                BusyRing().Visibility(Visibility::Collapsed);
                ProgressBar().Value(100);
                ProgressBar().Visibility(Visibility::Collapsed);
				co_return;
            }

            if (i == 0)
            {
                accumulatedCloud = cloud;
                estimateNormalsPCA(accumulatedCloud, accumulatedNormals, 20);
                DispatcherQueue().TryEnqueue([this, cloud = accumulatedCloud, normals = accumulatedNormals]()
                    {
                        std::lock_guard lock(m_visMutex);
                        m_visCloud = cloud;
                        m_visNormals = normals;
                        PointCanvas().Invalidate(); // 触发重绘
                    });
                continue;
            }
			double now_percent = (double)(i) / (double)(ascFiles.size()) * 100.0;
			ReportProgress(now_percent);
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            ICPPointToPlane(cloud, accumulatedCloud, accumulatedNormals, R, t, 20, 0.5, 1e-6);
			ReportProgress(now_percent + (1.0 / ascFiles.size())*0.9 * 100.0);
            accumulatedCloud.insert(accumulatedCloud.end(), cloud.begin(), cloud.end());
            estimateNormalsPCA(accumulatedCloud, accumulatedNormals, 20);
            ReportProgress(now_percent + (1.0 / ascFiles.size()) * 100.0);
            DispatcherQueue().TryEnqueue([this, cloud = accumulatedCloud, normals = accumulatedNormals]()
                {
                    std::lock_guard lock(m_visMutex);
                    m_visCloud = cloud;
                    m_visNormals = normals;
                    PointCanvas().Invalidate(); // 触发重绘
                });
        }
        std::wstring outputFile = folderPath + L"\\merged_output.asc";
        if (!savePointCloud(outputFile, accumulatedCloud)) {
            ChangeStatus(L"无法保存合并后的点云到: " + outputFile);
            co_await ui_thread;
            ActionButton().IsEnabled(true);
            m_isRunning = false;
            // 完成后
            BusyRing().IsActive(false);
            BusyRing().Visibility(Visibility::Collapsed);
            ProgressBar().Value(100);
            ProgressBar().Visibility(Visibility::Collapsed);
            co_return;
        }
        
        
        co_await ui_thread;
        StatusTextBlock().Text(L"处理完成，合并后的点云已保存到: " + outputFile);
        ActionButton().IsEnabled(true);
        m_isRunning = false;
        // 完成后
        BusyRing().IsActive(false);
        BusyRing().Visibility(Visibility::Collapsed);
        ProgressBar().Value(100);
        ProgressBar().Visibility(Visibility::Collapsed);
    }

    void MainWindow::PointCanvas_Draw(
        winrt::Microsoft::Graphics::Canvas::UI::Xaml::CanvasControl const& sender,
        winrt::Microsoft::Graphics::Canvas::UI::Xaml::CanvasDrawEventArgs const& args) 
    {
        std::lock_guard<std::mutex> lock(m_visMutex);
        if (m_visCloud.empty() || m_visCloud.size() != m_visNormals.size())
            return;

        auto ds = args.DrawingSession();

        // ---------- bounding box ----------
        double minX = 1e9, minY = 1e9, minZ = 1e9;
        double maxX = -1e9, maxY = -1e9, maxZ = -1e9;
        for (auto& p : m_visCloud)
        {
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            minZ = std::min(minZ, p.z);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
            maxZ = std::max(maxZ, p.z);
        }

        double cx = 0.5 * (minX + maxX);
        double cy = 0.5 * (minY + maxY);
        double sizeX = maxX - minX;
        double sizeY = maxY - minY;

        float W = sender.ActualWidth();
        float H = sender.ActualHeight();
        if (W < 2 || H < 2) return;

        double scale = 0.9 * std::min(
            W / (sizeX + 1e-9),
            H / (sizeY + 1e-9));

        // ---------- light ----------
        Eigen::Vector3d lightDir(0.3, 0.3, 0.9);
        lightDir.normalize();

        // ---------- parameters tuned for contours ----------
        float radius = std::max(0.8f, float(scale * 0.002));

        for (size_t i = 0; i < m_visCloud.size(); ++i)
        {
            const auto& p = m_visCloud[i];
            const auto& n = m_visNormals[i];

            float sx = float((p.x - cx) * scale + W * 0.5);
            float sy = float((p.y - cy) * scale + H * 0.5);
            sy = H - sy;

            double ndotl = n.x * lightDir.x()
                + n.y * lightDir.y()
                + n.z * lightDir.z();

            double intensity = 0.5 * ndotl + 0.5;
            intensity = std::clamp(intensity, 0.0, 1.0);

            uint8_t alpha = static_cast<uint8_t>(20 + 60 * intensity);
            uint8_t base = static_cast<uint8_t>(180 + 50 * intensity);

            auto color = Windows::UI::ColorHelper::FromArgb(
                alpha,
                base,
                base,
                static_cast<uint8_t>(200 + 30 * intensity));

            ds.FillCircle(sx, sy, radius, color);
        }
    }

    void MainWindow::ReportProgress(double percent)
    {
        DispatcherQueue().TryEnqueue([this, percent]()
            {
                ProgressBar().Value(percent);
                StatusTextBlock().Text(
                    L"处理中... " + std::to_wstring((int)percent) + L"%");
            });
    }

    void MainWindow::ChangeStatus(std::wstring status)
    {
        DispatcherQueue().TryEnqueue([this, status]()
            {
                StatusTextBlock().Text(L"处理中... " + status);
            });
	}

    // 预留的按钮点击事件方法
    void MainWindow::ActionButton_Click(IInspectable const& /*sender*/, RoutedEventArgs const& /*e*/)
    {
        // TODO: 在此处编写按钮点击后的逻辑
        // 示例：更新状态文本
        if(!fileChosen)
        {
            HWND hwnd{};
            auto windowNative = this->try_as<IWindowNative>();
            windowNative->get_WindowHandle(&hwnd);
            folderPath = SelectFolderPath(hwnd);
            std::wstring status = L"选择的文件夹路径： " + folderPath;
            StatusTextBlock().Text(status);
            return;
        }
        else {
            progress = 0;
            StatusTextBlock().Text(L"处理中，请稍候...");
            ICP(); 
        }
		
		
    }
    std::wstring MainWindow::GetCurrentWorkingDirectory()
    {
        wchar_t buffer[MAX_PATH]{};
        DWORD len = GetCurrentDirectoryW(MAX_PATH, buffer);
        return len ? std::wstring(buffer) : L"";
    }
    std::wstring MainWindow::SelectFolderPath(HWND owner)
    {
        std::wstring result;

        HRESULT hrInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
        bool needUninit = hrInit == S_OK || hrInit == S_FALSE;

        IFileDialog* pfd = nullptr;
        HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pfd));
        if (SUCCEEDED(hr) && pfd)
        {
            DWORD dwOptions = 0;
            if (SUCCEEDED(pfd->GetOptions(&dwOptions)))
            {
                pfd->SetOptions(dwOptions | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);
            }

            if (SUCCEEDED(pfd->Show(owner)))
            {
                IShellItem* psi = nullptr;
                if (SUCCEEDED(pfd->GetResult(&psi)) && psi)
                {
                    PWSTR pszPath = nullptr;
                    if (SUCCEEDED(psi->GetDisplayName(SIGDN_FILESYSPATH, &pszPath)) && pszPath)
                    {
                        result = pszPath;
                        CoTaskMemFree(pszPath);
                    }
                    psi->Release();
                }
            }
            pfd->Release();
        }

        // 回退：旧式浏览对话框
        if (result.empty())
        {
            /*BROWSEINFOW bi = { 0 };
            bi.hwndOwner = owner;
            bi.lpszTitle = L"选择一个文件夹";
            bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE | BIF_EDITBOX;

            PIDLIST_ABSOLUTE pidl = SHBrowseForFolderW(&bi);
            if (pidl)
            {
                WCHAR path[MAX_PATH] = { 0 };
                if (SHGetPathFromIDListW(pidl, path))
                {
                    result = path;
                }
                CoTaskMemFree(pidl);
            }*/
            result = GetCurrentWorkingDirectory();
        }

        if (needUninit)
        {
            CoUninitialize();
        }
		fileChosen = !result.empty();
        if (fileChosen) {
            ActionButton().Content(winrt::box_value(L"执行"));
        }
        return result;
    }

   
}