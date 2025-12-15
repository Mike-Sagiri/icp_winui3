# Use cpp/winrt and winui3 for icp pointcloud alignment

This repository demonstrates how to use C++/WinRT and WinUI 3 to create a simple application for aligning ICP (Iterative Closest Point) point clouds. The application provides a user interface for loading, visualizing, and aligning 3D point cloud data.  

## Note
This project is a basic implementation and serves as a starting point for more complex applications. It focuses on demonstrating the integration of C++/WinRT, WinUI 3, and ICP algorithms. I use it as a learning project to get familiar with these technologies.

## Prerequisites
- Visual Studio 2022 or later with C++ and Windows SDK components installed.
- Windows 10 or later.
- C++/WinRT extension for Visual Studio.
- WinUI 3 extension for Visual Studio.
- Nanoflann library for nearest neighbor search.
- Eigen library for matrix operations.
- You need to change the setting of c/c++ standard to c++20 or later.
- You need to install the nuget package "Microsoft.Graphics.Win2d".
- You need to add eigen and nanoflann to your project include directories.

## Getting Started
1. Clone the repository
2. Open the solution file in Visual Studio, and **Release** x64 mode is recommended.
3. Build the solution to restore NuGet packages and compile the code.
4. Run the application. You should see a window with options to load point clouds and perform ICP alignment.

## Tutorial for developing the application
https://www.xjh.zone/myblogs/posts/winui3-a-glance/
