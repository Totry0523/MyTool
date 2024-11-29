cmake_minimum_required(VERSION 3.10)

# 项目名称
project(PointCloudViewer)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 PCL 和 Eigen 包
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# 查找 OpenGL (对于 PCL 可视化功能)
# find_package(OpenGL REQUIRED)

# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# 链接库目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加可执行文件
add_executable(3dboxinPCL 3dboxinPCL.cpp)
target_link_libraries(3dboxinPCL ${PCL_LIBRARIES} ${OpenGL_LIBRARIES})

add_executable(gem gem.cpp)
target_link_libraries(gem ${PCL_LIBRARIES} ${OpenGL_LIBRARIES})

add_executable(showSpheroid showSpheroid.cpp)
target_link_libraries(showSpheroid ${PCL_LIBRARIES} ${OpenGL_LIBRARIES})
