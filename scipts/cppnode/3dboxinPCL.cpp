#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

struct ObjectInfo {
    std::string label;
    float x, y, z;
    float length, width, height;
};

// 读取.bin文件，假设文件中是X,Y,Z格式的点云数据
bool loadPointCloudFromBin(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return false;
    }

    // Read the binary file
    pcl::PointXYZ point;
    while (file.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    std::cout << ("Loaded ") << cloud->points.size() << (" points.") << std::endl;
    cloud->height = 1; // Unorganized point cloud
    file.close();
    return true;
}

void drawBoundingBoxEdges(pcl::visualization::PCLVisualizer::Ptr viewer, const ObjectInfo& obj, int obj_index) {
    // 创建转换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << obj.x, obj.y, obj.z;

    // 获取每个角点的位置
    std::vector<Eigen::Vector3f> corners(8);
    corners[0] = transform * Eigen::Vector3f(-obj.length / 2, -obj.width / 2, -obj.height / 2);
    corners[1] = transform * Eigen::Vector3f(-obj.length / 2, obj.width / 2, -obj.height / 2);
    corners[2] = transform * Eigen::Vector3f(obj.length / 2, obj.width / 2, -obj.height / 2);
    corners[3] = transform * Eigen::Vector3f(obj.length / 2, -obj.width / 2, -obj.height / 2);
    corners[4] = transform * Eigen::Vector3f(-obj.length / 2, -obj.width / 2, obj.height / 2);
    corners[5] = transform * Eigen::Vector3f(-obj.length / 2, obj.width / 2, obj.height / 2);
    corners[6] = transform * Eigen::Vector3f(obj.length / 2, obj.width / 2, obj.height / 2);
    corners[7] = transform * Eigen::Vector3f(obj.length / 2, -obj.width / 2, obj.height / 2);

    // 绘制边框
    int id = 0;
    auto addEdge = [&](int idx1, int idx2) {
        // 使用物体索引 obj_index 生成唯一 ID
        viewer->addLine(pcl::PointXYZ(corners[idx1].x(), corners[idx1].y(), corners[idx1].z()),
                        pcl::PointXYZ(corners[idx2].x(), corners[idx2].y(), corners[idx2].z()),
                        1.0, 0.0, 0.0, "edge_" + std::to_string(obj_index) + "_" + std::to_string(id++));
    };

    // 绘制立方体的12条边
    addEdge(0, 1); addEdge(1, 2); addEdge(2, 3); addEdge(3, 0);  // 底面
    addEdge(4, 5); addEdge(5, 6); addEdge(6, 7); addEdge(7, 4);  // 顶面
    addEdge(0, 4); addEdge(1, 5); addEdge(2, 6); addEdge(3, 7);  // 侧面
}


int main(int argc, char** argv) {
    // 检查参数
    if (argc != 3 ) {
        std::cerr << "Usage: " << argv[0] << " <pointcloud.bin>"  << "<label02.txt>"<< std::endl;
        return -1;
    }

    // 加载点云
    std::string bin_file = argv[1];
    std::string label2_file = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!loadPointCloudFromBin(bin_file, cloud)) {
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_green(new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& pt: cloud->points){
        if(pt.x < 40 && pt.y < 40 && pt.x > -40 && pt.y > -40) 
            Cloud_green->points.push_back(pt);
    }

    // 读取检测结果
    std::vector<ObjectInfo> objects;
    std::ifstream obj_file(label2_file);
    std::string line;
    while (std::getline(obj_file, line)) {
        std::istringstream iss(line);
        ObjectInfo obj;
        iss >> obj.label >> obj.x >> obj.y >> obj.z >> obj.length >> obj.width >> obj.height;
        objects.push_back(obj);
    }

    // 创建PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_near_handler(Cloud_green, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler,"cloud");
    viewer->addPointCloud(Cloud_green, cloud_color_near_handler, "cloud_green");
    viewer->addCoordinateSystem(5, "global");
    viewer->setBackgroundColor(0, 0, 0);

    // 绘制每个物体的检测框边框
    for (size_t i = 0; i < objects.size(); ++i) {
        drawBoundingBoxEdges(viewer, objects[i], i);
    }

    // 开始显示
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;    

}
