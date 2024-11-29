#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
    float yaw;
};

// 定义椭球结构体，包含半轴长度、平移和旋转参数
struct Spheroid {
    std::string label;
    float x,y,z; // 平移：x, y, z
    float semi_axes_x,semi_axes_y,semi_axes_z;   // 半轴长度：a, b, c
    float yaw;    // 旋转（欧拉角）：roll, pitch, yaw
};

// 读取.bin文件，假设文件中是X,Y,Z格式的点云数据
bool loadPointCloudFromBin(const std::string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return false;
    }

    pcl::PointXYZ point;
    while (file.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud
    file.close();
    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
    return true;
}

void addSpheroidToViewer(const Spheroid& spheroid,const pcl::visualization::PCLVisualizer::Ptr viewer, int index) {
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(1.0); // 基础球体半径为1
    sphereSource->SetThetaResolution(30);
    sphereSource->SetPhiResolution(30);
    sphereSource->Update();

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    transform->Translate(spheroid.x, spheroid.y, spheroid.z);
    transform->RotateZ(spheroid.yaw * 180.0 / M_PI); // yaw旋转
    transform->Scale(spheroid.semi_axes_x/2, spheroid.semi_axes_y/2, spheroid.semi_axes_z/2); // 按半轴长度缩放

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(sphereSource->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // 生成唯一的ID
    std::string unique_id = "spheroid_" + std::to_string(index);
    viewer->addModelFromPolyData(transformFilter->GetOutput(), unique_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, unique_id); // 灰色
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, unique_id); // 不透明
}

void drawBoundingBoxEdges(const pcl::visualization::PCLVisualizer::Ptr viewer, const ObjectInfo& obj, int obj_index) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << obj.x, obj.y, obj.z;

    // 创建 yaw 旋转矩阵
    Eigen::Matrix3f rotation;
    float yaw = obj.yaw; // 假设 yaw 是弧度
    rotation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    std::vector<Eigen::Vector3f> corners(8);
    // 计算边界框的角点
    corners[0] = rotation * Eigen::Vector3f(-obj.length / 2, -obj.width / 2, -obj.height / 2);
    corners[1] = rotation * Eigen::Vector3f(-obj.length / 2, obj.width / 2, -obj.height / 2);
    corners[2] = rotation * Eigen::Vector3f(obj.length / 2, obj.width / 2, -obj.height / 2);
    corners[3] = rotation * Eigen::Vector3f(obj.length / 2, -obj.width / 2, -obj.height / 2);
    corners[4] = rotation * Eigen::Vector3f(-obj.length / 2, -obj.width / 2, obj.height / 2);
    corners[5] = rotation * Eigen::Vector3f(-obj.length / 2, obj.width / 2, obj.height / 2);
    corners[6] = rotation * Eigen::Vector3f(obj.length / 2, obj.width / 2, obj.height / 2);
    corners[7] = rotation * Eigen::Vector3f(obj.length / 2, -obj.width / 2, obj.height / 2);

    // 将角点平移到正确的位置
    for (int i = 0; i < 8; ++i) {
        corners[i] += Eigen::Vector3f(obj.x, obj.y, obj.z);
    }

    int id = 0;
    auto addEdge = [&](int idx1, int idx2) {
        viewer->addLine(pcl::PointXYZ(corners[idx1].x(), corners[idx1].y(), corners[idx1].z()),
                                       pcl::PointXYZ(corners[idx2].x(), corners[idx2].y(), corners[idx2].z()),
                                       1.0, 0.0, 0.0, "edge_" + std::to_string(obj_index) + "_" + std::to_string(id++));
    };

    addEdge(0, 1); addEdge(1, 2); addEdge(2, 3); addEdge(3, 0);  // 底面
    addEdge(4, 5); addEdge(5, 6); addEdge(6, 7); addEdge(7, 4);  // 顶面
    addEdge(0, 4); addEdge(1, 5); addEdge(2, 6); addEdge(3, 7);  // 侧面
}


int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pointcloud.bin> <label02.txt>" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!loadPointCloudFromBin(argv[1], cloud)) {
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_green(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : cloud->points) {
        if (pt.x < 40 && pt.y < 40 && pt.x > -40 && pt.y > -40) 
            Cloud_green->points.push_back(pt);
    }

    std::vector<ObjectInfo> objects;
    std::vector<Spheroid> spheroids;
    std::ifstream obj_file(argv[2]);
    std::string line;
    while (std::getline(obj_file, line)) {
        std::istringstream iss1(line);
        std::istringstream iss2(line);
        ObjectInfo obj;
        Spheroid sph;
        iss1 >> obj.label >> obj.x >> obj.y >> obj.z >> obj.length >> obj.width >> obj.height >> obj.yaw;
        iss2 >> sph.label >> sph.x >> sph.y >> sph.z >> sph.semi_axes_x >> sph.semi_axes_y >> sph.semi_axes_z >> sph.yaw;
        objects.push_back(obj);
        spheroids.push_back(sph);
    }
    obj_file.close();

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_near_handler(Cloud_green, 0, 255, 0);
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addCoordinateSystem(6);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_color_handler, "cloud");
    viewer->addPointCloud<pcl::PointXYZ>(Cloud_green,cloud_color_near_handler ,"Cloud_green");

    for (size_t i = 0; i < objects.size(); ++i) {
        // //output for test
        // std::cout << "object " << i << ": " << objects[i].label
        //  << " //xyz: " << objects[i].x << " " << objects[i].y << " " << objects[i].z
        //  << " //lwh: " << objects[i].length << " " << objects[i].width << " " << objects[i].height
        //  << " //yaw: " << objects[i].yaw << std::endl;
        drawBoundingBoxEdges(viewer, objects[i], i);
    }

    for(size_t i = 0; i < spheroids.size(); ++i) {
        // //output for test
        // std::cout << "spheroid " << i << ": " << spheroids[i].label
        //  << " //xyz: " << spheroids[i].x << " " << spheroids[i].y << " " << spheroids[i].z 
        //  << " //lwh: " << spheroids[i].semi_axes_x << " " << spheroids[i].semi_axes_y << " " << spheroids[i].semi_axes_z
        //  << " //yaw: " << spheroids[i].yaw << std::endl;
        addSpheroidToViewer(spheroids[i], viewer,i);
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    return 0;
}
