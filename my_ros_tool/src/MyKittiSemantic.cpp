#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <filesystem> // C++17

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace std;

typedef uint16_t LabelType;

struct labeled_point
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_movable;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground;
};


labeled_point* converter(const std::string &filename, const std::string &__frame) {

    const std::string filelabe_path = filename + "/labels/" + __frame +".label";
    const std::string filevelodyne_path = filename + "/velodyne/" + __frame + ".bin";

    std::vector<LabelType> __labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_static(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_moving(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_movable(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream file_lable(filelabe_path, std::ios::binary);
    std::ifstream file_velodyne(filevelodyne_path, std::ios::binary);

    if (file_lable.is_open() && file_velodyne.is_open()) {
        LabelType label;
        pcl::PointXYZ point;

        while (file_lable.read(reinterpret_cast<char*>(&label), sizeof(2*label)) &&
               file_velodyne.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
            if (label > 200) {
                __cloud_moving->push_back(point);
            }else if(label >= 10 && label <= 32){
                __cloud_movable->push_back(point);
            }else if((label >= 40 && label <= 49) || label == 72){
                __cloud_ground->push_back(point);
            }else{
                __cloud_static->push_back(point);
            }
            // 否则跳过地面点 KittiMovingVisualize MfmosVisualize
        }

        if (file_lable.bad()) {
            std::cerr << "读取标签文件时发生错误。" << std::endl;
        }

        if (file_velodyne.bad()) {
            std::cerr << "读取点云文件时发生错误。" << std::endl;
        }

        file_lable.close();
        file_velodyne.close();
    } else {
        std::cerr << "无法打开文件 " << filelabe_path << " 和/或 " << filevelodyne_path << std::endl;
    }
    std::cout << "static pioint size: " << __cloud_static->size() << std::endl;
    std::cout << "moving pioint size: " << __cloud_moving->size() << std::endl;
    std::cout << "movable pioint size: " << __cloud_movable->size() << std::endl;
    std::cout << "ground pioint size: " << __cloud_ground->size() << std::endl;

    labeled_point *return_type = new labeled_point;
    return_type->cloud_static = __cloud_static;
    return_type->cloud_moving = __cloud_moving;
    return_type->cloud_movable = __cloud_movable;
    return_type->cloud_ground = __cloud_ground;

    return return_type;
}

std::string formatFrame(const std::string &frame){

    size_t zerotoadd = 6 - frame.size();
    return std::string(zerotoadd, '0') + frame;
}

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_static, const pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_moving,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_movable, const pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_ground){

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> static_cloud_color_handler(__cloud_static, 0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moving_cloud_color_handler(__cloud_moving, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> movable_cloud_color_handler(__cloud_movable, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ground_cloud_color_handler(__cloud_ground, 0, 255, 0);
    viewer.setBackgroundColor(255, 255, 255);

    viewer.addPointCloud<pcl::PointXYZ>(__cloud_static, static_cloud_color_handler, "static_cloud");
    viewer.addPointCloud<pcl::PointXYZ>(__cloud_moving, moving_cloud_color_handler, "moving_cloud");
    viewer.addPointCloud<pcl::PointXYZ>(__cloud_movable, movable_cloud_color_handler, "movable_cloud");
    viewer.addPointCloud<pcl::PointXYZ>(__cloud_ground, ground_cloud_color_handler, "ground_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "static_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "moving_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "movable_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground_cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main(int agrc, char** argv) {

    const std::string path = argv[1];
    const std::string frame = argv[2];

    const std::string formatterframe = formatFrame(frame);
    labeled_point *data = converter(path,formatterframe);
    visualizePointCloud(data->cloud_static, data->cloud_moving,data->cloud_movable,data->cloud_ground);
    delete data;

    return 0;
}