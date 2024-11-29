#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace std;

typedef uint16_t LabelType;

struct labeled_point
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving;
};

// 将标签文件和点云文件解析为静态点和动态点的点云
labeled_point* converter(const std::string &filename, const std::string &__frame,LabelType __selLabel) {

    const std::string filelabe_path = filename + "/labels/" + __frame +".label";
    const std::string filevelodyne_path = filename + "/velodyne/" + __frame + ".bin";

    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_static(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_moving(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream file_lable(filelabe_path, std::ios::binary);
    std::ifstream file_velodyne(filevelodyne_path, std::ios::binary);

    if (file_lable.is_open() && file_velodyne.is_open()) {
        LabelType label;
        pcl::PointXYZ point;

        while (file_lable.read(reinterpret_cast<char*>(&label), sizeof(2*label)) &&
               file_velodyne.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZ))) {
            if (label == __selLabel) {
                __cloud_static->push_back(point);
            }else{
                __cloud_moving->push_back(point);
            }
            // 否则跳过地面点
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
    std::cout << "selLabel pioint size: " << __cloud_static->size() << std::endl;
    std::cout << "unselLabel pioint size: " << __cloud_moving->size() << std::endl;

    labeled_point *return_type = new labeled_point;
    return_type->cloud_static = __cloud_static;
    return_type->cloud_moving = __cloud_moving;

    return return_type;
}

// 格式化帧编号，确保长度为6位
std::string formatFrame(const std::string &frame){
    size_t zerotoadd = 6 - frame.size();
    return std::string(zerotoadd, '0') + frame;
}

// 可视化点云：非标签点为黑色，标签点为红色
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving){

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> static_cloud_color_handler(cloud_static, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moving_cloud_color_handler(cloud_moving, 0, 0, 0);

    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_static, static_cloud_color_handler, "static cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_moving, moving_cloud_color_handler, "moving cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "static cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "moving cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "用法: " << argv[0] << " <路径> <帧编号><标签>" << std::endl;
        return -1;
    }

    const std::string path = argv[1];
    const std::string frame = argv[2];
    const std::string label = argv[3];

    const std::string formatted_frame = formatFrame(frame);
    LabelType converted_label = std::stoi(label);

    labeled_point *data = converter(path, formatted_frame,converted_label);
    if (data) {
        visualizePointCloud(data->cloud_static, data->cloud_moving);
        delete data;
    } else {
        std::cerr << "转换过程中出现问题，无法显示点云。" << std::endl;
    }

    return 0;
}

