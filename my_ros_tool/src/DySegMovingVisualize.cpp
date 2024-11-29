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
    vector<LabelType> labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving;
};

// 将标签文件和点云文件解析为静态点和动态点的点云
labeled_point* converter(const std::string &filename, const std::string &frame) {

    const std::string label_path = filename + "/label_txt/" + frame + ".txt";
    const std::string velodyne_path = filename + "/pc_txt/" + frame + ".txt";

    std::vector<LabelType> labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving(new pcl::PointCloud<pcl::PointXYZ>);

    std::ifstream file_label(label_path);
    std::ifstream file_velodyne(velodyne_path);

    if (!file_label.is_open() || !file_velodyne.is_open()) {
        std::cerr << "无法打开文件 " << label_path << " 或 " << velodyne_path << std::endl;
        return nullptr;
    }

    LabelType label;
    pcl::PointXYZ point;
    while (file_label >> label && file_velodyne >> point.x >> point.y >> point.z) {
        if (label == 1 || label == 0) {
            cloud_static->push_back(point);
        } else if (label == 2) {
            cloud_moving->push_back(point);
        }
    }

    if (file_label.bad() || file_velodyne.bad()) {
        std::cerr << "读取文件时发生错误。" << std::endl;
    }

    file_label.close();
    file_velodyne.close();

    labeled_point *result = new labeled_point;
    result->labels = labels;
    result->cloud_static = cloud_static;
    result->cloud_moving = cloud_moving;

    std::cout << "静态点数量: " << cloud_static->size() << std::endl;
    std::cout << "动态点数量: " << cloud_moving->size() << std::endl;

    return result;
}

// 格式化帧编号，确保长度为6位
std::string formatFrame(const std::string &frame){
    size_t zerotoadd = 6 - frame.size();
    return std::string(zerotoadd, '0') + frame;
}

// 可视化点云：静态点为黑色，动态点为红色
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_static, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_moving){

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> static_cloud_color_handler(cloud_static, 0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moving_cloud_color_handler(cloud_moving, 255, 0, 0);

    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_static, static_cloud_color_handler, "static cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_moving, moving_cloud_color_handler, "moving cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "static cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "moving cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <路径> <帧编号>" << std::endl;
        return -1;
    }

    const std::string path = argv[1];
    const std::string frame = argv[2];

    const std::string formatted_frame = formatFrame(frame);

    labeled_point *data = converter(path, formatted_frame);
    if (data) {
        visualizePointCloud(data->cloud_static, data->cloud_moving);
        delete data;
    } else {
        std::cerr << "转换过程中出现问题，无法显示点云。" << std::endl;
    }

    return 0;
}

