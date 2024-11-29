#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tf.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <fstream>
#include <chrono>

// 全局变量定义
std::ofstream backend_time_File;
int frame_index = 0;

void IMUCallback(const sensor_msgs::Imu& msg) {
    auto imu_start = std::chrono::high_resolution_clock::now();

    if (msg.orientation_covariance[0] < 0) return;

    tf::Quaternion quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    ROS_INFO("滚转= %.2f 俯仰= %.2f 朝向= %.2f", roll, pitch, yaw);

    auto imu_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> imu_duration = imu_end - imu_start;
    log_backend_time(imu_duration.count(), -1);
}

void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    auto pc_start = std::chrono::high_resolution_clock::now();

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    ROS_INFO("Received PointCloud2 message. Number of points: %ld", cloud->size());

    auto pc_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> pc_duration = pc_end - pc_start;
    log_backend_time(-1, pc_duration.count());
}

void log_backend_time(double imu_time, double pc_time) {
    if (!backend_time_File.is_open()) return;

    backend_time_File << frame_index << ",";
    backend_time_File << (imu_time >= 0 ? std::to_string(imu_time) : "") << ",";
    backend_time_File << (pc_time >= 0 ? std::to_string(pc_time) : "") << std::endl;

    frame_index++;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "demo_imu_data");

    std::string odom_eigen_csv_file;
    ros::NodeHandle n;
    n.param<std::string>("odom_eigen_csv_file", odom_eigen_csv_file, std::string("output.csv")); 

    backend_time_File.open(odom_eigen_csv_file, std::ios::out | std::ios::trunc);
    if (!backend_time_File.is_open()) {
        ROS_ERROR("Failed to open CSV file: %s", odom_eigen_csv_file.c_str());
        return -1;
    }

    backend_time_File << "frame_index" << "," << "imu_time" << ","<< "pc_time" << std::endl;
    
    ros::Subscriber sub_imu = n.subscribe("/fd_imu/imu", 50, &IMUCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_imu2 = n.subscribe("/imu_raw", 50, &IMUCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_lidar = n.subscribe("/points_raw", 10, &LidarCallback, ros::TransportHints().tcpNoDelay());
    ros::spin();

    backend_time_File.close();  // 关闭文件
    return 0;
}