#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>
#include <Eigen/Dense>

// 定义椭球结构体，包含半轴长度、平移和旋转参数
struct Spheroid {
    Eigen::Vector3d semi_axes;   // 半轴长度：a, b, c
    Eigen::Vector3d translation; // 平移：x, y, z
    Eigen::Vector3d rotation;    // 旋转（欧拉角）：roll, pitch, yaw
};

// 添加椭球模型到 PCL 可视化器
void addSpheroidToViewer(const Spheroid& spheroid, pcl::visualization::PCLVisualizer::Ptr viewer) {
    // 使用VTK生成球体
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(1.0);            // 基础球体半径为1
    sphereSource->SetThetaResolution(100);   // 设置经度分辨率
    sphereSource->SetPhiResolution(100);     // 设置纬度分辨率
    sphereSource->Update();

    // 设置缩放变换：将球体按半轴长度缩放为椭球
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Scale(spheroid.semi_axes[0], spheroid.semi_axes[1], spheroid.semi_axes[2]);

    // 应用旋转
    transform->RotateX(spheroid.rotation[0] * 180.0 / M_PI);  // roll
    transform->RotateY(spheroid.rotation[1] * 180.0 / M_PI);  // pitch
    transform->RotateZ(spheroid.rotation[2] * 180.0 / M_PI);  // yaw

    // 应用平移
    transform->Translate(spheroid.translation[0], spheroid.translation[1], spheroid.translation[2]);

    // 将变换应用到球体上
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(sphereSource->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // 使用 PCL 可视化器添加生成的椭球模型
    viewer->addModelFromPolyData(transformFilter->GetOutput(), "spheroid");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "spheroid");  // 灰色
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "spheroid");          // 不透明
}

int main() {
    // 创建PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Spheroid Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  // 设置背景为黑色

    // 定义一个椭球
    Spheroid spheroid;
    spheroid.semi_axes = Eigen::Vector3d(1.0, 2.0, 6.0);  // 半轴长度：a, b, c
    spheroid.translation = Eigen::Vector3d(0.0, 0.0, 0.0); // 平移
    spheroid.rotation = Eigen::Vector3d(0.0, 0.0, 0.0);    // 旋转（roll, pitch, yaw）

    // 将椭球添加到可视化器中
    addSpheroidToViewer(spheroid, viewer);

    // 可视化循环
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
