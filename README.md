# SomeToolofROS
一些为方便个人日常开发所编写的ROS工具

### 目录

## my_pcl_tool
viewbin.cpp 查看一个.bin格式的点云

viewpcd.cpp 查看一个.pcd格式的点云

## my_ros_tool
KittiCleanGround_demo.cpp 一个去除KITTI数据集地面点（label=40）并可视化的demo


KittiCleanGroundAndSave.cpp 去除所有文件的地面点并保存到另一个文件夹

`rosrun my_ros_tool KittiCleanGroundAndSave </path/to/input> </path/to/output>`

KittiMovingVisualize.cpp 可视化点云的移动物体点

`rosrun my_ros_tool KittiMovingVisualize </path/to/tragetdir>`

SemanticKitiiLabelVisualize 可视化KITTI数据集的一个标签

` rosrun my_ros_tool SemanticKitiiLabelVisualize <path> <frame> <label>`

MyKittiSemantic 将KITTI数据集中的地面、可移动物体、移动物体分别进行分割显示

` rosrun my_ros_tool MyKittiSemantic <path> <frame>`
***
![img](3E84F52A.gif)

