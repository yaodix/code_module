// 车体系4条直线投影到相机图像的几种角度变换效果
// 优化方法自动求解车体到相机系的变换参数

#include <iostream>
#include <ceres/ceres.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// 定义成像1920*1080的虚拟相机内参数
cv::Mat_<double>(3, 3) virtual_intrinsic << 1000, 0, 960,
                             0, 1000, 540,
                             0, 0, 1;

cv::Mat gt_veh2cam <<;

// 车体系虚拟车道线生成
pointpt


int main() {
    return 0;
}