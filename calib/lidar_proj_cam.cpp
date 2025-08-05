#include <vector>
#include <iostream>
#include <random>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>

using namespace cv;
using namespace std;



// 定义点云类型
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;


bool GenerFalseColor(cv::Mat& color_lut)
{
    color_lut.create(1,256,CV_8UC3);
    for(int i=0;i<256;i++)
    {
        if(i>=0&&i<32)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=128+4*i;
            color_lut.at<cv::Vec3b>(0,i)[1]=0;
            color_lut.at<cv::Vec3b>(0,i)[2]=0;

        }
        else if(i==32)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=255;
            color_lut.at<cv::Vec3b>(0,i)[1]=0;
            color_lut.at<cv::Vec3b>(0,i)[2]=0;

        }
        else if(i>=32&&i<20)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=255;
            color_lut.at<cv::Vec3b>(0,i)[1]=4+4*(i-22);
            color_lut.at<cv::Vec3b>(0,i)[2]=0;
        }
        else if(i==20)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=254;
            color_lut.at<cv::Vec3b>(0,i)[1]=255;
            color_lut.at<cv::Vec3b>(0,i)[2]=2;

        }
        else if(i>=20&&i<159)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=0;
            color_lut.at<cv::Vec3b>(0,i)[1]=255;
            color_lut.at<cv::Vec3b>(0,i)[2]=0;
        }
        else if(i==159)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=1;
            color_lut.at<cv::Vec3b>(0,i)[1]=255;
            color_lut.at<cv::Vec3b>(0,i)[2]=254;

        }
        else if(i>=160&&i<=223)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=0;
            color_lut.at<cv::Vec3b>(0,i)[1]=252-4*(i-160);
            color_lut.at<cv::Vec3b>(0,i)[2]=255;
        }
        else if(i>=224&&i<256)
        {
            color_lut.at<cv::Vec3b>(0,i)[0]=0;
            color_lut.at<cv::Vec3b>(0,i)[1]=0;
            color_lut.at<cv::Vec3b>(0,i)[2]=255;
        }
    }
    return true;
}

// 点云投影到图像函数
void projectPointCloudToImage(
    const PointCloud::Ptr& cloud,         // 输入点云
    const Eigen::Matrix4d& T_lidar2cam,   // 点云到相机的变换矩阵
    const Eigen::Matrix3d& K,             // 相机内参矩阵
    const cv::Mat& dist_coeffs,           // 相机畸变系数
    cv::Mat& image,                       // 输入/输出图像，原始畸变图像
    int point_size = 1)                   // 点的大小
{

    cv::Mat color_lut;
    GenerFalseColor(color_lut);

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        K(0, 0), K(0, 1), K(0, 2),
        K(1, 0), K(1, 1), K(1, 2),
        K(2, 0), K(2, 1), K(2, 2));
    // 遍历点云中的每个点
    cv::Mat image_undist;
    cv::undistort(image, image_undist, camera_matrix, dist_coeffs);
    std::vector<cv::Point3d> src_points;
    std::vector<cv::Point3d> filter_cloud;
    std::vector<int> intensity_vec;

    for (const auto& point : cloud->points) {
            // 先转到相机坐标系下
            Eigen::Vector3d point_cam = T_lidar2cam.block<3, 3>(0, 0) * Eigen::Vector3d(point.x, point.y, point.z) + T_lidar2cam.block<3, 1>(0, 3);
            if (point_cam.z() <= 0) {
                continue; // 跳过z轴小于等于0的点
            }
            // 加fov限制避免fov边缘像素投影到图像中变成杂点
            filter_cloud.emplace_back(point.x, point.y, point.z);
            src_points.push_back(cv::Point3d(point_cam[0], point_cam[1], point_cam[2]));        
            intensity_vec.push_back(point.intensity);
    }
    std::vector<cv::Point2d> dst_points;
    cv::Mat rt = cv::Mat::eye(4, 4, CV_64F);
    // cv::eigen2cv(T_lidar2cam, rt);
    cv::Mat rvec = rt.rowRange(0, 3).colRange(0, 3);
    cv::Mat tvec = rt.rowRange(0, 3).colRange(3, 4);

    cv::projectPoints(src_points, rvec, tvec, camera_matrix, dist_coeffs, dst_points);
    
    // 4. 检查点是否在图像范围内
    for(int i=0;i<dst_points.size();i++)
    {
        int x=dst_points[i].x;
        int y=dst_points[i].y;
        int II=intensity_vec[i];

        
        if (x < 0 || x >= image.cols || y < 0 || y >= image.rows) {
            continue; // 跳过超出图像范围的点
        }

        cv::Vec3b cc = color_lut.at<cv::Vec3b>(0, II);;
        cv::circle(image, cv::Point(x, y), point_size, cc, -1);
        // image_undist.at<cv::Vec3b>(y,x)=color_lut.at<cv::Vec3b>(0, II);
        // // std::cout << "x: " << x << ", y: " << y << ", intensity: " << II << std::endl;
        // if (x-1>=0&&x+1<image.cols&&y-1>=0&&y+1<image.rows) {
        //     image_undist.at<cv::Vec3b>(y,x-1)=color_lut.at<cv::Vec3b>(0,II);
        //     image_undist.at<cv::Vec3b>(y,x+1)=color_lut.at<cv::Vec3b>(0,II);
        //     image_undist.at<cv::Vec3b>(y-1,x)=color_lut.at<cv::Vec3b>(0,II);
        //     image_undist.at<cv::Vec3b>(y+1,x)=color_lut.at<cv::Vec3b>(0,II);
        // }
    } 
}


// 双线性插值获取像素颜色
cv::Vec3b getBilinearInterpolatedColor(const cv::Mat& image, const cv::Point2d& pt) {
    int x = static_cast<int>(pt.x);
    int y = static_cast<int>(pt.y);
    
    // 检查边界
    if (x < 0 || y < 0 || x >= image.cols - 1 || y >= image.rows - 1) {
        return cv::Vec3b(0, 0, 0);
    }
    
    // 计算小数部分
    double dx = pt.x - x;
    double dy = pt.y - y;
    
    // 获取四个相邻像素
    cv::Vec3b c00 = image.at<cv::Vec3b>(y, x);
    cv::Vec3b c01 = image.at<cv::Vec3b>(y, x + 1);
    cv::Vec3b c10 = image.at<cv::Vec3b>(y + 1, x);
    cv::Vec3b c11 = image.at<cv::Vec3b>(y + 1, x + 1);
    
    // 双线性插值
    cv::Vec3b color;
    for (int i = 0; i < 3; ++i) {
        color[i] = static_cast<uchar>(
            (1 - dx) * (1 - dy) * c00[i] +
            dx * (1 - dy) * c01[i] +
            (1 - dx) * dy * c10[i] +
            dx * dy * c11[i]);
    }
    
    return color;
}

// 利用图像给点云着色
void colorPointCloudByImage(
    const PointCloud::Ptr& cloud,           // 输入点云
    const Eigen::Matrix4d& T_lidar2cam,     // 点云到相机的变换矩阵
    const Eigen::Matrix3d& K,                // 相机内参矩阵
    const cv::Mat& dist_coeffs,              // 相机畸变系数
    const cv::Mat& image,                    // 输入图像
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_colored) {
  if (cloud_colored == nullptr) {
    cloud_colored = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  }
  cloud_colored->points.resize(cloud->points.size());
  cloud_colored->height = cloud->height;
  cloud_colored->width = cloud->width;

  for (int i =0; i < cloud->points.size();i++) {
    const auto& xyzi = cloud->points[i];
    auto& xyzrgb = cloud_colored->points[i];
    xyzrgb.x = xyzi.x;
    xyzrgb.y = xyzi.y;
    xyzrgb.z = xyzi.z;
  }

  if (image.empty()) {
    return;
  }
  
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        K(0, 0), K(0, 1), K(0, 2),
        K(1, 0), K(1, 1), K(1, 2),
        K(2, 0), K(2, 1), K(2, 2));

  cv::Mat rt = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat rvec = rt.rowRange(0, 3).colRange(0, 3);
  cv::Mat tvec = rt.rowRange(0, 3).colRange(3, 4);

  // 遍历点云中的每个点
  cv::Mat image_undist;
  cv::undistort(image, image_undist, camera_matrix, dist_coeffs);
  std::vector<cv::Point3d> src_points;

  for (int i = 0; i < cloud->points.size(); i++) {

    auto point = cloud->points[i];
    // 先转到相机坐标系下
    Eigen::Vector3d point_cam = T_lidar2cam.block<3, 3>(0, 0) * Eigen::Vector3d(point.x, point.y, point.z) + T_lidar2cam.block<3, 1>(0, 3);
    if (point_cam.z() <= 0) {  
        continue; // 跳过z轴小于等于0的点
    }
    // 加fov限制避免fov边缘像素投影到图像中变成杂点
    src_points.clear();
    src_points.push_back(cv::Point3d(point_cam[0], point_cam[1], point_cam[2]));        
    std::vector<cv::Point2d> dst_points;
    cv::projectPoints(src_points, rvec, tvec, camera_matrix, dist_coeffs, dst_points);
    cv::Point2d pixel = dst_points[0];

    if (pixel.x >= 0 && pixel.x < image.cols && pixel.y >= 0 && pixel.y < image.rows) {
        // 获取颜色（双线性插值）
        cv::Vec3b color = getBilinearInterpolatedColor(image, pixel);
        
        // 设置点云颜色
        cloud_colored->points[i].r = color[2];
        cloud_colored->points[i].g = color[1];
        cloud_colored->points[i].b = color[0];
    } else {
        // 超出图像范围的点设为黑色
        cloud_colored->points[i].r = 0;
        cloud_colored->points[i].g = 0;
        cloud_colored->points[i].b = 0;
    }
  }
}


int interPcd() {
    std::string pcd_file1 = "/media/yao/t7/A04/highway/record_20250729_150612_dump/20250729_150612_C.record.00001/top/1753772846.700.pcd";
    std::string pcd_file2 = "/media/yao/t7/A04/highway/record_20250729_150612_dump/20250729_150612_C.record.00001/top/1753772846.800.pcd";

    // position1"position": [
        // 2964.6792946592195,
        // 939.5545828839404,
        // -1.2112158075568875
    
    // position2
        // 2966.979507724687,
        // 939.3599986768057,
        // -1.237169441880671
    double det = 0.55;
    double diff_x = (2966.979507724687 - 2964.6792946592195) * det;
    double diff_y = (939.3599986768057 - 939.5545828839404) * det;
    double diff_z = (-1.237169441880671 + 1.2112158075568875) * det;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file1, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file2, *cloud2);

    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].x -= diff_x;
        cloud->points[i].y -= diff_y;
        cloud->points[i].z -= diff_z;
    }


    // save
    std::string save_path1 = "/media/yao/t7/A04/highway/record_20250729_150612_dump/20250729_150612_C.record.00001/top/1753772846.755.pcd";
    pcl::io::savePCDFile<pcl::PointXYZI>(save_path1, *cloud);

}






int main() {
    // 示例数据 - 需要替换为你的实际数据
    // interPcd();
    // return 0;
    // 1. 创建或加载点云
    PointCloud::Ptr cloud(new PointCloud);
    std::string pcd_file = "/media/yao/t7/A04/highway/record_20250729_150612_dump/20250729_150612_C.record.00001/top/1753772846.600.pcd";
    std::string img_file = "/media/yao/t7/A04/highway/record_20250729_150612_dump/20250729_150612_C.record.00001/top/1753772846.687.jpg";
    // std::string img_file = "/media/yao/t7/A04/highway/record_20250729_150302_dump/20250729_150302_C.record.00001/top/1753772649.587.jpg";

    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1) {
        std::cerr << "Failed to load PCD file!" << std::endl;
        return -1;
    }

    // 2. 相机外参: 相机到车体的变换矩阵 (4x4)
    Eigen::Matrix4d T_camera2lidar = Eigen::Matrix4d::Identity();
    T_camera2lidar << -2.6110081465080034e-02, -4.5484681048058438e-02, 9.9862375669510650e-01, 7.0000000000000007e-02,
       -9.9964557368155538e-01, 6.3796411690987967e-03, -2.5846222111392127e-02, -2.9999999999999999e-02,
       -5.1952540616182503e-03, -9.9894466511839919e-01, -4.5635133030651941e-02, -9.0000000000000011e-02,
                      0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Isometry3d cam2lidar = Eigen::Isometry3d(T_camera2lidar);
    Eigen::Isometry3d lidar2cam = cam2lidar.inverse();          
    Eigen::Matrix4d T_lidar2cam;
    T_lidar2cam = lidar2cam.matrix();


    // 
    Eigen::Matrix4d T_camera2veh = Eigen::Matrix4d::Identity();
    T_camera2veh << -0.02407827477735782, -0.06217649697931055, 0.9977746839377756, 6.3650000000000002e+00,
       -0.9996607517841355, 0.01141177306837478, -0.02341266276639763, 0.,
       -0.009930660910694731, -0.9979999271837737, -0.06243017952127879,
       2.6050000000000000e+00,
                      0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Isometry3d cam2veh = Eigen::Isometry3d(T_camera2veh);

    // 
    // 分步初始化 lidar2veh 变量
    // 1. 定义四元数（这里假设示例值，需替换为实际值）
    Eigen::Quaterniond q_lidar2veh(0.999957, 0.00124991, 0.00910862 , 0.00110702); // w, x, y, z
    // 2. 定义平移向量（这里假设示例值，需替换为实际值）
    Eigen::Vector3d t_lidar2veh(6.195, 0.0, 2.705); // x, y, z
    // 3. 初始化 Isometry3d 对象
    Eigen::Isometry3d lidar2veh = Eigen::Isometry3d::Identity();
    // 设置旋转部分
    lidar2veh.linear() = q_lidar2veh.toRotationMatrix();
    // 设置平移部分
    lidar2veh.translation() = t_lidar2veh;


    Eigen::Isometry3d lidar2cam_2 = cam2veh.inverse() *  lidar2veh;

    // lidar2cam 转换到欧拉角
    Eigen::Vector3d euler_angles = lidar2cam.linear().eulerAngles(2, 1, 0);
    std::cout << "lidar2cam 欧拉角: " << euler_angles.transpose()* 180.0 / M_PI << std::endl;
    std::cout << "lidar2cam 平移: " << lidar2cam.translation().transpose() << std::endl;

    //lidar2cam_2 转换到欧拉角
    Eigen::Vector3d euler_angles_2 = lidar2cam_2.linear().eulerAngles(2, 1, 0);
    std::cout << "lidar2cam_com 欧拉角: " << euler_angles_2.transpose() * 180.0 / M_PI << std::endl;
    std::cout << "lidar2cam_com 平移: " << lidar2cam_2.translation().transpose() << std::endl;
       
    T_lidar2cam = lidar2cam_2.matrix();

    // 3. 相机内参矩阵 (3x3)
    Eigen::Matrix3d K;
    K << 3.7900999999999999e+03/2., 0., 1.9352000000000000e+03/2.,
         0., 3.7883000000000002e+03/2., 1.0685999999999999e+03/2., 
         0., 0., 1.;
    
    // 4. 畸变系数 (k1, k2, p1, p2, k3)
    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -3.1950000000000001e-01, -3.9560000000000001e-01,
       1.1000000000000001e-03, 8.2842999999999996e-06,
       7.9369999999999996e-01 );
    
    // 5. 创建或加载图像
    cv::Mat image = cv::imread(img_file);
    cv::Mat raw_img = image.clone();
    
    // 投影点云到图像
    projectPointCloudToImage(cloud, T_lidar2cam, K, dist_coeffs, image);
    // 显示结果
    cv::imwrite("/home/yao/myproject/code_module/calib/workspace/a04_proj_trigger.png", image);
    // cv::resize(image, image, cv::Size(1920, 1080));
    // cv::imshow("Point Cloud Projection", image);

    // cv::waitKey(0);

    // 点云着色
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorPointCloudByImage(cloud, T_lidar2cam, K, dist_coeffs, raw_img, cloud_colored);

    pcl::io::savePCDFile("/home/yao/myproject/code_module/calib/workspace/a04_colored.pcd", *cloud_colored);

    
    return 0;
}