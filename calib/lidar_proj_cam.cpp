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
    const PointCloud::Ptr& cloud,           // 输入点云
    const Eigen::Matrix4d& T_lidar2cam, // 相机到车体的外参
    const Eigen::Matrix3d& K,                // 相机内参矩阵
    const cv::Mat& dist_coeffs,              // 相机畸变系数
    cv::Mat& image,                          // 输入/输出图像
    int point_size = 2)                      // 点的大小
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

int main() {
    // 示例数据 - 需要替换为你的实际数据
    
    // 1. 创建或加载点云
    PointCloud::Ptr cloud(new PointCloud);
    std::string pcd_file = "/home/yao/myproject/code_module/calib/workspace/1740127535524983985.pcd";
    std::string img_file = "/home/yao/myproject/code_module/calib/workspace/1740127535537373241.jpg";

    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1) {
        std::cerr << "Failed to load PCD file!" << std::endl;
        return -1;
    }

    // 2. 相机外参: 相机到车体的变换矩阵 (4x4)
    Eigen::Matrix4d T_camera2lidar = Eigen::Matrix4d::Identity();
    T_camera2lidar << 0.0084219,  0.0239859, 0.9996768, 0.5480117235062314,
                     -0.9999303, -0.0080677, 0.0086176, -0.06893379271456662,
                      0.0082718, -0.9996797, 0.0239163, -0.45476354640306543,
                      0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Isometry3d cam2lidar = Eigen::Isometry3d(T_camera2lidar);
    Eigen::Isometry3d lidar2cam = cam2lidar.inverse();          
    Eigen::Matrix4d T_lidar2cam;
    T_lidar2cam = lidar2cam.matrix();

       
    // 3. 相机内参矩阵 (3x3)
    Eigen::Matrix3d K;
    K << 1904.6, 0., 1920.2,
         0., 1906.1, 1086.2, 
         0., 0., 1.;
    
    // 4. 畸变系数 (k1, k2, p1, p2, k3)
    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.3198, 0.112, 0.00021485, 6.4215e-06, -0.0187);
    
    // 5. 创建或加载图像
    cv::Mat image = cv::imread(img_file);
    
    // 投影点云到图像
    projectPointCloudToImage(cloud, T_lidar2cam, K, dist_coeffs, image);
    
    // 显示结果
    cv::imwrite("/home/yao/myproject/code_module/calib/workspace/1740127535537373241_pro.png", image);
    cv::resize(image, image, cv::Size(1920, 1080));
    // cv::imshow("Point Cloud Projection", image);

    // cv::waitKey(0);
    
    return 0;
}