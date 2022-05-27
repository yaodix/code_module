#include <iostream>

#include "opevcv320_stereoRectify.h"
#include <opencv2/opencv.hpp>


int main() {

  cv::Mat _cameraMatrix1 = (cv::Mat_<double>(3,3) <<  1.9205999999999999e+03, 0., 9.7331579999999997e+02, 0.,
       1.9204000000000001e+03, 5.4895809999999994e+02, 0., 0., 1.);

  cv::Mat _cameraMatrix2 = (cv::Mat_<double>(3,3) << 1.9240999999999999e+03, 0., 9.7507119999999998e+02, 0.,
       1.9268000000000000e+03, 5.4393920000000003e+02, 0., 0., 1.);

  cv::Mat _distCoeffs1 = (cv::Mat_<double>(1,5) << -5.6979999999999997e-01, 3.5920000000000002e-01,
       2.7535999999999999e-04, -3.4212000000000001e-04,
       -1.6800000000000001e-01);

  cv::Mat _distCoeffs2 = (cv::Mat_<double>(1, 5) << -5.5910000000000004e-01, 3.0349999999999999e-01,
       3.5112000000000001e-04, 7.6303000000000005e-04,
       -6.8199999999999997e-02);

cv::Size imageSize(1920, 1080), newImageSize(1920, 1080);

  cv::Mat _Rmat = (cv::Mat_<double>(3, 3) <<9.9942266626431142e-01, 1.7719342400063249e-02,
       2.8988947239314264e-02, -1.6149087960255909e-02,
       9.9843550573031870e-01, -5.3532680253231248e-02,
       -2.9892158088474379e-02, 5.3033628972116262e-02,
       9.9814522644896764e-01);

  cv::Mat _Tmat = (cv::Mat_<double>(3, 1) << -8.4999999999999998e-01, 0., 0.);

cv::Mat _Rmat1, _Rmat2, _Pmat1, _Pmat2, _Qmat;
cv::Rect validPixROI1, validPixROI2;

cv::stereoRectify( _cameraMatrix1, _distCoeffs1,
                       _cameraMatrix2, _distCoeffs2,
                         imageSize, _Rmat, _Tmat,
                         _Rmat1, _Rmat2,
                       _Pmat1,  _Pmat2,
                       _Qmat, 0,
                        0.,  newImageSize,
                        &validPixROI1, &validPixROI2 );	

std::cout<< _Rmat1<< std::endl;
std::cout<< _Pmat1<< std::endl;

OpenCV_3_2_0stereoRectify( _cameraMatrix1, _distCoeffs1,
                       _cameraMatrix2, _distCoeffs2,
                         imageSize, _Rmat, _Tmat,
                         _Rmat1, _Rmat2,
                       _Pmat1,  _Pmat2,
                       _Qmat, 0,
                        0.,  newImageSize,
                        &validPixROI1, &validPixROI2 );	
std::cout<< _Rmat1<< std::endl;
std::cout<< _Pmat1<< std::endl;
return 0;
}