/**
@brief class to rectify stereo images
@author Shane Yuan
@date Apr 26, 2018
*/

#ifndef __DEPTH_ESTIMATION_STEREO_RECTIFY__
#define __DEPTH_ESTIMATION_STEREO_RECTIFY__

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>



void OpenCV_3_2_0stereoRectify(cv::InputArray _cameraMatrix1, cv::InputArray _distCoeffs1,
                        cv::InputArray _cameraMatrix2, cv::InputArray _distCoeffs2,
                        cv::Size imageSize, cv::InputArray _Rmat, cv::InputArray _Tmat,
                        cv::OutputArray _Rmat1, cv::OutputArray _Rmat2,
                        cv::OutputArray _Pmat1, cv::OutputArray _Pmat2,
                        cv::OutputArray _Qmat, int flags,
                        double alpha, cv::Size newImageSize,
                        cv::Rect* validPixROI1, cv::Rect* validPixROI2 );	


#endif
