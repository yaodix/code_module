/*
 参考： https://blog.csdn.net/shuoyueqishilove/article/details/81713142


*/
#ifndef _MYKALMAN_H
#define _MYKALMAN_H

#include <eigen3/Eigen/Dense>


class KalmanFilter
{
private:
    int stateSize; // state variable's dimenssion
    int measSize;  // measurement variable's dimession
    int uSize;     // control variables's dimenssion
    Eigen::VectorXd x;  // 系统状态
    Eigen::VectorXd z;  // k时刻测量值
    Eigen::MatrixXd A;  // 系统参数
    Eigen::MatrixXd B;  // 系统参数，控制矩阵
    Eigen::VectorXd u;
    Eigen::MatrixXd P;  // coveriance
    Eigen::MatrixXd H;  // 测量系统的参数
    Eigen::MatrixXd R;  // measurement noise covariance， 高斯分布的方差
    Eigen::MatrixXd Q;  // process noise covariance， 高斯分布的方差

public:
    KalmanFilter(int stateSize_, int measSize_,int uSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    void update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};

#endif