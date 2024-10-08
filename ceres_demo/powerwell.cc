#include <vector>
#include "ceres/ceres.h"

//创建四个残差块，也就是四个仿函数
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = x[0] + 10.0 * x[1];
    residual[1] = sqrt(5.0) * (x[2] - x[3]);
    residual[2] = (x[1] - 2.0 * x[2]) * (x[1] - 2.0 * x[2]);
    residual[3] = sqrt(10.0) * (x[0] - x[3]) * (x[0] - x[3]);
    return true;
  }
};

int main(int argc, char** argv) {
  // 设置初始值
  double x[4] = {3.0, -1.0, 0.0, 1.0};

  // 加入problem
  ceres::Problem problem;
  problem.AddResidualBlock(
    new ceres::AutoDiffCostFunction<CostFunctor, 4, 4>(new CostFunctor),
      NULL, x);

  // 设置options
  ceres::Solver::Options options;
  options.max_num_iterations = 100;  // 迭代次数
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  std::cout << "Final x1 = " << x[0]
            << ", x2 = " << x[1]
            << ", x3 = " << x[2]
            << ", x4 = " << x[3]
            << "\n";

  return 0;
}
