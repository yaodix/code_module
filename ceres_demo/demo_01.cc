// ceres 入门示例

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// 第一部分：构建代价函数,重载（）符号，仿函数的小技巧
// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
// 定义cost function的调用函数接口，是一个仿函数functor
// ceres::CostFunction成本函数通过这个函数来计算残差Residual
struct CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {   // 待优化变量x, residual:代价函数的输出
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // 寻找参数x的初始值，设置为5
    // The variable to solve for with its initial value. It will be
    // mutated(修改) in place by the solver.
    double x = 0.5;
    const double initial_x = x;

    // 第二部分： 构建求解问题
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    // 定义cost function，此处是使用自动求导
    CostFunction *cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);  // 误差类型，输出维度，输入维度
    problem.AddResidualBlock(cost_function, NULL, &x);    //  添加残差块，具体包括CostFunction, LossFunction, ParameterBlock

    // 第三部分，配置并运行求解器
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}
