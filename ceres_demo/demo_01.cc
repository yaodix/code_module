// ceres 入门示例

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// 第一部分：构建代价函数,重载（）符号，仿函数的小技巧.必须使用模板类型，所有的输入参数和输出参数都要使用T类型。
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

// In some cases, its not possible to define a templated cost functor,
// for example when the evaluation of the residual involves a call to a 
// library function that you do not have control over. In such a situation, numerical differentiation can be used.
// not recommended.!!!
struct NumericDiffCostFunctor {
  bool operator()(const double* const x, double* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};
// CostFunction* cost_function =
//  new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>();
//problem.AddResidualBlock(cost_function, nullptr, &x);

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // 寻找参数x的初始值，设置为5
    // The variable to solve for with its initial value. It will be
    // mutated(修改) in place by the solver.
    double x = 5;
    const double initial_x = x;

    // 第二部分： 构建求解问题
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    // 定义cost function，此处是使用自动求导, <>: 误差类型，1个残差residual[0](输出维度)，参数1是1维x[0](输入维度)
    CostFunction *cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);  
    
    // 添加残差块，具体包括CostFunction, LossFunction, ParameterBlock
    // LossFunction可以为NULL，表示不使用LossFunction，param最多有10个
    problem.AddResidualBlock(cost_function, NULL, &x);

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

