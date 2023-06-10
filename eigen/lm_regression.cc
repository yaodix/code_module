#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include <eigen3/unsupported/Eigen/NonLinearOptimization>
//#include <unsupported/Eigen/LevenbergMarquardt>

// LM minimize for the model y = a x + b
using Point2DVector =  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > ;

Point2DVector GeneratePoints();

// anther example: 
// https://stackoverflow.com/questions/18509228/how-to-use-the-eigen-unsupported-levenberg-marquardt-implementation?rq=1

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  // Information that tells the caller the numeric type (eg. double) and size (input / output dim)
  typedef _Scalar Scalar;
  enum {// Required by numerical differentiation module
      InputsAtCompileTime = NX,
      ValuesAtCompileTime = NY
  };
   // Tell the caller the matrix sizes associated with the input, output, and jacobian
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  // Local copy of the number of inputs
  int m_inputs, m_values;

  // Two constructors:
  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  // Get methods for users to determine function input and output dimensions
  int inputs() const { return m_inputs; }
  int values() const { return m_values; }
};


struct MyFunctor : Functor<double>
{
  // fvec: 
  int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
  {
    // "a" in the model is x(0), and "b" is x(1)
    for(unsigned int i = 0; i < this->Points.size(); ++i)  {
        fvec(i) = this->Points[i](1) - (x(0) * this->Points[i](0) + x(1));
    }
    std::cout<< "optimizing..." << x(0) << " " << x(1) <<std::endl;
    return 0;
  }
  // target(observations)
  Point2DVector Points;
  
  int inputs() const { return 2; } // There are two parameters of the model
  int values() const { return this->Points.size(); } // The number of observations
};

// https://en.wikipedia.org/wiki/Test_functions_for_optimization
// Booth Function
// Implement f(x,y) = (x + 2*y -7)^2 + (2*x + y - 5)^2
struct BoothFunctor : Functor<double>
{
  // Simple constructor
  BoothFunctor(): Functor<double>(2,2) {}

  // Implementation of the objective function
  int operator()(const Eigen::VectorXd &z, Eigen::VectorXd &fvec) const {
    double x = z(0);   double y = z(1);
    /*
     * Evaluate the Booth function.
     * Important: LevenbergMarquardt is designed to work with objective functions that are a sum
     * of squared terms. The algorithm takes this into account: do not do it yourself.
     * In other words: objFun = sum(fvec(i)^2)
     */
    fvec(0) = x + 2*y - 7;
    fvec(1) = 2*x + y - 5;
    return 0;
  }
};

struct MyFunctorNumericalDiff : Eigen::NumericalDiff<MyFunctor> {};

Point2DVector GeneratePoints(const unsigned int numberOfPoints)
{
  Point2DVector points;
  // Model y = 2*x + 5 with some noise (meaning that the resulting minimization should be about (2,5)
  std::cout << "target line is y = 2*x + 5" << std::endl;
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
      double x = static_cast<double>(i);
      Eigen::Vector2d point;
      point(0) = x;
      point(1) = 2.0 * x + 5.0 + drand48()/10.0;
      points.push_back(point);
    }

  return points;
}

int main(int , char *[])
{
  unsigned int numberOfPoints = 50;
  Point2DVector points = GeneratePoints(numberOfPoints);

  Eigen::VectorXd x(2);
  x.fill(1.0f);

  MyFunctorNumericalDiff functor;
  functor.Points = points;
  Eigen::LevenbergMarquardt<MyFunctorNumericalDiff> lm(functor);

  Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
  std::cout << "status: " << status << std::endl;

  //std::cout << "info: " << lm.info() << std::endl;

  std::cout << "x that minimizes the function: " << std::endl << x << std::endl;

  return 0;
}