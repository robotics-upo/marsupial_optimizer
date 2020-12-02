#ifndef __CATENARY_SOLVER_FC_HPP__
#define __CATENARY_SOLVER_FC_HPP__

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class PointCostFunctor
{
 public:
    PointCostFunctor(double x, double y)
    {
      _x = x;
      _y = y;
    }

    ~PointCostFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* p, T* r) const 
    {
      T a  = p[0];
      T x0 = p[1];
      T y0 = p[2];
      
      r[0] = y0 - _y + a * cosh((_x - x0)/a);

      return true;
    }

  private:

    // Point to be evaluated
    double _x, _y;
};

class LengthCostFunctor
{
 public:
    LengthCostFunctor(double length, double x1, double x2)
    {
      _x1 = x1;
      _x2 = x2;
      _l = length;
    }

    ~LengthCostFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* p, T* r) const 
    {
      T a  = p[0];
      T x0 = p[1];
      T y0 = p[2];
      
      r[0] = _l - a *sinh(abs(_x1-x0)/a) - a * sinh(abs(_x2-x0)/a);
      // r[0] = _l - _a *sinh((x0)/_a) - _a * sinh(((_x2-_x1)-x0)/_a);


      return true;
    }

  private:

    // Points to be evaluated
    double _x1, _x2;

    // Catenary length
    double _l;
};

class CatenarySolver
{
  private:

    // Optimizer parameters
    int _max_num_iterations;

  public:

    CatenarySolver(void) 
    {
        google::InitGoogleLogging("LidarRegSolver");
        _max_num_iterations = 50;
    }

    ~CatenarySolver(void)
    {

    } 

    bool setMaxNumIterations(int n)
    {
        if(n>0)
        {
            _max_num_iterations = n;
            return true;
        } 
        else
            return false;
    }

    bool solve(double x1, double y1, double x2, double y2, double length,
               double &a, double &x0, double &y0)
    {
        // Initial solution
        double x[3];
        x[0] = 1.0 ; x[1] = 0.0; x[2] = 0.0; 

        // Build the problem.
        Problem problem;

        // Set up a cost funtion per point into the cloud
        CostFunction* cf1 = new ceres::AutoDiffCostFunction<PointCostFunctor, 1, 3>( new PointCostFunctor(x1, y1) );
        CostFunction* cf2 = new ceres::AutoDiffCostFunction<PointCostFunctor, 1, 3>( new PointCostFunctor(x2, y2) );
        CostFunction* cf3 = new ceres::AutoDiffCostFunction<LengthCostFunctor, 1, 3>( new LengthCostFunctor(length, x1, x2) );
        problem.AddResidualBlock(cf1, NULL, x);
        problem.AddResidualBlock(cf2, NULL, x);
        problem.AddResidualBlock(cf3, NULL, x);
        

        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = _max_num_iterations;
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        // Some debug information
        std::cout << summary.BriefReport() << "\n";

        // Get the solution
        a = x[0]; x0 = x[1]; y0 = x[2]; 

        return true; 
    }
};



#endif


