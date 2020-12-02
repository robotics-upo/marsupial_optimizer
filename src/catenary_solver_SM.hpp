#ifndef __CATENARY_SOLVER_HPP__
#define __CATENARY_SOLVER_HPP__

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class PhiCostFunctor
{
 public:
    PhiCostFunctor(double x1, double y1, double x2, double y2, double length)
    {
      _x1 = x1;
      _y1 = y1;
      _x2 = x2;
      _y2 = y2;
      _l = length;
      _k = (sqrt(_l*_l - (_y2-_y1)*(_y2-_y1)) )/ (_x2 - _x1);
      printf("value _k = [%f]\n",_k);
    }

    ~PhiCostFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* p, T* r) const 
    {
      T _phi  = p[0];
      r[0] = sinh(_phi) -  _k * _phi;

      return true;
    }

  private:

    // Point to be evaluated
    double _x1, _y1, _x2, _y2;
    // Catenary length
    double _l;
    // Constant from equation length cable
    double _k;
};


class PointCostFunctor
{
 public:
    PointCostFunctor(double x1, double y1, double x2, double y2, double a)
    {
      _x1 = x1;
      _y1 = y1;
      _x2 = x2;
      _y2 = y2;
      _a = a;
    }

    ~PointCostFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* p, T* r) const 
    {
      T x0 = p[0];
      T y0 = p[1];
      
      // r[0] = y0 - _y + _a * cosh((_x - x0)/ _a);
      r[0] = _a * cosh(((_x2-_x1)-x0)/_a) - _a * cosh(x0/_a) - (_y2-_y1);
      r[1] = _a * cosh(((_x2-_x1)-x0)/_a) - (_y2-_y1) + y0;

      // (c_value*cosh(((XB)-x)/c_value) - c_value*cosh((x)/c_value) - YB);
      // c_value*cosh(((XB)-bs_X0)/c_value) - (YB) + y);

      return true;
    }

  private:

    // Point to be evaluated
    double _x1, _y1, _x2, _y2;
    // Constant from PhiCostFunctor
    double _a;
};

class LengthCostFunctor
{
 public:
    LengthCostFunctor(double length, double x1, double x2, double a)
    {
      _x1 = x1;
      _x2 = x2;
      _l = length;
      _a = a;
    }

    ~LengthCostFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* p, T* r) const 
    {
      T x0 = p[0];
      T y0 = p[1];
      
      r[0] = _l - _a *sinh((x0)/_a) - _a * sinh(((_x2-_x1)-x0)/_a);
      // r[0] = _l - a *sinh(abs(_x1-x0)/a) - a * sinh(abs(_x2-x0)/a); //FC

      return true;
    }

  private:

    // Points to be evaluated
    double _x1, _x2;
    // Catenary length
    double _l;
    // Constant from PhiCostFunctor
    double _a;
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
       
        /***First Part***/
        double phi[1];
        phi[0] = 1.0;
        
        // Build the problem.
        Problem prob1;
        CostFunction* cf = new ceres::AutoDiffCostFunction<PhiCostFunctor, 1, 1>( new PhiCostFunctor(x1, y1, x2, y2, length) );
        prob1.AddResidualBlock(cf, NULL, phi);

        // Run the solver!
        Solver::Options options1;
        options1.minimizer_progress_to_stdout = true;
        options1.max_num_iterations = _max_num_iterations;
        Solver::Summary summary1;
        Solve(options1, &prob1, &summary1);

        // Some debug information
        std::cout << summary1.BriefReport() << "\n";

        std::cout << std::endl <<"phi: " << phi[0] << std::endl;

        // Get the solution
        double _a = (x2-x1)/(2.0 * phi[0]);  

        std::cout << std::endl <<" _a: " << _a << std::endl;


        /***Second Part***/

         // Initial solution
        double x[2];
        x[0] = (x2-x1);  
        if (y2 >= y1)
          x[1] = y1;
        else
          x[1] = y2;
        
        // Build the problem.
        Problem prob2;

        // Set up a cost funtion per point into the cloud
        CostFunction* cf1 = new ceres::AutoDiffCostFunction<PointCostFunctor, 2, 2>( new PointCostFunctor(x1, y1, x2, y2,_a) );
        // CostFunction* cf2 = new ceres::AutoDiffCostFunction<PointCostFunctor, 1, 2>( new PointCostFunctor(x2, y2,_a) );
        // CostFunction* cf3 = new ceres::AutoDiffCostFunction<LengthCostFunctor, 1, 2>( new LengthCostFunctor(length, x1, x2,_a) );
        prob2.AddResidualBlock(cf1, NULL, x);
        // prob2.AddResidualBlock(cf2, NULL, x);
        // prob2.AddResidualBlock(cf3, NULL, x);
        

        // Run the solver!
        Solver::Options options2;
        options2.minimizer_progress_to_stdout = true;
        options2.max_num_iterations = _max_num_iterations;
        Solver::Summary summary2;
        Solve(options2, &prob2, &summary2);

        // Some debug information
        std::cout << summary2.BriefReport() << "\n";

        // Get the solution
        a = _a; x0 = x[0]; y0 = x[1]; 

        return true; 
    }
};



#endif


