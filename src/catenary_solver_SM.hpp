#ifndef __CATENARY_SOLVER_HPP__
#define __CATENARY_SOLVER_HPP__

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "Eigen/Core"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class LengthCostFunctor
{
 public:
    LengthCostFunctor(double xB, double yB, double length)
    {
      _xB = xB;
      _yB = yB; 
      _l = length;
      _k = (sqrt(_l*_l - _yB*_yB) )/ (_xB);
      printf("values: l=[%f] , yB=[%f] , xB=[%f] , k = [%f]\n", _l, _yB, _xB, _k);
    }

    ~LengthCostFunctor(void) 
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
    double _xB, _yB;
    // Catenary length
    double _l;
    // Constant from equation length cable
    double _k;
};


class PointCostFunctor
{
 public:
    PointCostFunctor(double xB, double yB, double a)
    {
      _xB = xB;
      _yB = yB; 
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
      
      r[0] = _a * cosh(((_xB)-x0)/_a) - _a * cosh(x0/_a) - (_yB);
      r[1] = _a * cosh(((_xB)-x0)/_a) - (_yB) + y0;

      return true;
    }

  private:

    // Point to be evaluated
    double _xB, _yB;
    // Constant from LengthCostFunctor
    double _a;
};

struct points_catenary_2D
{
    double x;
    double y;
};

class CatenarySolver
{
  private:

    // Optimizer parameters
    int _max_num_iterations, _num_point_per_unit_length;

  public:

    CatenarySolver(void) 
    {
        google::InitGoogleLogging("CatenaryCeresSolver");
        _max_num_iterations = 50;
        _num_point_per_unit_length = 10;
    }

    ~CatenarySolver(void)
    {

    } 

    std::vector <points_catenary_2D> v_points_catenary_2D;
    bool x_const, y_const, z_const;
    double num_point_catenary;

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

    bool setNumPointsPerUnitLength(int n)
    {
        if(n>0)
        {
            _num_point_per_unit_length = n;
            return true;
        } 
        else
            return false;
    }

    inline void getNumberPointsCatenary(double _length){num_point_catenary = ceil( (double)_num_point_per_unit_length * _length);}  

    inline double evaluteCatenaryChain(double _x, double _xc, double _yc, double _a){return (_a * cosh((_x- _xc)/_a)+ (_yc-_a)); }

    void integrateCatenaryChain(double _x1, double _y1, double _x2, double _y2, double _xc, double _yc, double _a)
    {
      double _xB = sqrt(pow(_x2 - _x1,2)+pow(_y2 - _y1,2));
      double x_value = 0.0;
      double y_value = 0.0; 
      double x_step = (_xB)/num_point_catenary;
      points_catenary_2D point_catenary_2D;
      v_points_catenary_2D.clear();
      
      for (int i=0 ; i < num_point_catenary +1 ; i ++)
      {
          y_value = evaluteCatenaryChain(x_value, _xc, _yc, _a);
          point_catenary_2D.x = x_value;
          point_catenary_2D.y = y_value;
          v_points_catenary_2D.push_back(point_catenary_2D);
          x_value = x_value + x_step;
      }
    }

    void convert2DTo3D(double _x1, double _y1, double _z1, double _x2, double _y2, double _z2, std::vector<Eigen::Vector3d> &_vector3D)
    {
      if (x_const && y_const){
        double _dist_3d = sqrt(pow(_x2-_x1,2)+pow(_y2-_y1,2)+pow(_z2-_z1,2)); 
        getPointCatenaryStraight(_x1, _y1, _z1, _dist_3d, _vector3D);
      }
      else{
        double tetha = atan(fabs(_y2-_y1)/fabs(_x2-_x1));
        double Zmin = _z1;
        int pos_lower_point;
        Eigen::Vector3d point_cat3D;

        double _direc_x, _direc_y;
        if (_x2 > _x1)
          _direc_x = 1;
        if (_x2 < _x1)
            _direc_x = -1;
        if (_x2 == _x1){
            _direc_x = 0;    
            printf("\nChain perperdicular with Axes X\n");
        }
        if (_y2 > _y1)
            _direc_y = 1;
        if (_y2 < _y1)
            _direc_y = -1;
        if (_y2 == _y1){
            _direc_y = 0;    
            printf("\nChain perperdicular with Axes Y\n");
        }

        for(int i=0; i < v_points_catenary_2D.size(); i++)
        {
            point_cat3D.z() = v_points_catenary_2D[i].y;
            point_cat3D.x() = _x1 + _direc_x* cos(tetha) * v_points_catenary_2D[i].x;
            point_cat3D.y() = _y1 + _direc_y* sin(tetha) * v_points_catenary_2D[i].x;
            _vector3D.push_back(point_cat3D);
            if (Zmin > point_cat3D.z())
            {
                Zmin = point_cat3D.z();
                pos_lower_point = i;
            }
            printf("Points Catenary X_= %f , Y_= %f , Z = %f\n", _vector3D[i].x(),_vector3D[i].y(),_vector3D[i].z());
        }
        // printf("\nFOUND IT!! The lowest point Catenary 3D X_= %f , Y_= %f , Z = %f\n", _vector3D[pos_lower_point].x(), _vector3D[pos_lower_point].y(), _vector3D[pos_lower_point].z());
      }
    }

    void getPointCatenaryStraight(double _x1, double _y1, double _z1, double _dis3D, std::vector<Eigen::Vector3d> &_v_p)
    {

      double _step = _dis3D / (double) num_point_catenary;

      for(int i=0; i < num_point_catenary +1; i++)
      {
          Eigen::Vector3d _p;

          _p.x() = _x1;
          _p.y() = _y1;
          _p.z() = _z1 + _step* (double)i;    
          _v_p.push_back(_p);
      }
    }

    void checkStateCatenary(double _x1, double _y1, double _z1, double _x2, double _y2, double _z2)
    {
      if ( fabs(_x1 - _x2) < 0.000001)
          x_const = true;
      if ( fabs(_y1 - _y2) < 0.000001)
          y_const = true;
      if ( fabs(_z1 - _z2) < 0.000001)
          z_const = true;
      if (x_const && y_const){
          // printf("Straight Catenary: Axes X and Y fix for start and goal point\n");
      }
    }

    bool solve(double x1, double y1, double z1, double x2, double y2, double z2, double length,
               double &a, double &x0, double &y0, std::vector<Eigen::Vector3d> &_vector3D)
    {
        double _xB = sqrt(pow(x2 - x1,2)+pow(y2 - y1,2));
        double _yB = z2 - z1;
        _vector3D.clear();
        x_const = y_const = z_const = 0;


        /***First Part: Get phi value from Length equation***/
        double phi[1];
        phi[0] = 1.0;
        
        // Build the problem.
        Problem prob1;
        CostFunction* cf = new ceres::AutoDiffCostFunction<LengthCostFunctor, 1, 1>( new LengthCostFunctor(_xB, _yB, length) );
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
        double _a = _xB/(2.0 * phi[0]);  
        std::cout << std::endl <<" _a: " << _a << std::endl;


        /***Second Part: Get x0 and y0 values from Points equations***/
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
        CostFunction* cf1 = new ceres::AutoDiffCostFunction<PointCostFunctor, 2, 2>( new PointCostFunctor(_xB, _yB, _a) );
        prob2.AddResidualBlock(cf1, NULL, x);

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

        std::cout << std::endl <<"a_: " << a << " , x0: " << x0 << " , y0: "<< y0 << std::endl;

        double _h = fabs(y0)- a;
        double _xc = x0;
        double _yc = z1 - _h;  

        std::cout <<"h: " << _h << " , xc: " << _xc << " , yc: "<< _yc << std::endl;

        /***Thirs Part: Get points Catenary***/

        getNumberPointsCatenary(length);
        checkStateCatenary(x1, y1, z1, x2, y2, z2);
        integrateCatenaryChain(x1, y1, x2, y2,_xc, _yc, a);
        convert2DTo3D(x1, y1, z1, x2, y2, z2, _vector3D);


        return true; 
    }
};



#endif


