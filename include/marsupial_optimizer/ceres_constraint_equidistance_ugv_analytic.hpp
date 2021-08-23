#ifndef CERES_CONSTRAINS_EQUIDISTANCE_UGV_ANALYTIC_HPP_
#define CERES_CONSTRAINS_EQUIDISTANCE_UGV_ANALYTIC_HPP_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class EquiDistanceFunctorUGVAnalytic : public SizedCostFunction<1, 4, 4> 
{
  public:
    EquiDistanceFunctorUGVAnalytic(double weight_factor, double initial_distance_ugv)
    :wf_(weight_factor), int_d_ugv(initial_distance_ugv)
    {}

    virtual ~EquiDistanceFunctorUGVAnalytic(void) 
    {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
      double p1 = parameters[0][0];
      double x1 = parameters[0][1];
      double y1 = parameters[0][2];
      double z1 = parameters[0][3];

      double p2 = parameters[1][0];
      double x2 = parameters[1][1];
      double y2 = parameters[1][2];
      double z2 = parameters[1][3];
      
      double arg_d_pos_ugv, d_pos_ugv;
      double factor_ = 10.0;
      
      //Get distance between two consecutive poses
      arg_d_pos_ugv= pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2);
      if(arg_d_pos_ugv< 0.0001 && arg_d_pos_ugv > -0.0001)
        d_pos_ugv = 0.0;
      else
        d_pos_ugv = sqrt(arg_d_pos_ugv);

      if (d_pos_ugv < 0.001)
         residuals[0] = 0.0;
      else
         residuals[0] = wf_ * factor_ * ( exp(2.0*(d_pos_ugv - int_d_ugv)) );

      if (jacobians != NULL && jacobians[0] != NULL) 
      {
        if (d_pos_ugv < 0.001){
          jacobians[0][0] = 0.0;
          jacobians[0][1] = 0.0;
          jacobians[0][2] = 0.0;
          jacobians[0][3] = 0.0;
          jacobians[1][0] = 0.0;
          jacobians[1][1] = 0.0;
          jacobians[1][2] = 0.0;
          jacobians[1][3] = 0.0;
        }
        else{
          jacobians[0][0] = 0.0;
          jacobians[0][1] = (factor_*wf_*exp(2.0*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)) - 2.0*int_d_ugv)*(2.0*x1 - 2.0*x2))/sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
          jacobians[0][2] = (factor_*wf_*exp(2.0*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)) - 2.0*int_d_ugv)*(2.0*y1 - 2.0*y2))/sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
          jacobians[0][3] = (factor_*wf_*exp(2.0*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)) - 2.0*int_d_ugv)*(2.0*z1 - 2.0*z2))/sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
          jacobians[1][0] = 0.0;
          jacobians[1][1] = -(factor_*wf_*exp(2.0*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)) - 2.0*int_d_ugv)*(2.0*x1 - 2.0*x2))/sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
          jacobians[1][2] = -(factor_*wf_*exp(2.0*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)) - 2.0*int_d_ugv)*(2.0*y1 - 2.0*y2))/sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
          jacobians[1][3] = -(factor_*wf_*exp(2.0*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)) - 2.0*int_d_ugv)*(2.0*z1 - 2.0*z2))/sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
        }
      }

      return true;
    }

    double wf_, int_d_ugv;

  private:

};

#endif