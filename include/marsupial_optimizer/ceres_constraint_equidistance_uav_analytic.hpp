#ifndef CERES_CONSTRAINT_EQUIDISTANCE_UAV_ANALYTIC_HPP
#define CERES_CONSTRAINT_EQUIDISTANCE_UAV_ANALYTIC_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class EquiDistanceFunctorUAVAnalytic : public SizedCostFunction<1, 4, 4> 
{
  public:
    EquiDistanceFunctorUAVAnalytic(double weight_factor, double initial_distance_ugv, double size)
    :wf_(weight_factor), i_d(initial_distance_ugv) , s_(size)
    {}

    virtual ~EquiDistanceFunctorUAVAnalytic(void) 
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
      double factor_ = 100.0;

      double d_pos = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
      
      // residuals[0] = wf_ * factor_ * ( exp(2.0*(d_pos - init_d)));

      // if (p1 > 0){ 
      //   if (jacobians != NULL && jacobians[0] != NULL){
      //     jacobians[0][0] = 0.0;
      //     jacobians[0][1] = (factor_*wf_*exp(2.0*d_pos - 2.0*init_d)*(2.0*x1 - 2.0*x2))/d_pos;
      //     jacobians[0][2] = (factor_*wf_*exp(2.0*d_pos - 2.0*init_d)*(2.0*y1 - 2.0*y2))/d_pos;
      //     jacobians[0][3] = (factor_*wf_*exp(2.0*d_pos - 2.0*init_d)*(2.0*z1 - 2.0*z2))/d_pos;
      //   }
      // }
      // if(p2 < (s_-1.0)){
      //   if (jacobians != NULL && jacobians[1] != NULL){
      //     jacobians[1][0] = 0.0;
      //     jacobians[1][1] = -(factor_*wf_*exp(2.0*d_pos - 2.0*init_d)*(2.0*x1 - 2.0*x2))/d_pos;
      //     jacobians[1][2] = -(factor_*wf_*exp(2.0*d_pos - 2.0*init_d)*(2.0*y1 - 2.0*y2))/d_pos;
      //     jacobians[1][3] = -(factor_*wf_*exp(2.0*d_pos - 2.0*init_d)*(2.0*z1 - 2.0*z2))/d_pos;
      //   }
      // }

      double max_value_residual = 100.0;
      double min_value_residual = 0.0;
      double m;
      double init_d = i_d*1.2;
      if (init_d > d_pos)
        m = 0.0;
      else
        m = (max_value_residual- min_value_residual)/( 2.0*init_d - init_d);

      residuals[0] = wf_ * m *(d_pos - init_d);

      if (p1 > 0){ 
        if (jacobians != NULL && jacobians[0] != NULL){
          jacobians[0][0] = 0.0;
          jacobians[0][1] = (m*wf_*(2.0*x1 - 2.0*x2))/(2.0*d_pos);
          jacobians[0][2] = (m*wf_*(2.0*y1 - 2.0*y2))/(2.0*d_pos);
          jacobians[0][3] = (m*wf_*(2.0*z1 - 2.0*z2))/(2.0*d_pos);
        }
      }
      if(p2 < (s_-1.0)){
        if (jacobians != NULL && jacobians[1] != NULL){
          jacobians[1][0] = 0.0;
          jacobians[1][1] = -(m*wf_*(2.0*x1 - 2.0*x2))/(2.0*d_pos);
          jacobians[1][2] = -(m*wf_*(2.0*y1 - 2.0*y2))/(2.0*d_pos);
          jacobians[1][3] = -(m*wf_*(2.0*z1 - 2.0*z2))/(2.0*d_pos);
        }
      }
      printf("EquiDistanceFunctorUAVAnalytic[%f-%f]: residuals[0] =%f , d_=[%f/%f]\n",p1,p2, residuals[0], d_pos, init_d);
      return true;
    }

    double wf_, i_d, s_;

  private:

};

#endif