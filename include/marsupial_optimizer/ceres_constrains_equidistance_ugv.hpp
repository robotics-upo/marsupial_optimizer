#ifndef CERES_CONSTRAINS_EQUIDISTANCE__UGV_HPP_
#define CERES_CONSTRAINS_EQUIDISTANCE__UGV_HPP_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class EquiDistanceFunctorUGV : public SizedCostFunction<1, 4, 4> 
{
  public:
    EquiDistanceFunctorUGV(double weight_factor, double initial_distance_ugv)
    :wf_(weight_factor), int_d_ugv(initial_distance_ugv)
    {

    }

    virtual ~EquiDistanceFunctorUGV(void) 
    {

    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

      double a0 = parameters[0][0];
      double a1 = parameters[0][1];
      double a2 = parameters[0][2];
      double a3 = parameters[0][3];

      double b0 = parameters[1][0];
      double b1 = parameters[1][1];
      double b2 = parameters[1][2];
      double b3 = parameters[1][3];
      
      double arg_d_pos_ugv, d_pos_ugv;
      double factor_ = 10.0;
      
      //Get distance between two consecutive poses
      arg_d_pos_ugv= (pow(a1-b1,2)) + pow(a2-b2,2) + pow(a3-b3,2);
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
            jacobians[0][1] = (factor_*wf_*exp(2.0*sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3)) - 2.0*int_d_ugv)*(2.0*a1 - 2.0*b1))/sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3));
            jacobians[0][2] = (factor_*wf_*exp(2.0*sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3)) - 2.0*int_d_ugv)*(2.0*a2 - 2.0*b2))/sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3));
            jacobians[0][3] = (factor_*wf_*exp(2.0*sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3)) - 2.0*int_d_ugv)*(2.0*a3 - 2.0*b3))/sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3));
            jacobians[1][0] = 0.0;
            jacobians[1][1] = -(factor_*wf_*exp(2.0*sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3)) - 2.0*int_d_ugv)*(2.0*a1 - 2.0*b1))/sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3));
            jacobians[1][2] = -(factor_*wf_*exp(2.0*sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3)) - 2.0*int_d_ugv)*(2.0*a2 - 2.0*b2))/sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3));
            jacobians[1][3] = -(factor_*wf_*exp(2.0*sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3)) - 2.0*int_d_ugv)*(2.0*a3 - 2.0*b3))/sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2) + (a3 - b3)*(a3 - b3));
          }
        }

      return true;
    }

    double wf_, int_d_ugv;

  private:

};

#endif