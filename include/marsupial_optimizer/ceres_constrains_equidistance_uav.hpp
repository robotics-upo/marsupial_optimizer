#ifndef CERES_CONSTRAINS_EQUIDISTANCE_UAV_HPP
#define CERES_CONSTRAINS_EQUIDISTANCE_UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class EquiDistanceFunctorUAV 
{

  public:
    EquiDistanceFunctorUAV(double weight_factor, double initial_distance_uav): wf_(weight_factor), int_d_uav(initial_distance_uav) {}

    template <typename T>
    bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
    {
      T arg_d_pos_uav, d_pos_uav;
      
      arg_d_pos_uav = pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2);
      if (arg_d_pos_uav < 0.0001 && arg_d_pos_uav > -0.0001)
        d_pos_uav = T{0.0};
      else  
        d_pos_uav = sqrt(arg_d_pos_uav);

      residual[0] = wf_ * 10.0 * ( exp(2.0*(d_pos_uav - int_d_uav)));

      // std::cout<< "Equi-Distance UAV" << std::endl;
      // std::cout<< "d_pos_uav= " << d_pos_uav <<" , int_d_uav= " << int_d_uav << std::endl;
      // std::cout<< "residual[0]= " << residual[0] << std::endl;

      return true;
    
    }

    double wf_, int_d_uav;

  private:

};

#endif