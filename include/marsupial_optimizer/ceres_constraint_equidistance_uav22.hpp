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
    bool operator()(const T* const statePos1,const T* const statePos2,const T* const statePos3,const T* const statePos4,const T* const statePos5,T* residual) const 
    {
      T arg_d_pos_uav1, arg_d_pos_uav2, arg_d_pos_uav3, arg_d_pos_uav4; 
      T d_uav1, d_uav2, d_uav3, d_uav4;
      
      arg_d_pos_uav1 = pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2);
      arg_d_pos_uav2 = pow(statePos2[1]-statePos3[1],2) + pow(statePos2[2]-statePos3[2],2) + pow(statePos2[3]-statePos3[3],2);
      arg_d_pos_uav3 = pow(statePos3[1]-statePos4[1],2) + pow(statePos3[2]-statePos4[2],2) + pow(statePos3[3]-statePos4[3],2);
      arg_d_pos_uav4 = pow(statePos4[1]-statePos5[1],2) + pow(statePos4[2]-statePos5[2],2) + pow(statePos4[3]-statePos5[3],2);

      if (arg_d_pos_uav1 < 0.0001 && arg_d_pos_uav1 > -0.0001)
        d_uav1 = T{0.0};
      else  
        d_uav1 = sqrt(arg_d_pos_uav1);

      if (arg_d_pos_uav2 < 0.0001 && arg_d_pos_uav2 > -0.0001)
        d_uav2 = T{0.0};
      else  
        d_uav2 = sqrt(arg_d_pos_uav2);

      if (arg_d_pos_uav3 < 0.0001 && arg_d_pos_uav3 > -0.0001)
        d_uav3 = T{0.0};
      else  
        d_uav3 = sqrt(arg_d_pos_uav3);

      if (arg_d_pos_uav4 < 0.0001 && arg_d_pos_uav4 > -0.0001)
        d_uav4 = T{0.0};
      else  
        d_uav4 = sqrt(arg_d_pos_uav4);

      T d_ = (d_uav1 + d_uav2 + d_uav3 + d_uav4)/4.0;   

      residual[0] = wf_ * 100.0 * ( exp(2.0*(d_uav1 - d_)));
      residual[1] = wf_ * 100.0 * ( exp(2.0*(d_uav2 - d_)));
      residual[2] = wf_ * 100.0 * ( exp(2.0*(d_uav3 - d_)));
      residual[3] = wf_ * 100.0 * ( exp(2.0*(d_uav4 - d_)));

      // std::cout<< "Equi-Distance UAV" << std::endl;
      // std::cout<< "d_pos_uav= " << d_pos_uav <<" , int_d_uav= " << int_d_uav << std::endl;
      // std::cout<< "residual[0]= " << residual[0] << std::endl;

      return true;
    
    }

    double wf_, int_d_uav;

  private:

};

#endif