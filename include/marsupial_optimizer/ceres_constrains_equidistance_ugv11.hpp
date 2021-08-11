#ifndef CERES_CONSTRAINS_EQUIDISTANCE__UGV_HPP_
#define CERES_CONSTRAINS_EQUIDISTANCE__UGV_HPP_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class EquiDistanceFunctorUGV 
{

  public:
    EquiDistanceFunctorUGV(double weight_factor, double initial_distance_ugv): wf_(weight_factor), int_d_ugv(initial_distance_ugv) {}

    template <typename T>
    bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
    {  
      T arg_d_pos_ugv, d_pos_ugv, arg_D, _D ;
      
      //Get distance between two consecutive poses
      arg_d_pos_ugv= (pow(statePos1[1]-statePos2[1],2)) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2);
      if(arg_d_pos_ugv< 0.0001 && arg_d_pos_ugv > -0.0001)
        d_pos_ugv = T{0.0};
      else
        d_pos_ugv = sqrt(arg_d_pos_ugv);

      // arg_D = (d_pos_ugv - int_d_ugv) * (d_pos_ugv - int_d_ugv);
      // if(arg_D< 0.0001 && arg_D > -0.0001)
      //   _D = T{0.0};
      // else
      //   _D = sqrt(arg_D);
      
      if (d_pos_ugv < 0.001)
         residual[0] = T{0.0};
      else
         residual[0] = wf_ * 10.0 * ( exp(2.0*(d_pos_ugv - int_d_ugv)) );

        // residual[0] = wf_ * 10.0 * (d_pos_ugv - int_d_ugv);

      // std::cout<< "Equi-Distance UGV" << std::endl;
      // std::cout<< "d_pos_ugv= " << d_pos_ugv <<" , residual[0]= " << residual[0] << std::endl;

      return true;

    }

    double wf_, int_d_ugv;

  private:

};

#endif