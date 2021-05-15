#ifndef CERES_CONTRAIN_EQUIDISTANCE__UGV_HPP_
#define CERES_CONTRAIN_EQUIDISTANCE__UGV_HPP_

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
      T d_pos_ugv = sqrt((pow(statePos1[1]-statePos2[1],2)) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2));
      
      if (d_pos_ugv < 0.01)
        residual[0] = T{0.0};
      else
        residual[0] = wf_ * ( exp(2.0*(d_pos_ugv - int_d_ugv)) );

      return true;

    }

    double wf_, int_d_ugv;

  private:

};

#endif