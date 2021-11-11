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

#include <iostream>
#include <fstream>
#include <string>

class EquiDistanceFunctorUAV 
{
  public:
    EquiDistanceFunctorUAV(double weight_factor, double initial_distance_uav)
    : wf_(weight_factor), i_d(initial_distance_uav) 
    {}

    template <typename T>
    bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
    {
      T arg_d_pos, d_pos, diff, dr_;
      double f_slope_ = 10.0;
      
      arg_d_pos = pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2);
      
      if (arg_d_pos < 0.0001 && arg_d_pos > -0.0001)
        d_pos = T{0.0};
      else  
        d_pos = sqrt(arg_d_pos);

      // diff = (d_pos - i_d);
      // if (diff < 0.0)
      //   residual[0] = T{0.0};
      // else
      //   residual[0] = wf_ * exp(f_slope_*(diff));


      T max_value_residual = T{100.0};
      T min_value_residual = T{0.0};
      T m;
      T init_d = i_d*T{1.2};
      if (init_d >= d_pos && d_pos > i_d){
        m = T{0.0};
      }
      else if(d_pos > init_d){
        dr_ = init_d; 
        m = (max_value_residual- min_value_residual)/( 2.0*dr_ - dr_);
      }
      else if(i_d > d_pos){
        dr_ = T{i_d};
        m = -(max_value_residual- min_value_residual)/( 2.0*dr_ - dr_);
      }

      residual[0] = wf_ * m *(d_pos - dr_);  

      // std::cout<< "Equi-Distance UAV: residual[0]= "<<residual[0]<<" , d_pos= "<< d_pos <<" , init_d= "<<init_d<<std::endl;

      std::ofstream ofs;
      std::string name_output_file = "/home/simon/residuals_optimization_data/equidistancia_uav.txt";
      ofs.open(name_output_file.c_str(), std::ofstream::app);
      if (ofs.is_open()) 
        ofs << residual[0] << ";" <<std::endl;
      ofs.close();

      return true;
    
    }

    double wf_, i_d;

  private:

};

#endif