#ifndef CERES_CONSTRAINS_VELOCITY_UAV_HPP
#define CERES_CONSTRAINS_VELOCITY_UAV_HPP


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

class VelocityFunctorUAV {

public:
  VelocityFunctorUAV(double weight_factor, double init_vel_, bool write_data, std::string user_name): wf_(weight_factor), iv_(init_vel_), w_d_(write_data), user_(user_name) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const stateT, T* residual) const 
  {
      T d_, arg_d_;

      arg_d_ = pow(statePos2[1]-statePos1[1],2) + pow(statePos2[2]-statePos1[2],2) + pow(statePos2[3]-statePos1[3],2) ;

      if(arg_d_ < 0.0001 && arg_d_ > -0.0001)
        d_ = T{0.0};
      else  
        d_ = sqrt(arg_d_);
      
      if(stateT[1] < 0.001)
        residual[0] = T{0.0};
      else
        residual[0] =  wf_ * ((d_ / stateT[1]) - iv_);

      // std::cout << "VelocityFunctorUAV : residual[0]= " << residual[0] << " , d_= " << d_ << " , stateT[1]= " << stateT[1] << std::endl;
      
	    if(w_d_){
        std::ofstream ofs;
        std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/velocity_uav.txt";
        ofs.open(name_output_file.c_str(), std::ofstream::app);
        if (ofs.is_open()) 
            ofs << residual[0] << "/" <<std::endl;
        ofs.close();
      }

      return true;
  }

 bool w_d_;
 double wf_, iv_;
 std::string user_;

 private:
};


#endif