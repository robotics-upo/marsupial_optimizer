#ifndef CERES_CONSTRAINS_VELOCITYUGV_HPP
#define CERES_CONSTRAINS_VELOCITYUGV_HPP


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

class VelocityFunctorUGV {

public:
  VelocityFunctorUGV(double weight_factor, double init_vel_, double count_fix_points_ugv, bool write_data, std::string user_name)
                    : wf_(weight_factor), iv_(init_vel_), cfp_ugv(count_fix_points_ugv), w_d_(write_data), user_(user_name) 
  {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const stateT, T* residual) const 
  {
      if (statePos2[0] < cfp_ugv){
        residual[0] = T{0.0}; 
        return true;
      }
      else{
        T d_, arg_d_ugv_;
        
        arg_d_ugv_ = pow(statePos2[1]-statePos1[1],2) + pow(statePos2[2]-statePos1[2],2) + pow(statePos2[3]-statePos1[3],2);
        
        if (arg_d_ugv_ < 0.001 && arg_d_ugv_ > -0.001)
          d_ = T{0.0};
        else               
          d_ = sqrt(arg_d_ugv_);
        
        if(stateT[1] < 0.001)
          residual[0] = T{0.0};
        else
          residual[0] =  wf_ * ((d_ / stateT[1]) - iv_);  //To avoid division that make underterminated residual: v*t=d

        // std::cout << "VelocityFunctorUGV : residual[0]= " << residual[0] << " , d_= " << d_ << " , stateT[1]= " << stateT[1] << std::endl;
        
	      if(w_d_){
          std::ofstream ofs;
          std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/velocity_ugv.txt";
          ofs.open(name_output_file.c_str(), std::ofstream::app);
          if (ofs.is_open()) 
              ofs << residual[0] << "/" <<std::endl;
          ofs.close();
        }

        return true;
      }
      
  }

 bool w_d_;
 double wf_, iv_, cfp_ugv;
 std::string user_;

 private:
};


#endif