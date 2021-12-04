#ifndef CERES_CONSTRAINS_TIME_HPP
#define CERES_CONSTRAINS_TIME_HPP


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

class TimeFunctor {

public:
  TimeFunctor(double weight_factor, double init_time_, bool write_data): wf_(weight_factor), it_(init_time_), w_d_(write_data) {}

  template <typename T>
  bool operator()(const T* const stateT, T* residual) const 
  {
    residual[0] =  wf_ * (stateT[1] - it_);

    // std::cout << "TimeFunctor: residual[0] =" << residual[0] <<  " , stateT[1]=" << stateT[1] << " , it_=" << it_ << std::endl;
   
    if(w_d_){
      std::ofstream ofs;
	    std::string name_output_file = "/home/simon/residuals_optimization_data/time.txt";
	    ofs.open(name_output_file.c_str(), std::ofstream::app);
	    if (ofs.is_open()) 
	      ofs << residual[0] << "/" <<std::endl;
	    ofs.close();
    }

    return true;
  }

 bool w_d_;
 double wf_, it_;

 private:
};


#endif