#ifndef CERES_ACCELERATION_VELOCITY_UAV_HPP
#define CERES_ACCELERATION_VELOCITY_UAV_HPP

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

class AccelerationFunctorUAV {

public:
	AccelerationFunctorUAV(double weight_factor, double init_acc_uav, bool write_data, std::string user_name): wf_(weight_factor), ia_(init_acc_uav), w_d_(write_data), user_(user_name){}

  	template <typename T>
  	bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, 
					  const T* const stateT1, const T* const stateT2, T* residual) const 
{
  	T d1_, d2_, arg_d1_, arg_d2_, v1_, v2_, a_;
	
	double low_value = 0.001;

	arg_d1_ = sqrt( pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2) );
	if (arg_d1_ < low_value && arg_d1_ > -low_value)
		d1_ = T{0.0};
	else
		d1_ = sqrt(arg_d1_);

	arg_d2_ = sqrt( pow(statePos2[1]-statePos3[1],2) + pow(statePos2[2]-statePos3[2],2) + pow(statePos2[3]-statePos3[3],2) );
	if (arg_d2_ < low_value && arg_d2_ > -low_value)
		d2_ = T{0.0};
	else
		d2_ = sqrt(arg_d2_);

	if(stateT1[1] < low_value)
		v1_ = T{0.0};
	else
		v1_ = d1_ / (stateT1[1]);
	
	if(stateT2[1] < low_value)
		v2_ = T{0.0};
	else
		v2_ = d2_ / (stateT2[1]);
	

	if(stateT1[1] < low_value && stateT2[1]< low_value){
  		a_ = T{0.0};
		residual[0] =  a_;
	}	
	else{	  
	  	a_ = (v2_ - v1_)/(stateT1[1] + stateT2[1]);
		residual[0] =  wf_ * (a_ - ia_);
	}

	// std::cout << "AccelerationFunctorUAV : residual[0]= " << residual[0] << " , a_= " << a_ << " , stateT1[1]= " << stateT1[1]
	// 	 		  << " , stateT2[1]= " << stateT2[1] << " , v1_= " << v1_ <<" , v2_= " << v2_ << std::endl;
	
	if(w_d_){
		std::ofstream ofs;
		std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/acceleration_uav.txt";
		ofs.open(name_output_file.c_str(), std::ofstream::app);
		if (ofs.is_open()) 
			ofs << residual[0] << "/" <<std::endl;
		ofs.close();
	}

	return true;
}

double wf_, ia_;
bool w_d_;
std::string user_;

private:
};


#endif