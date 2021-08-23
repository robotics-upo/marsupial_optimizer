#ifndef CERES_CONSTRAINS_VELOCITY_UAV_HPP
#define CERES_CONSTRAINS_VELOCITY_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class VelocityFunctorUAV : public SizedCostFunction<1, 4, 4, 2>   
{

public:
  	VelocityFunctorUAV(double weight_factor, double init_vel_)
  	: wf_(weight_factor), iv_(init_vel_) 
  	{}

  	~VelocityFunctorUAV(void)
  	{}

	virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
	{	
		double p1 = parameters[0][0];
	  	double x1 = parameters[0][1];
	  	double y1 = parameters[0][2];
	  	double z1 = parameters[0][3];
	  	double p2 = parameters[1][0];
	  	double x2 = parameters[1][1];
	  	double y2 = parameters[1][2];
	  	double z2 = parameters[1][3];
	  	double q = parameters[2][0];
	  	double t = parameters[2][1];
    	double d_ugv;
    
   		double d_uav = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
     
    	if(t < 0.001 || d_uav < 0.001){
    	  residuals[0] = 0.0;
    	  if (jacobians != NULL && jacobians[0] != NULL){
					jacobians[0][0] = 0.0;
					jacobians[0][1] = 0.0;
					jacobians[0][2] = 0.0;
					jacobians[0][3] = 0.0;
				}
    	  if (jacobians != NULL && jacobians[1] != NULL){
					jacobians[1][0] = 0.0;
					jacobians[1][1] = 0.0;
					jacobians[1][2] = 0.0;
					jacobians[1][3] = 0.0;
				}
    	  if (jacobians != NULL && jacobians[2] != NULL){
					jacobians[2][0] = 0.0;
					jacobians[2][1] = 0.0;
				}
    	}
    	else{
    	  residuals[0] =  wf_ * ((d_uav / t) - iv_);
    	  if (jacobians != NULL && jacobians[0] != NULL){
					jacobians[0][0] = 0.0;
					jacobians[0][1] = (wf_*(2.0*y1 - 2.0*y2))/(2.0*t*d_uav);
					jacobians[0][2] = (wf_*(2.0*y1 - 2.0*y2))/(2.0*t*d_uav);
					jacobians[0][3] = (wf_*(2.0*z1 - 2.0*z2))/(2.0*t*d_uav);
				}
    	  if (jacobians != NULL && jacobians[1] != NULL){
					jacobians[1][0] = 0.0;
					jacobians[1][1] = -(wf_*(2.0*x1 - 2.0*x2))/(2.0*t*d_uav);
					jacobians[1][2] = -(wf_*(2.0*y1 - 2.0*y2))/(2.0*t*d_uav);
					jacobians[1][3] = -(wf_*(2.0*z1 - 2.0*z2))/(2.0*t*d_uav);
				}
    	  if (jacobians != NULL && jacobians[2] != NULL){
					jacobians[2][0] = 0.0;
					jacobians[2][1] = -(wf_*d_uav)/(t*t);
				}
    	}  
    	return true;
  	}

 	double wf_, iv_;

 	private:
};

#endif