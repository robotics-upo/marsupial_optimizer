#ifndef CERES_ACCELERATION_VELOCITY_UAV_HPP
#define CERES_ACCELERATION_VELOCITY_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class AccelerationFunctorUAV : public SizedCostFunction<1, 4, 4, 4, 2, 2>    
{

public:
	AccelerationFunctorUAV(double weight_factor, double init_acc_uav)
	: wf_(weight_factor), ia_uav(init_acc_uav)
	{}

	~AccelerationFunctorUAV()
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
	  	double p3 = parameters[2][0];
	  	double x3 = parameters[2][1];
	  	double y3 = parameters[2][2];
	  	double z3 = parameters[2][3];
		double q1 = parameters[3][0];
	  	double t1 = parameters[3][1];
	  	double q2 = parameters[4][0];
	  	double t2 = parameters[4][1];

		double d1_uav_ = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
		double d2_uav_ = sqrt( (x2-x3)*(x2-x3) + (y2-y3)*(y2-y3) + (z2-z3)*(z2-z3) );
		double v1_uav, v2_uav, a_uav;

		v1_uav = d1_uav_ / (t1);
		v2_uav = d2_uav_ / (t2);
		a_uav = (v2_uav - v1_uav)/(t1 + t2);
		
		residuals[0] =  wf_ * (a_uav - ia_uav);
	
		if (jacobians != NULL && jacobians[0] != NULL){
			jacobians[0][0] = 0.0;
			jacobians[0][1] = -(wf_*(2.0*x1 - 2.0*x2))/(2.0*t1*(t1 + t2)*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)));
			jacobians[0][2] = -(wf_*(2.0*y1 - 2.0*y2))/(2.0*t1*(t1 + t2)*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)));
			jacobians[0][3] = -(wf_*(2.0*z1 - 2.0*z2))/(2.0*t1*(t1 + t2)*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2)));
		}
    	if (jacobians != NULL && jacobians[1] != NULL){
			jacobians[1][0] = 0.0;
			jacobians[1][1] = (wf_*((2.0*x1 - 2.0*x2)/(2.0*t1*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2))) + (2*x2 - 2*x3)/(2*t2*sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3)))))/(t1 + t2);
			jacobians[1][2] = (wf_*((2.0*y1 - 2.0*y2)/(2.0*t1*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2))) + (2*y2 - 2*y3)/(2*t2*sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3)))))/(t1 + t2);
			jacobians[1][3] = (wf_*((2.0*z1 - 2.0*z2)/(2.0*t1*sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2))) + (2*z2 - 2*z3)/(2*t2*sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3)))))/(t1 + t2);
		}
		if (jacobians != NULL && jacobians[2] != NULL){
			jacobians[2][0] = 0.0;
			jacobians[2][1] = -(wf_*(2.0*x2 - 2.0*x3))/(2*t2*(t1 + t2)*sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3)));
			jacobians[2][2] = -(wf_*(2.0*y2 - 2.0*y3))/(2*t2*(t1 + t2)*sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3)));
			jacobians[2][3] = -(wf_*(2.0*z2 - 2.0*z3))/(2*t2*(t1 + t2)*sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3)));
		}		
    	if (jacobians != NULL && jacobians[3] != NULL){
			jacobians[3][0] = 0.0;
			jacobians[3][1] = wf_*((sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2))/t1 - sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3))/t2)/((t1 + t2)*(t1 + t2)) + sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2))/(t1*t1*(t1 + t2)));
		}
		if (jacobians != NULL && jacobians[4] != NULL){
			jacobians[4][0] = 0.0;
			jacobians[4][1] = wf_*((sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2))/t1 - sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3))/t2)/((t1 + t2)*(t1 + t2)) - sqrt((x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3) + (x2 - x3)*(x2 - x3))/(t2*t2*(t1 + t2)));
		}
		return true;
	}

double wf_, ia_uav;

private:

};

#endif