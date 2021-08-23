#ifndef CERES_CONSTRAINS_KINEMATICS_UGV_ANALYTIC_HPP
#define CERES_CONSTRAINS_KINEMATICS_UGV_ANALYTIC_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <ros/ros.h>

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class KinematicsFunctorUGVAnalytic : public SizedCostFunction<1, 4, 4, 4> 
{

public:

  	KinematicsFunctorUGVAnalytic(double weight_factor, double angle_bound, double count_fix_points_ugv)
	  : wf_(weight_factor), ang_(angle_bound), cfp_ugv(count_fix_points_ugv)
	{}

    virtual ~KinematicsFunctorUGVAnalytic(void) 
    {}


	virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {	
		// Kinematics for ugv XY Axes
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
		double factor_ = 100.0;

		double dot_product = (x3 - x2)*(x2 - x1) + (y3 - y2)*(y2 - y1);
	
		//Compute norm of vectors
		double arg1 = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
		double arg2 = (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3);
		double norm_vector1 = sqrt(arg1);
		double norm_vector2 = sqrt(arg2);

		// Compute cos(angle)	
		double cos_angle = dot_product/(norm_vector1 * norm_vector2);
		double bound = cos(ang_);

		//Compute Residual
		if ( cos_angle > bound || norm_vector1 < 0.0001 && norm_vector1 > -0.0001 || norm_vector2 < 0.0001 && norm_vector2 > -0.0001) 
			residuals[0] = 0.0;
		else
			residuals[0] =  wf_ * factor_ * (cos_angle - 1.0);

		if (jacobians != NULL && jacobians[0] != NULL) 
      	{
			if ( cos_angle > bound || norm_vector1 < 0.0001 && norm_vector1 > -0.0001 || norm_vector2 < 0.0001 && norm_vector2 > -0.0001){
				jacobians[0][0] = 0.0; 
				jacobians[0][1] = 0.0;
				jacobians[0][2] = 0.0;
				jacobians[0][3] = 0.0;
			}
        	else{
				jacobians[0][0] = 0.0;
				jacobians[0][1] = factor_*wf_*((x2 - x3)/(sqrt(arg1)*sqrt(arg2)) - ((2.0*x1 - 2.0*x2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*pow(arg1,3/2)*sqrt(arg2)));
				jacobians[0][2] = factor_*wf_*((y2 - y3)/(sqrt(arg1)*sqrt(arg2)) - ((2.0*y1 - 2.0*y2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*pow(arg1,3/2)*sqrt(arg2)));
				jacobians[0][3] = 0.0;
			}
      	}
		if (jacobians != NULL && jacobians[1] != NULL) 
      	{
			if ( cos_angle > bound || norm_vector1 < 0.0001 && norm_vector1 > -0.0001 || norm_vector2 < 0.0001 && norm_vector2 > -0.0001){
				jacobians[1][0] = 0.0;
				jacobians[1][1] = 0.0;
				jacobians[1][2] = 0.0;
				jacobians[1][3] = 0.0;
			}
        	else{
				jacobians[1][0] = 0.0;
				jacobians[1][1] = factor_*wf_*((x1 - 2.0*x2 + x3)/(sqrt(arg1)*sqrt(arg2)) + ((2.0*x1 - 2.0*x2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*pow(arg1,3/2)*sqrt(arg2)) - ((2.0*x2 - 2.0*x3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*sqrt(arg1)*pow(arg2,3/2)));
				jacobians[1][2] = factor_*wf_*((y1 - 2.0*y2 + y3)/(sqrt(arg1)*sqrt(arg2)) + ((2.0*y1 - 2.0*y2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*pow(arg1,3/2)*sqrt(arg2)) - ((2.0*y2 - 2.0*y3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*sqrt(arg1)*pow(arg2,3/2)));
				jacobians[1][3] = 0.0;
			}
      	}
		if (jacobians != NULL && jacobians[2] != NULL) 
      	{
			if ( cos_angle > bound || norm_vector1 < 0.0001 && norm_vector1 > -0.0001 || norm_vector2 < 0.0001 && norm_vector2 > -0.0001){
				jacobians[2][0] = 0.0;
				jacobians[2][1] = 0.0;
				jacobians[2][2] = 0.0;
				jacobians[2][3] = 0.0;
			}
        	else{
				jacobians[2][0] = 0.0;
				jacobians[2][1] = -factor_*wf_*((x1 - x2)/(sqrt(arg1)*sqrt(arg2)) - ((2.0*x2 - 2.0*x3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*sqrt(arg1)*pow(arg2,3/2)));
				jacobians[2][2] = -factor_*wf_*((y1 - y2)/(sqrt(arg1)*sqrt(arg2)) - ((2.0*y2 - 2.0*y3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3)))/(2.0*sqrt(arg1)*pow(arg2,3/2)));
				jacobians[2][3] = 0.0;
			}
      	}
		
		return true;
	}

	double wf_, ang_, cfp_ugv;

private:

};


#endif