#ifndef CERES_CONSTRAINT_KINEMATICS_UAV_ANALYTIC_HPP
#define CERES_CONSTRAINT_KINEMATICS_UAV_ANALYTIC_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class smoothnessFunctorUAVAnalytic : public SizedCostFunction<1, 4, 4, 4>  
{

public:
	smoothnessFunctorUAVAnalytic(double weight_factor, double angle_bound, int fix_pos_init_uav, int fix_pos_final_uav)
						: wf_(weight_factor), ang_(angle_bound), fpi_(fix_pos_init_uav), fpf_(fix_pos_final_uav) 
	{}

    virtual ~smoothnessFunctorUAVAnalytic(void) 
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
		double factor_ = 1.0;
        double f_slope_ = -4.0;

		double dot_product = (x3 - x2)*(x2 - x1) + (y3 - y2)*(y2 - y1) + (z3 - z2)*(z2 - z1);
	
		//Compute norm of vectors
		double arg1 = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2);
		double arg2 = (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) + (z2 - z3)*(z2 - z3);
		double norm_vector1 = sqrt(arg1);
		double norm_vector2 = sqrt(arg2);
		double cos_angle, angle_;

		// Compute cos(angle)
		if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001)
			cos_angle = 0.0;
		else
			cos_angle = dot_product/(norm_vector1 * norm_vector2);
		
		//Compute Residual
		double bound = cos(ang_);
		if ( cos_angle > bound) 
			angle_ = bound;
		else
			angle_ = cos_angle;
		
		double max_value_residual = 100.0;
		double min_value_residual = 0.0;
		double m = (max_value_residual - min_value_residual) / (-1 - bound);
		
		residuals[0] =  wf_ * m*(angle_ - bound);
		// residuals[0] =  wf_ * factor_ * exp(f_slope_*(cos_angle/2.0 - 1.0));
		

		if(p1 > (double)fpi_){
			if (jacobians != NULL && jacobians[0] != NULL) {
				jacobians[0][0] = 0.0;
				jacobians[0][1] = m*wf_*((x2 - x3)/(norm_vector1*norm_vector2) - ((2.0*x1 - 2.0*x2)*(dot_product))/(2.0*pow(arg1,3/2)*norm_vector2));
				jacobians[0][2] = m*wf_*((y2 - y3)/(norm_vector1*norm_vector2) - ((2.0*y1 - 2.0*y2)*(dot_product))/(2.0*pow(arg1,3/2)*norm_vector2));
				jacobians[0][3] = m*wf_*((z2 - z3)/(norm_vector1*norm_vector2) - ((2.0*z1 - 2.0*z2)*(dot_product))/(2.0*pow(arg1,3/2)*norm_vector2));
			}
		}
		if (jacobians != NULL && jacobians[1] != NULL) {
			jacobians[1][0] = 0.0;
			jacobians[1][1] = m*wf_*((x1 - 2*x2 + x3)/(norm_vector1*norm_vector2) + ((2.0*x1 - 2.0*x2)*(dot_product))/(2.0*pow(arg1,3/2)*norm_vector2) - ((2.0*x2 - 2.0*x3)*(dot_product))/(2.0*norm_vector1*pow(arg2,3/2)));
			jacobians[1][2] = m*wf_*((y1 - 2*y2 + y3)/(norm_vector1*norm_vector2) + ((2.0*y1 - 2.0*y2)*(dot_product))/(2.0*pow(arg1,3/2)*norm_vector2) - ((2.0*y2 - 2.0*y3)*(dot_product))/(2.0*norm_vector1*pow(arg2,3/2)));
			jacobians[1][3] = m*wf_*((z1 - 2*z2 + z3)/(norm_vector1*norm_vector2) + ((2.0*z1 - 2.0*z2)*(dot_product))/(2.0*pow(arg1,3/2)*norm_vector2) - ((2.0*z2 - 2.0*z3)*(dot_product))/(2.0*norm_vector1*pow(arg2,3/2)));
		}
		if(p3 < (double)fpf_){
			if (jacobians != NULL && jacobians[2] != NULL) {
				jacobians[2][0] = 0.0;
				jacobians[2][1] = -m*wf_*((x1 - x2)/(norm_vector1*norm_vector2) - ((2.0*x2 - 2.0*x3)*(dot_product))/(2.0*norm_vector1*pow(arg2,3/2)));
				jacobians[2][2] = -m*wf_*((y1 - y2)/(norm_vector1*norm_vector2) - ((2.0*y2 - 2.0*y3)*(dot_product))/(2.0*norm_vector1*pow(arg2,3/2)));
				jacobians[2][3] = -m*wf_*((z1 - z2)/(norm_vector1*norm_vector2) - ((2.0*z2 - 2.0*z3)*(dot_product))/(2.0*norm_vector1*pow(arg2,3/2)));
			}
		}
		

		return true;
  	}

 double wf_, ang_, fpi_, fpf_;

 private:
};

#endif