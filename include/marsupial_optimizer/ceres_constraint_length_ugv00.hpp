#ifndef CERES_CONSTRAINS_LENGTH_UGV_HPP
#define CERES_CONSTRAINS_LENGTH_UGV_HPP


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

class LengthFunctorUGV : public SizedCostFunction<1, 4, 4, 4, 4, 4, 4> 
{

public:
  	LengthFunctorUGV(double weight_factor, double dist_1, double dist_2, double dist_3, double dist_4, double dist_5): 
	  				 wf_(weight_factor), d1_(dist_1), d2_(dist_2), d3_(dist_3), d4_(dist_4), d5_(dist_5) {}

	virtual ~LengthFunctorUGV(void) 
    {

    }
	
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
	{
		// Length UGV trajectory
		double p1 = parameters[0][0]; double x1 = parameters[0][1]; double y1 = parameters[0][2]; double z1 = parameters[0][3];
		double p2 = parameters[1][0]; double x2 = parameters[1][1]; double y2 = parameters[1][2]; double z2 = parameters[1][3]; 
		double p3 = parameters[2][0]; double x3 = parameters[2][1]; double y3 = parameters[2][2]; double z3 = parameters[2][3]; 
		double p4 = parameters[3][0]; double x4 = parameters[3][1]; double y4 = parameters[3][2]; double z4 = parameters[3][3]; 
		double p5 = parameters[4][0]; double x5 = parameters[4][1]; double y5 = parameters[4][2]; double z5 = parameters[4][3]; 
		double p6 = parameters[5][0]; double x6 = parameters[5][1]; double y6 = parameters[5][2]; double z6 = parameters[5][3];

		double arg_dist_1_ =  (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2) ;
		double arg_dist_2_ =  (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) + (z2 - z3)*(z2 - z3) ;
		double arg_dist_3_ =  (x3 - x4)*(x3 - x4) + (y3 - y4)*(y3 - y4) + (z3 - z4)*(z3 - z4) ;
		double arg_dist_4_ =  (x4 - x5)*(x4 - x5) + (y4 - y5)*(y4 - y5) + (z4 - z5)*(z4 - z5) ;
		double arg_dist_5_ =  (x5 - x6)*(x5 - x6) + (y5 - y6)*(y5 - y6) + (z5 - z6)*(z5 - z6) ;
		double dist_1_ = sqrt( arg_dist_1_ );
		double dist_2_ = sqrt( arg_dist_2_ );
		double dist_3_ = sqrt( arg_dist_3_ );
		double dist_4_ = sqrt( arg_dist_4_ );
		double dist_5_ = sqrt( arg_dist_5_ );

		// Reference to current path
		double dist_ = dist_1_ + dist_2_ + dist_3_ + dist_4_ + dist_5_;
		// Reference Length initial path
		double d_ref_ = d1_ + d2_ + d3_ + d4_ + d5_;

		//Compute Residual
		if (dist_ < d_ref_)
			residuals[0] =  0.0;
		else
			residuals[0] =  wf_ * 100.0 * (exp(dist_ - d_ref_)-1.0);
		
		if (jacobians != NULL && jacobians[0] != NULL) 
        {
        	if (arg_dist_1_ < 0.001){
				jacobians[0][0] = 0.0;
				jacobians[0][1] = 0.0;
				jacobians[0][2] = 0.0;
				jacobians[0][3] = 0.0;
          	}
          	else{
				jacobians[0][0] = 0.0 ;
				jacobians[0][1] = (2.0*x1 - 2.0*x2)/sqrt(2.0*(arg_dist_1_));
				jacobians[0][2] = (2.0*y1 - 2.0*y2)/sqrt(2.0*(arg_dist_1_));
				jacobians[0][3] = (2.0*z1 - 2.0*z2)/sqrt(2.0*(arg_dist_1_));
			}
			if (arg_dist_2_ < 0.001){
				jacobians[1][0] = 0.0;
				jacobians[1][1] = 0.0;
				jacobians[1][2] = 0.0;
				jacobians[1][3] = 0.0;
			}else{
				jacobians[1][0] = 0.0 ;
				jacobians[1][1] = (2.0*x2 - 2.0*x3)/(2.0*sqrt(arg_dist_2_)) - (2.0*x1 - 2.0*x2)/(2.0*sqrt(arg_dist_1_));
				jacobians[1][2] = (2.0*y2 - 2.0*y3)/(2.0*sqrt(arg_dist_2_)) - (2.0*y1 - 2.0*y2)/(2.0*sqrt(arg_dist_1_));
				jacobians[1][3] = (2.0*z2 - 2.0*z3)/(2.0*sqrt(arg_dist_2_)) - (2.0*z1 - 2.0*z2)/(2.0*sqrt(arg_dist_1_));
			}
			
				jacobians[2][0] = 0.0 ;
				jacobians[2][1] = (2.0*x3 - 2.0*x4)/(2.0*sqrt(arg_dist_3_)) - (2.0*x2 - 2.0*x3)/(2.0*sqrt(arg_dist_2_));
				jacobians[2][2] = (2.0*y3 - 2.0*y4)/(2.0*sqrt(arg_dist_3_)) - (2.0*y2 - 2.0*y3)/(2.0*sqrt(arg_dist_2_));
				jacobians[2][3] = (2.0*z3 - 2.0*z4)/(2.0*sqrt(arg_dist_3_)) - (2.0*z2 - 2.0*z3)/(2.0*sqrt(arg_dist_2_));
				jacobians[3][0] = 0.0 ;
				jacobians[3][1] = (2.0*x4 - 2.0*x5)/(2.0*sqrt(arg_dist_4_)) - (2.0*x3 - 2.0*x4)/(2.0*sqrt(arg_dist_3_));
				jacobians[3][2] = (2.0*y4 - 2.0*y5)/(2.0*sqrt(arg_dist_4_)) - (2.0*y3 - 2.0*y4)/(2.0*sqrt(arg_dist_3_));
				jacobians[3][3] = (2.0*z4 - 2.0*z5)/(2.0*sqrt(arg_dist_4_)) - (2.0*z3 - 2.0*z4)/(2.0*sqrt(arg_dist_3_));
				jacobians[4][0] = 0.0 ;
				jacobians[4][1] = (2.0*x5 - 2.0*x6)/(2.0*sqrt(arg_dist_5_)) - (2.0*x4 - 2.0*x5)/(2.0*sqrt(arg_dist_4_));
				jacobians[4][2] = (2.0*y5 - 2.0*y6)/(2.0*sqrt(arg_dist_5_)) - (2.0*y4 - 2.0*y5)/(2.0*sqrt(arg_dist_4_));
				jacobians[4][3] = (2.0*z5 - 2.0*z6)/(2.0*sqrt(arg_dist_5_)) - (2.0*z4 - 2.0*z5)/(2.0*sqrt(arg_dist_4_));
				jacobians[5][0] = 0.0 ;
				jacobians[5][1] = -(2.0*x5 - 2.0*x6)/(2.0*sqrt(arg_dist_5_));
				jacobians[5][2] = -(2.0*y5 - 2.0*y6)/(2.0*sqrt(arg_dist_5_));
				jacobians[5][3] = -(2.0*z5 - 2.0*z6)/(2.0*sqrt(arg_dist_5_));
			}
        }

		return true;
		
	}

	double wf_, d1_, d2_, d3_, d4_, d5_;

private:

};


#endif