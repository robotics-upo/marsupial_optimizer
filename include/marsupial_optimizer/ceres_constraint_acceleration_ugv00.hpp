#ifndef CERES_ACCELERATION_VELOCITY_UGV_HPP
#define CERES_ACCELERATION_VELOCITY_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class AccelerationFunctorUGV: public SizedCostFunction<1, 4, 4, 4, 2, 2>  
{

public:
  	AccelerationFunctorUGV(double weight_factor, double init_acc_ugv, double count_fix_points_ugv)
  						: wf_(weight_factor), ia_ugv(init_acc_ugv), cfp_ugv(count_fix_points_ugv) 
	{}

	~AccelerationFunctorUGV()
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

		double d1_ugv, d2_ugv;
		double v1_ugv, v2_ugv, a_ugv;
		printf("\n\t\t Starting AccelerationFunctorUGV()\n");	
		printf("p1=%f p2=%f p3=%f cfp_ugv=%f \n",p1,p2,p3,cfp_ugv);
		//Get value distance between pos 1 and pos 2
		d1_ugv = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
	
		//Get value distance between pos 2 and pos 3
		d2_ugv = sqrt((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) + (z2 - z3)*(z2 - z3));

		if(t1 < 0.001)
			v1_ugv = 0.0;
		else
			v1_ugv = d1_ugv / (t1);
		
		if(t2 < 0.001)
			v2_ugv = 0.0;
		else
			v2_ugv = d2_ugv / (t2);
		
		a_ugv = (v2_ugv - v1_ugv)/(t1 + t2);

		if(t1 < 0.001 && t2< 0.001  ){
			residuals[0] =  0.0;
			printf("1 residual = %f\n", residuals[0]);
		}
		else{
			residuals[0] =  wf_ * (a_ugv - ia_ugv);
			printf("2 residual = %f\n", residuals[0]);
		}

		printf("d1_ugv=%f , d2_ugv=%f , t1=%f , t2=%f\n",d1_ugv, d2_ugv, t1, t2);

		if (jacobians != NULL && jacobians[0] != NULL) 
     	{
			if (t1 < 0.001 || p2 < cfp_ugv){
				jacobians[0][0] = 0.0;
				jacobians[0][1] = 0.0;
				jacobians[0][2] = 0.0;
				jacobians[0][3] = 0.0;
			}else{
				jacobians[0][0] = 0.0;
				jacobians[0][1] = -(wf_*(2.0*x1 - 2.0*x2))/(2.0*t1*(t1 + t2)*d1_ugv);
				jacobians[0][2] = -(wf_*(2.0*y1 - 2.0*y2))/(2.0*t1*(t1 + t2)*d1_ugv);
				jacobians[0][3] = -(wf_*(2.0*z1 - 2.0*z2))/(2.0*t1*(t1 + t2)*d1_ugv);
			}
			printf("jacobians[0]=[%f %f %f %f]\n",jacobians[0][0], jacobians[0][1], jacobians[0][2], jacobians[0][3]);
			
			if ( t1 < 0.001 || t2< 0.001 || p2 < cfp_ugv){ // p2 < cfp_ugv it's skipped because this condition is constant
				jacobians[1][0] = 0.0;
				jacobians[1][1] = 0.0;
				jacobians[1][2] = 0.0;
				jacobians[1][3] = 0.0;
			}else{
				jacobians[1][0] = 0.0;
				jacobians[1][1] = (wf_*((2.0*x1 - 2.0*x2)/(2.0*t1*d1_ugv) + (2*x2 - 2*x3)/(2.0*t2*d2_ugv)))/(t1 + t2);
				jacobians[1][2] = (wf_*((2.0*y1 - 2.0*y2)/(2.0*t1*d1_ugv) + (2*y2 - 2*y3)/(2.0*t2*d2_ugv)))/(t1 + t2);
				jacobians[1][3] = (wf_*((2.0*z1 - 2.0*z2)/(2.0*t1*d1_ugv) + (2*z2 - 2*z3)/(2.0*t2*d2_ugv)))/(t1 + t2);
			}
			printf("jacobians[1]=[%f %f %f %f]\n",jacobians[1][0], jacobians[1][1], jacobians[1][2], jacobians[1][3]);
			
			if (t2 < 0.001 || p3 < cfp_ugv){ // p3 < cfp_ugv it's skipped because this condition is constant
				jacobians[2][0] = 0.0;
				jacobians[2][1] = 0.0;
				jacobians[2][2] = 0.0;
				jacobians[2][3] = 0.0;
			}else{
				jacobians[2][0] = 0.0;
				jacobians[2][1] = -(wf_*(2.0*x2 - 2.0*x3))/(2.0*t2*(t1 + t2)*d2_ugv);
				jacobians[2][2] = -(wf_*(2.0*y2 - 2.0*y3))/(2.0*t2*(t1 + t2)*d2_ugv);
				jacobians[2][3] = -(wf_*(2.0*z2 - 2.0*z3))/(2.0*t2*(t1 + t2)*d2_ugv);
			}
			printf("jacobians[2]=[%f %f %f %f]\n",jacobians[2][0], jacobians[2][1], jacobians[2][2], jacobians[2][3]);
		
			if (p2 >= cfp_ugv){
				jacobians[3][0] = 0.0;
				jacobians[3][1] = 0.0;
				printf("jacobians[3]=[%f %f]\n",jacobians[3][0], jacobians[3][1]);
			}if (p3 >= cfp_ugv){
				jacobians[4][0] = 0.0;
				jacobians[4][1] = wf_*((d1_ugv/t1 - d2_ugv/t2)/((t1 + t2)*(t1 + t2)) - d2_ugv/(t2*t2*(t1 + t2)));
				printf("jacobians[4]=[%f %f]\n",jacobians[4][0], jacobians[4][1]);
			}
			printf("here 1\n");
				// jacobians[3][0] = 0.0;
			 	// jacobians[3][1] = wf_*((d1_ugv/t1 - d2_ugv/t2)/((t1 + t2)*(t1 + t2)) + d1_ugv/(t1*t1*(t1 + t2)));
			 	// jacobians[4][0] = 0.0;
			 	// jacobians[4][1] = wf_*((d1_ugv/t1 - d2_ugv/t2)/((t1 + t2)*(t1 + t2)) - d2_ugv/(t2*t2*(t1 + t2)));
		}
		return true;
	}

 	double wf_, ia_ugv, cfp_ugv;

 	private:
};

#endif