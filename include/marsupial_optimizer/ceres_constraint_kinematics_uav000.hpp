#ifndef CERES_CONSTRAINS_KINEMATICS_UAV_HPP
#define CERES_CONSTRAINS_KINEMATICS_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class KinematicsFunctorUAV : public SizedCostFunction<1, 4, 4, 4>  
{

public:
	KinematicsFunctorUAV(double weight_factor, double angle_bound, int fix_pos_init_uav, int fix_pos_final_uav)
						: wf_(weight_factor), ang_(angle_bound), fpi_(fix_pos_init_uav), fpf_(fix_pos_final_uav) 
	{}

    virtual ~KinematicsFunctorUAV(void) 
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

		double dot_product = (x3 - x2)*(x2 - x1) + (y3 - y2)*(y2 - y1) + (z3 - z2)*(z2 - z1);
	
		//Compute norm of vectors
		double arg1 = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2);
		double arg2 = (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) + (z2 - z3)*(z2 - z3);
		double norm_vector1 = sqrt(arg1);
		double norm_vector2 = sqrt(arg2);
		double cos_angle;

		// Compute cos(angle)
		if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001)
			cos_angle = 0.0;
		else
			cos_angle = dot_product/(norm_vector1 * norm_vector2);
		
		//Compute Residual
		double bound = cos(ang_);
		if ( cos_angle > bound) 
			residuals[0] = 0.0;
		else
			residuals[0] =  wf_ * factor_ * exp(-1.0*(cos_angle - 1.0));
        printf("KinematicsFunctorUAV[%.2f,%.2f,%.2f]: residuals[0] =[%f]\n",p1,p2,p3,residuals[0]);
		
		// printf("KinematicsFunctorUAV: Vec=[%.2f %.2f %.2f] residuals[0]=%f cos_angle=%f  norm_vector1=%f  norm_vector2=%f  arg1=%f  arg2=%f (s_-1.0)=%f\n",
		// p1,p2,p3,residuals[0], cos_angle, norm_vector1, norm_vector2, arg1, arg2, s_-1.0);

		if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001 || cos_angle > bound){
			if(p1 > (double)fpi_){
			if (jacobians != NULL && jacobians[0] != NULL) 
				{
				// printf("entro: ");
					jacobians[0][0] = 0.0;
					jacobians[0][1] = 0.0;
					jacobians[0][2] = 0.0;
					jacobians[0][3] = 0.0;
				// printf("1 jacobians[0][0]=%f , jacobians[0][1]=%f , jacobians[0][2]=%f , jacobians[0][3]=%f\n",jacobians[0][0],jacobians[0][1],jacobians[0][2],jacobians[0][3]);
					// printf("KinematicsFunctorUAV: jacobians[0]=[%f,%f,%f,%f]\n", jacobians[0][0],jacobians[0][1],jacobians[0][2],jacobians[0][3]);
				}
			}
			if (jacobians != NULL && jacobians[1] != NULL) 
			{
				// printf("entro: ");
				jacobians[1][0] = 0.0;
				jacobians[1][1] = 0.0;
				jacobians[1][2] = 0.0;
				jacobians[1][3] = 0.0;
				// printf("1 jacobians[1][0]=%f , jacobians[1][1]=%f , jacobians[1][2]=%f , jacobians[1][3]=%f\n",jacobians[1][0],jacobians[1][1],jacobians[1][2],jacobians[1][3]);
					// printf("KinematicsFunctorUAV: jacobians[1]=[%f,%f,%f,%f]\n", jacobians[1][0],jacobians[1][1],jacobians[1][2],jacobians[1][3]);
			}
			if(p3 < (double)fpf_){
				// printf("Entro: ");
				if (jacobians != NULL && jacobians[2] != NULL) 
				{
				// printf("entro: ");
					jacobians[2][0] = 0.0;
					jacobians[2][1] = 0.0;
					jacobians[2][2] = 0.0;
					jacobians[2][3] = 0.0;
					// printf("1 jacobians[2][0]=%f , jacobians[2][1]=%f , jacobians[2][2]=%f , jacobians[2][3]=%f\n",jacobians[2][0],jacobians[2][1],jacobians[2][2],jacobians[2][3]);
					// printf("KinematicsFunctorUAV: jacobians[2]=[%f,%f,%f,%f]\n", jacobians[2][0],jacobians[2][1],jacobians[2][2],jacobians[2][3]);
				}
			}
		}
		else{
			if(p1 > (double)fpi_){
				if (jacobians != NULL && jacobians[0] != NULL) 
				{
				// printf("entro: ");
					jacobians[0][0] = 0.0;
					jacobians[0][1] = -1.0*factor_*wf_*((x2 - x3)/(norm_vector1*norm_vector2) - ((2*x1 - 2*x2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*pow(arg1,3/2)*norm_vector2));
					jacobians[0][2] = -1.0*factor_*wf_*((y2 - y3)/(norm_vector1*norm_vector2) - ((2*y1 - 2*y2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*pow(arg1,3/2)*norm_vector2));
					jacobians[0][3] = -1.0*factor_*wf_*((z2 - z3)/(norm_vector1*norm_vector2) - ((2*z1 - 2*z2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*pow(arg1,3/2)*norm_vector2));
				// printf("2 jacobians[0][0]=%f , jacobians[0][1]=%f , jacobians[0][2]=%f , jacobians[0][3]=%f\n",jacobians[0][0],jacobians[0][1],jacobians[0][2],jacobians[0][3]);
					// printf("KinematicsFunctorUAV: jacobians[0]=[%f,%f,%f,%f]\n", jacobians[0][0],jacobians[0][1],jacobians[0][2],jacobians[0][3]);
				}
			}
			if (jacobians != NULL && jacobians[1] != NULL) 
			{
				// printf("entro: ");
				jacobians[1][0] = 0.0;
				jacobians[1][1] = -1.0*factor_*wf_*((x1 - 2*x2 + x3)/(norm_vector1*norm_vector2) + ((2*x1 - 2*x2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*pow(arg1,3/2)*norm_vector2) - ((2.0*x2 - 2.0*x3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*norm_vector1*pow(arg2,3/2)));
				jacobians[1][2] = -1.0*factor_*wf_*((y1 - 2*y2 + y3)/(norm_vector1*norm_vector2) + ((2*y1 - 2*y2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*pow(arg1,3/2)*norm_vector2) - ((2.0*y2 - 2.0*y3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*norm_vector1*pow(arg2,3/2)));
				jacobians[1][3] = -1.0*factor_*wf_*((z1 - 2*z2 + z3)/(norm_vector1*norm_vector2) + ((2*z1 - 2*z2)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*pow(arg1,3/2)*norm_vector2) - ((2.0*z2 - 2.0*z3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*norm_vector1*pow(arg2,3/2)));
				// printf("2 jacobians[1][0]=%f , jacobians[1][1]=%f , jacobians[1][2]=%f , jacobians[1][3]=%f\n",jacobians[1][0],jacobians[1][1],jacobians[1][2],jacobians[1][3]);
					// printf("KinematicsFunctorUAV: jacobians[1]=[%f,%f,%f,%f]\n", jacobians[1][0],jacobians[1][1],jacobians[1][2],jacobians[1][3]);
			}
			if(p3 < (double)fpf_){
				// printf("Entro: ");
				if (jacobians != NULL && jacobians[2] != NULL) 
				{
				// printf("entro: ");
					jacobians[2][0] = 0.0;
					jacobians[2][1] = -1.0*	-factor_*wf_*((x1 - x2)/(norm_vector1*norm_vector2) - ((2*x2 - 2*x3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*norm_vector1*pow(arg2,3/2)));
					jacobians[2][2] = -1.0*	-factor_*wf_*((y1 - y2)/(norm_vector1*norm_vector2) - ((2*y2 - 2*y3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*norm_vector1*pow(arg2,3/2)));
					jacobians[2][3] = -1.0*	-factor_*wf_*((z1 - z2)/(norm_vector1*norm_vector2) - ((2*z2 - 2*z3)*((x1 - x2)*(x2 - x3) + (y1 - y2)*(y2 - y3) + (z1 - z2)*(z2 - z3)))/(2.0*norm_vector1*pow(arg2,3/2)));
					// printf("2 jacobians[2][0]=%f , jacobians[2][1]=%f , jacobians[2][2]=%f , jacobians[2][3]=%f\n",jacobians[2][0],jacobians[2][1],jacobians[2][2],jacobians[2][3]);
					// printf("KinematicsFunctorUAV: jacobians[2]=[%f,%f,%f,%f]\n", jacobians[2][0],jacobians[2][1],jacobians[2][2],jacobians[2][3]);
				}
			}
		}
        // printf("KinematicsFunctorUAV[%.2f,%.2f,%.2f]: residuals[0] =[%f] , jacobians[0]=[%f,%f,%f,%f] jacobians[3]=[%f,%f,%f,%f] jacobians[2]=[%f,%f,%f,%f]\n",
		// p1,p2,p3,residuals[0],
		// jacobians[0][0],jacobians[0][1],jacobians[0][2],jacobians[0][3],
		// jacobians[1][0],jacobians[1][1],jacobians[1][2],jacobians[0][3],
		// jacobians[2][0],jacobians[2][1],jacobians[2][2],jacobians[0][3]);
		return true;
  	}

 double wf_, ang_, fpi_, fpf_;

 private:
};

#endif