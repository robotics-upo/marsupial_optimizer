#ifndef CERES_CONSTRAINS_OBSTACLES_AUTODIFF_UGV_HPP
#define CERES_CONSTRAINS_OBSTACLES_AUTODIFF_UGV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/bisection_catenary_3D.h"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include <iostream>
#include <fstream>
#include <string>


class ComputeDistanceObstaclesUGV : public ceres::SizedCostFunction<1, 3> 
{
 public:

	ComputeDistanceObstaclesUGV(Grid3d *grid_, double safety_distance_)
      : g_3D(grid_), sd(safety_distance_)
    {
    }

    virtual ~ComputeDistanceObstaclesUGV(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double distance_;
		double min_dist_ = 0.01;

		if(g_3D->isIntoMap(x, y, z)){
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            distance_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;

			if(distance_ < min_dist_)
				distance_ = min_dist_;
	
			residuals[0] = distance_;

			if(jacobians != NULL && jacobians[0] != NULL){
				jacobians[0][0] = p.a1 + p.a4*y + p.a5*z + p.a7*y*z;
				jacobians[0][1] = p.a2 + p.a4*x + p.a6*z + p.a7*x*z;
				jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
			}
		}
        else{
			distance_ = -1.0;
			residuals[0] = distance_;
			if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
            }
        }

        return true;
  }
  private:
	Grid3d *g_3D;
	double sd;
};

class ObstacleDistanceFunctorUGV {

public:
    ObstacleDistanceFunctorUGV(){}
    
    struct ObstaclesFunctorUGV 
    {
        ObstaclesFunctorUGV(double weight_factor, double safty_bound, Grid3d* grid_3D_, bool write_data, std::string user_name)
        : wf_(weight_factor), sb_(safty_bound), g_3D(grid_3D_), w_d_(write_data), user_(user_name),
          compute_nearest_distance(new ComputeDistanceObstaclesUGV(g_3D, sb_))
        {
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T d_, arg_d_[1];

            compute_nearest_distance(statePos1, arg_d_);
            
            if (arg_d_ < 0.001)
                d_ = T{0.0};
            else
                d_ = sqrt(arg_d_);

            T d_sb_ = sb_ * T{1.05};
            T max_value_residual = T{100.0};
            T min_value_residual = T{0.0};
            T max_value_dependent = T{0.0};
            T m ;
            if (d_ > d_sb_ || statePos1[1] > n1_[2]-T{0.6}) // 6 cm is the radius of the wheel
                m = T{0.0};
            else
                m = (max_value_residual- min_value_residual)/(max_value_dependent - d_sb_);
            
            residual[0] = wf_ * m *(d_ - d_sb_);

	        if(w_d_){
                std::ofstream ofs;
                std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/obstacles_ugv.txt";
                ofs.open(name_output_file.c_str(), std::ofstream::app);
                if (ofs.is_open()) 
                    ofs << residual[0] << "/" <<std::endl;
                ofs.close();
            }

            return true;
        }

        bool w_d_;
	    ceres::CostFunctionToFunctor<1, 3> compute_nearest_distance;
		Grid3d *g_3D;
        double wf_, sb_;
        std::string user_;
    };


private:

};

#endif