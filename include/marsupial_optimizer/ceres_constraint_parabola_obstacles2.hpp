#ifndef CERES_CONSTRAINS_PARABOLA_OBSTACLES_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABOLA_OBSTACLES_AUTODIFF_HPP

#include "ceres/ceres.h"
#include "ceres/cost_function_to_functor.h"

#include "glog/logging.h"
#include "Eigen/Core"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "misc/marker_publisher.h"

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include <iostream>
#include <fstream>
#include <string>

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceFunctionObstacles : public ceres::SizedCostFunction<2, 3> 
{
 public:

	DistanceFunctionObstacles(Grid3d* grid_)
      : g_3D(grid_)
    {
    }

    DistanceFunctionObstacles(Grid3d* grid_obst_ , Grid3d* grid_trav_, double safety_distance_)
      : g_3D_obst(grid_obst_), g_3D_trav(grid_trav_), sd(safety_distance_)
    {
    }

    virtual ~DistanceFunctionObstacles(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double d1_, d2_, distance_;
		double min_dist_ = 0.005;
		double type_coll_; // -1 if collision with the ground , 1 if collision with another obstacle different than ground

		if(g_3D_obst->isIntoMap(x, y, z))
        {
			TrilinearParams p1 = g_3D_trav->getPointDistInterpolation(x, y, z);
            d1_ = p1.a0 + p1.a1*x + p1.a2*y + p1.a3*z + p1.a4*x*y + p1.a5*x*z + p1.a6*y*z + p1.a7*x*y*z;
			TrilinearParams p2 = g_3D_obst->getPointDistInterpolation(x, y, z);
            d2_ = p2.a0 + p2.a1*x + p2.a2*y + p2.a3*z + p2.a4*x*y + p2.a5*x*z + p2.a6*y*z + p2.a7*x*y*z;

			if (d1_ < d2_){
				distance_  = d1_;
				type_coll_ = -1.0;
			}else{
				distance_ = d2_;
				type_coll_ = 1.0;
			}

			if(distance_ > sd)
				type_coll_ = 0.0;
			if(distance_ < min_dist_)
				distance_ = min_dist_;

			residuals[0] = distance_;
			residuals[1] = type_coll_;

            if (d1_ < d2_) 
            {
				if(jacobians != NULL && jacobians[0] != NULL){
					jacobians[0][0] = p1.a1 + p1.a4*y + p1.a5*z + p1.a7*y*z;
					jacobians[0][1] = p1.a2 + p1.a4*x + p1.a6*z + p1.a7*x*z;
					jacobians[0][2] = p1.a3 + p1.a5*x + p1.a6*y + p1.a7*x*y;
				}
				else if(jacobians != NULL && jacobians[1] != NULL){
					jacobians[1][0] = 0.0;
					jacobians[1][1] = 0.0;
					jacobians[1][2] = 0.0;
				}
            }
			else if (d1_ >= d2_) 
            {
				if(jacobians != NULL && jacobians[0] != NULL){
					jacobians[0][0] = p2.a1 + p2.a4*y + p2.a5*z + p2.a7*y*z;
					jacobians[0][1] = p2.a2 + p2.a4*x + p2.a6*z + p2.a7*x*z;
					jacobians[0][2] = p2.a3 + p2.a5*x + p2.a6*y + p2.a7*x*y;
				}
				else if(jacobians != NULL && jacobians[1] != NULL){
					jacobians[1][0] = 0.0;
					jacobians[1][1] = 0.0;
					jacobians[1][2] = 0.0;
				}
            }
		}
        else
        {
			distance_ = -1.0;
			type_coll_ = -1.0;
			residuals[0] = distance_;
            residuals[1] = type_coll_;
			if (jacobians != NULL && jacobians[0] != NULL && jacobians[1] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
            }
        }

        return true;
  }
  private:
	Grid3d *g_3D, *g_3D_obst, *g_3D_trav;
	double sd;
};

class NumPointParabola : public ceres::SizedCostFunction<1, 1> 
{
 public:

    NumPointParabola(void)
    {
    }

    virtual ~NumPointParabola(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
		int num_point_per_unit_length = 20;
		
		residuals[0] = round( (double)num_point_per_unit_length * parameters[0][0] );
        if (jacobians != NULL && jacobians[0] != NULL) 
        	jacobians[0][0] = num_point_per_unit_length;

        return true;
  }
  private:

};

class AutodiffParableFunctor {

	public:
    AutodiffParableFunctor(){}

		struct ParableFunctor 
		{
			ParableFunctor(double weight_factor_, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			: wf(weight_factor_), g_3D(grid_3D_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), 
			_parableCostFunctor(new DistanceFunctionObstacles(g_3D)), _numPointFunctor(new NumPointParabola())
			{
			}

			ParableFunctor(double weight_factor_, Grid3d* grid_3D_obst_, Grid3d* grid_3D_trav_, geometry_msgs::Vector3 pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			: wf(weight_factor_), g_3D_obst(grid_3D_obst_), g_3D_trav(grid_3D_trav_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), 
			_parableCostFunctor(new DistanceFunctionObstacles(g_3D_obst, g_3D_trav, sb)), _numPointFunctor(new NumPointParabola())
			{

			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				double min_val_ = 0.005;
				T d_[1];
				T np_; 
				T u_x , u_y;
				bool x_const, y_const;
				x_const = y_const = false;

				T delta_x = (pUAV[1] - ugv_reel[1]);
				T delta_y = (pUAV[2] - ugv_reel[2]);
				T delta_z = (pUAV[3] - ugv_reel[3]);
				T dist_ = sqrt(delta_x * delta_x + delta_y * delta_y );
				if (dist_ < min_val_){
					u_x = u_y = T{0.0};
					d_[0] = sqrt(delta_z*delta_z); 
				} else{
					u_x = delta_x /dist_;
					u_y = delta_y /dist_;
					d_[0] = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z*delta_z);
				}
				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in parable
					d_[0] = T{1.0};

        		_numPointFunctor(d_, &np_); // To get the values and parameters needed for computing the parable interpolation

				// Here is compute the parable point and it cost 
				T point[3];
				T data_obst_[2];	
				T x_  =  T{0.0};
				vector<T> v_dist, v_coll; v_dist.clear(); v_coll.clear();
				vector<T> v_p1_, v_p2_, v_p3_; v_p1_.clear(), v_p2_.clear(), v_p3_.clear();

				for(int i = 0; i < np_; i++, x_ += dist_/ np_ ){  
					if (!(dist_ < min_val_)){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
						point[0] = ugv_reel[1] + u_x * x_;
						point[1] = ugv_reel[2] + u_y * x_;
						point[2] = param[1] * x_* x_ + param[2] * x_ + param[3];
					}
					else{ 	// In case that UGV and UAV are not in the same point the plane X-Y
						T _step = d_[0] / np_;
						point[0] = ugv_reel[1];
						point[1] = ugv_reel[2];
						point[2] = ugv_reel[3]+ _step* (double)i;    	
					}
        			_parableCostFunctor(point, data_obst_);
					v_dist.push_back(data_obst_[0]);
					v_coll.push_back(data_obst_[1]);
					v_p1_.push_back(point[0]);
					v_p2_.push_back(point[1]);
					v_p3_.push_back(point[2]);
				}

				int first_collision = -1;
				int last_collision = -1;
				for(int i = 0; i < np_; i++){ 
					std::cout << "	v_coll["<< i <<"]:"<< v_coll[i]<< std::endl;
					if (v_coll[i]==T{1.0}){
						if (first_collision ==-1)
							first_collision = i;
						last_collision = i;
					}
				}
std::cout << "	Collision between["<< first_collision <<"-"<< last_collision <<"]"<< std::endl;


				T distance_, div_, C, point_cost_;
				T cost_state_parabola = T{0.0};
				for(int i = 0; i < np_; i++){ 
					distance_ = v_dist[i];

					// div_ = T{1.0}/distance_;
					// if (v_coll[i] != T{0.0}){ // 0.0 = NOT COLLISION ; -1.0 = GROUND COLLISION ; 1.0 = OTHER COLLISION
					// 	if(v_dist[i] < T{0.0}){
					// 		C = T{-100000.0};
					// 		div_ = v_p3_[i];
					// 		if (v_p3_[i] > T{-min_val_})
					// 			div_ = T{-min_val_};
					// 	}
					// 	else
					// 		C = T{20.0};
					// }else
					// 	C = T{1.0};	
					// point_cost_ = div_*C;

					div_ = T{1.0}/distance_;
					if (distance_ < T{sb})
						C = T{100.0};
					else
						C = T{1.0};	
					point_cost_ = div_*C;
if (w_d_)
std::cout << "["<<i<<"]cost:" << point_cost_ <<" d:" << distance_ <<" C:" << C << std::endl;
// std::cout <<"	Point:["<< v_p1_[i]<<","<< v_p2_[i]<< ","<< v_p3_[i]<<"]"<<std::endl;
					cost_state_parabola = cost_state_parabola + point_cost_; // To get point parable cost
				}


				cost_state_parabola = cost_state_parabola/np_;

				residual[0] = wf * cost_state_parabola;
if (w_d_){
std::cout << "	Obs["<< param[0]<<"] : R[0]:"  << residual[0] << " P:["<< param[1]<<"/ "  << param[2] <<"/ "  << param[3] <<"]"<< std::endl ;
}			
				return true;
			}

			bool w_d_;
			double wf, sb;
			geometry_msgs::Vector3 pos_reel_ugv;
			std::string user;
			Grid3d *g_3D, *g_3D_obst, *g_3D_trav;
	    	ceres::CostFunctionToFunctor<2, 3> _parableCostFunctor;
	    	ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};
	private:
};

#endif