#ifndef CERES_CONSTRAINS_CATENARY_HPP
#define CERES_CONSTRAINS_CATENARY_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "misc/near_neighbor.hpp"
#include "misc/marker_publisher.hpp"
#include "misc/catenary_solver_ceres.hpp"
#include "misc/grid3d.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class CatenaryFunctor{

public:
    CatenaryFunctor(){}
	

	struct ComputeDistanceObstacles 
    {
        ComputeDistanceObstacles (Grid3d *_grid_3D): _g_3D(_grid_3D)
        {
        }
        bool operator()(const double *point_, double *dist_) const 
        {
            bool is_into_ = _g_3D->isIntoMap(point_[0], point_[1], point_[2]);
			double d_[1];
            if (is_into_)
				d_[0] = _g_3D->getPointDist((double)point_[0], (double)point_[1], (double)point_[2]);
			else
				d_[0] = -1.0;

            dist_[0] = d_[0];

            return true;
        }
        Grid3d *_g_3D;
    };

	struct SolveCatenary
    {
        SolveCatenary()
        {
			points_catenary->clear();
        }
        bool operator()(const double *all_status_, double *ret_) const 
        {
			CatenarySolver cS;
			std::vector<geometry_msgs::Point> p_cat;
            
			cS.setMaxNumIterations(100);
			cS.solve(all_status_[0], all_status_[1], all_status_[2], all_status_[3], all_status_[4], all_status_[5], all_status_[6], p_cat);


			*points_catenary = p_cat;

			if (points_catenary->size() > 0.0)
				ret_[0]= 1.0;
			else
				ret_[0]= -1.0;

            return true;
        }
			std::vector<geometry_msgs::Point>* points_catenary;

	};

	struct ComputeCatenary 
	{
		ComputeCatenary(double weight_factor_1, double weight_factor_2, double weight_factor_3, double safty_bound, Grid3d *grid_3D_, 
					pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Point pos_reel_ugv, int size, 
					double pos_reel_z, Eigen::Vector3d fix_pos_ref, ros::NodeHandlePtr nhP)
					: wf_1(weight_factor_1), wf_2(weight_factor_2), wf_3(weight_factor_3),sb_(safty_bound), _g_3D(grid_3D_),
					kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),s_(size), pr_z_(pos_reel_z), fp_ref_(fix_pos_ref)
		{
			catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
			pr_ugv_[0] = pos_reel_ugv_.x;
			pr_ugv_[1] = pos_reel_ugv_.y;
			pr_ugv_[2] = pos_reel_ugv_.z;

			compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<3,1>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstacles,
                                                                        ceres::CENTRAL, 
                                                                        3,
                                                                        1>( 
                                    new ComputeDistanceObstacles(_g_3D))));

			compute_catenary.reset(new ceres::CostFunctionToFunctor<7,1>(
                                    new ceres::NumericDiffCostFunction<SolveCatenary,
                                                                        ceres::CENTRAL, 
                                                                        7,
                                                                        1>( 
                                    new SolveCatenary())));
		}

        template <typename T>
		bool operator()(const T* const statePosUAV, const T* const statePosUGV, const T* const stateCat, T* residual) const 
		{
			MarkerPublisher mP_;
			NearNeighbor nn;

			double d_obs_[1];
			double d_min[1] = {100.0}; //Initial nearest distance catenary to obstacle
			
			int id_marker_;		// Related with Marker frame.id
			T n_points_cat_dis;

			T d_max_below_z = T{0.0};

			visualization_msgs::MarkerArray catenary_marker_;

			T pos_init_cat_[3];
			pos_init_cat_[0] = statePosUGV[1]; 
			pos_init_cat_[1] = statePosUGV[2];
			pos_init_cat_[2] = statePosUGV[3] + pr_z_;

			T dist_ = sqrt((statePosUAV[1]-pos_init_cat_[0])*(statePosUAV[1]-pos_init_cat_[0]) + 
								(statePosUAV[2]-pos_init_cat_[1])*(statePosUAV[2]-pos_init_cat_[1]) + 
								(statePosUAV[3]-pos_init_cat_[2])*(statePosUAV[3]-pos_init_cat_[2])); 
			T safety_length = 1.02 * dist_;

			std::vector<geometry_msgs::Point> points_catenary;
			points_catenary.clear();
			
			T all_status[7] = {pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3], stateCat[1]};
			T ret_[1];
		    (*compute_catenary)(all_status, ret_);
			SolveCatenary S_cat;
			S_cat.
			S_cat.points_catenary;

			//  bool operator()(const double *all_status_, double *ret_) const 
			// {
			// 	CatenarySolver cS;
				
			// 	cS.setMaxNumIterations(100);
			// 	cS.solve(all_status_[0], all_status_[1], all_status_[2], all_status_[3], all_status_[4], all_status_[5], all_status_[6], points_catenary);
			// 	if (points_catenary.size() > 0.0)
			// 		ret_[0]= 1.0;
			// 	else
			// 		ret_[0]= -1.0;

			// 	return true;
			// }



			printf("size S_cat.points_catenary=[%lu]\n",S_cat.points_catenary->size());

			// cS.setMaxNumIterations(100);
			// cS.solve(pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3], stateCat[1], points_catenary);

			double size_ = 	points_catenary.size();

			// if (size_<1.0)
			// 	ROS_ERROR ("Not posible to get Catenary for state[%f] = [%f %f %f / %f %f %f]", statePosUAV[0], 
			// 																					statePosUGV[1], statePosUGV[2], statePosUGV[3], 
			// 																					statePosUAV[1], statePosUAV[2], statePosUAV[3]);

			// id_marker_ = statePosUAV[0];
			id_marker_ = 1;
					
			if (safety_length>= stateCat[1]){
				// ROS_ERROR ("state[%f] ,  Length_Catenary < dist_  ( [%f] < [%f] )",statePosUAV[0], stateCat[1], safety_length);
			}
			else
				mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, s_, catenary_marker_pub_);

			n_points_cat_dis = T{ceil(1.5*ceil(stateCat[1]))}; // parameter to ignore collsion points in the begining and in the end of catenary
			if (n_points_cat_dis < 5.0 )
				n_points_cat_dis = T{5.0};

			for (int i = 0 ; i < size_ ; i++){
				double d_[1];
				double p_cat_[3];
				T max_count_ = T{size_ - n_points_cat_dis/2.0};
				double count_i = i;
				if (count_i >= n_points_cat_dis && (count_i < max_count_ )){
					p_cat_[0] = points_catenary[i].x;
					p_cat_[1] = points_catenary[i].y;
					p_cat_[2] = points_catenary[i].z;
		            (*compute_nearest_distance)(p_cat_, d_);
					// nn.nearestObstacleStateCeres(kdT_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z, o_p_, near_[0], near_[1], near_[2]);
					// d_obs_[0] = sqrt((points_catenary[i].x-near_[0])*(points_catenary[i].x-near_[0])+
									// (points_catenary[i].y-near_[1])*(points_catenary[i].y-near_[1])+(points_catenary[i].z-near_[2])*(points_catenary[i].z-near_[2]));
					if (d_[0] < d_min[0]){
						d_min[0] = d_[0];
					}
				}
				double below_z[3];
				T d_below_z;
				nn.nearestObstacleStateCeres(kdT_trav_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z, pc_trav_, below_z[0], below_z[1], below_z[2]);
				if (points_catenary[i].z <= below_z[2]){
					d_below_z = T{sqrt((points_catenary[i].x-below_z[0])*(points_catenary[i].x-below_z[0])+(points_catenary[i].y-below_z[1])*(points_catenary[i].y- below_z[1])+(points_catenary[i].z-below_z[2])*(points_catenary[i].z-below_z[2]))};
					if (d_below_z > d_max_below_z)
						d_max_below_z = d_below_z; 
				}
			}
			T dist_uav_ref = T{sqrt( pow(fp_ref_.x()-statePosUAV[1],2) + pow(fp_ref_.y()-statePosUAV[2],2) + pow(fp_ref_.z()-statePosUAV[3],2))};

			if (d_min[0] > sb_)
				residual[0] = T{0.0};
			else{
				T Diff_ = T{sb_-d_min[0]} ;
				residual[0] = wf_1 * 100.0 * (10.0*dist_uav_ref) * (exp(10.0*Diff_) - 1.0);
			}	
			// Constraint to make length Cable longer than 
			if (stateCat[1] > safety_length)
				residual[1] = T{0.0};
			else
				residual[1] = wf_2 *100.0 * (exp(10.0*(safety_length - stateCat[1])) - 1.0);

			residual[2] = wf_3 * 1000.0 * (exp(10.0* d_max_below_z) - 1.0);

			return true;
		}

		double wf_1, wf_2, wf_3, sb_, pr_z_;
		int s_;
		geometry_msgs::Point pos_reel_ugv_;
		double pr_ugv_[3];
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
		pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_;
		pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_;
		Eigen::Vector3d fp_ref_;

		ros::Publisher catenary_marker_pub_;
		Grid3d *_g_3D;

        std::unique_ptr<ceres::CostFunctionToFunctor<3,1> > compute_nearest_distance;
        std::unique_ptr<ceres::CostFunctionToFunctor<7,1> > compute_catenary;
	};

	


private:

};
#endif