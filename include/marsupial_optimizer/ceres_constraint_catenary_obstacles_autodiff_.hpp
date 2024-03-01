#ifndef CERES_CONSTRAINS_CATENARY_OBSTACLE_AUTODIFF_HPP
#define CERES_CONSTRAINS_CATENARY_OBSTACLE_AUTODIFF_HPP

#include "ceres/ceres.h"
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


class DistanceFunction : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceFunction(Grid3d* grid_, double security_dist_)
      : g_3D(grid_), sb(security_dist_)
    {
    }

    virtual ~DistanceFunction(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double dist_, div;
		double C;

		if(g_3D->isIntoMap(x, y, z))
        {
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
			if(dist_ < 0.0001)
				dist_ = 0.0001;
			div = 1/dist_;
			if (dist_ < sb)
				C = 10.0;
			else
				C = 4.0;	
			residuals[0] = div*C;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = -C*(p.a1 + p.a4*y + p.a5*z + p.a7*y*z)*div*div;
                jacobians[0][1] = -C*(p.a2 + p.a4*x + p.a6*z + p.a7*x*z)*div*div;
                jacobians[0][2] = -C*(p.a3 + p.a5*x + p.a6*y + p.a7*x*y)*div*div;
            }
        }
        else
        {
			C = 100000.0;
			residuals[0] = C*z;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = C;
            }
        }

        return true;
  }

  private:
	Grid3d* g_3D;
	double sb;
};

class NumPointFunction : public ceres::SizedCostFunction<1, 1> 
{
 public:

    NumPointFunction(void)
    {
    }

    virtual ~NumPointFunction(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
		int num_point_per_unit_length = 10;
		
		residuals[0] = round( (double)num_point_per_unit_length * parameters[0][0] );
        if (jacobians != NULL && jacobians[0] != NULL) 
        	jacobians[0][0] = num_point_per_unit_length;

        return true;
  }

  private:

};

class AutodiffTetherFunctor {

public:
    AutodiffTetherFunctor(){}

	struct BisectionTether 
	{
		BisectionTether (ros::NodeHandlePtr nhP_, Grid3d* g_3D_, octomap::OcTree* o_full_, double sb_, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_, 
		pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_, pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_, int size_, float map_resolution, bool use_dist_func_)
		: nhP(nhP_), g_3D(g_3D_), o_full(o_full_), sb(sb_), kdT(kdT_), o_p(o_p_), kdT_trav(kdT_trav_), pc_trav(pc_trav_), size(size_), m_r_(map_resolution), use_dist_func(use_dist_func_)
		{
			catenary_marker_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
			plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("plane_markerArray", 1);
			obs_plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("obsplane_markerArray", 1);
		}
		bool operator()(const double *state1,const double *state2, const double *state2f,const double *Length, double *Tether) const 
		{
			MarkerPublisher mP_;
			bisectionCatenary bc;
			NearNeighbor nn;

			visualization_msgs::MarkerArray catenary_marker_, plane_marker_, obs_plane_marker_;
			std::vector<geometry_msgs::Vector3> points_catenary;
			std::vector<double> dist_obst_cat; 
			std::vector<int> cat_between_obs, pos_cat_in_coll;
			geometry_msgs::Vector3 min_point_z_cat;

			int id_marker_ = state2[0];		// Related with Marker frame.id
			double min_val_proximity_ = 0.04;
			// double min_val_proximity_ = sb;
			double cost_cat = 0.0;
			int first_coll, last_coll;

			points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
			bool just_one_axe = bc.configBisection(Length[1], state1[1], state1[2], state1[3], state2[1], state2[2], state2[3]);
			bc.getPointCatenary3D 	(points_catenary, false);
			bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);
			double min_distance_cat_obs_ = bc.min_distance_cat_obs;

			mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size, catenary_marker_pub_);
	
			if (( (state2f[1]-state2[1]) != 0.0 || (state2f[2]-state2[2]) != 0.0 || (state2f[3]-state2[3]) != 0.0) && first_coll!= 0){
				octomap::point3d r_;
				octomap::point3d s_(state2f[1], state2f[2], state2f[3]); //start for rayCast
				octomap::point3d d_(state2[1]-state2f[1] , state2[2]-state2f[2] , state2[3]-state2f[3]); //direction for rayCast
				bool r_cast_coll = false; 
				r_cast_coll =  o_full->castRay(s_, d_, r_);
				double dist_b_uav = sqrt(pow(state2[1]-s_.x(),2)+pow(state2[2]-s_.y(),2)+pow(state2[3]-s_.z(),2));
				double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
				if(r_cast_coll && distObs_ <= dist_b_uav )
					last_coll = points_catenary.size();
			}

			double x_, y_, z_, cost_ , add_extracost;
			double d_below_z_trav = 0.0;
			geometry_msgs::Vector3 n_coll_cat, point_coll_trav;
			geometry_msgs::Vector3 p_;
			int is_collision = 0;
			int count_coll = 0;
			for (size_t i = 0; i < points_catenary.size(); i++){
				p_.x = m_r_ * round( (points_catenary[i].x) * 1/m_r_);
				p_.y = m_r_ * round( (points_catenary[i].y) * 1/m_r_);
				p_.z = m_r_ * round( (points_catenary[i].z) * 1/m_r_);
				// bool _is_into_ = g_3D->isIntoMap((double)p_.x, (double)p_.y, (double)p_.z);
				double d_obs;
				if(use_dist_func){
					bool is_into_ = g_3D->isIntoMap((double)p_.x,(double)p_.y,(double)p_.z);
					if (is_into_){
						TrilinearParams d = g_3D->getPointDistInterpolation((double)p_.x, (double)p_.y, (double)p_.z);
						x_ = p_.x;
						y_ = p_.y;
						z_ = p_.z;
						d_obs= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);
					}
					else
						d_obs = -1.0;
				}
				else{
					geometry_msgs::Vector3 near_;
            		nn.nearestObstacleStateCeres(kdT , p_.x,p_.y,p_.z, o_p, near_.x, near_.y, near_.z);
					d_obs = sqrt(pow(near_.x-p_.x,2) + pow(near_.y-p_.y,2) + pow(near_.z-p_.z,2));
				}

				if((i >= first_coll && i <= last_coll && i > 0) ){
					cost_ = (1.0/min_val_proximity_)*2.0;
					is_collision = 1;
					count_coll++;
				}
				else if (d_obs < sb){ 
					if (d_obs < min_val_proximity_)
						d_obs = min_val_proximity_;
					// cost_ = (1.0/d_obs)*1.0;
					cost_ = (1.0/min_distance_cat_obs_)*2.0;
					// cost_ = (1.0/min_val_proximity_)*1.0;
					is_collision = 2;
					count_coll ++;
				}
				else
					cost_ = (1.0/d_obs)*1.0;
					
				cost_cat = cost_cat + cost_;
			}
			Tether[0] = cost_cat;
			Tether[1] = first_coll;
			Tether[2] = last_coll;
			// Tether[3] = last_coll-first_coll;
			Tether[3] = count_coll;
			Tether[4] = points_catenary.size();
			Tether[5] = bc.num_point_per_unit_length;

			return true;
		}
		ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
		ros::NodeHandlePtr nhP;

		octomap::OcTree* o_full;
		bool use_dist_func;
		float m_r_;
		double sb;
		int size;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT, kdT_trav;
		pcl::PointCloud <pcl::PointXYZ>::Ptr o_p, pc_trav;
		Grid3d* g_3D;
	};

	struct TetherFunctor 
	{
	TetherFunctor(double weight_factor_1, double weight_factor_3, double safty_bound, float min_length_cat, double length_tether_max_,
					pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
					pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Vector3 pos_reel_ugv, int size, 
					geometry_msgs::Vector3 fix_pos_ref, ros::NodeHandlePtr nhP_, octomap::OcTree* octotree_full_, float map_resolution, bool write_data, 
					bool use_dist_func_, std::string user_name)
					: wf_1(weight_factor_1), sb_(safty_bound), m_L_c_(min_length_cat), kdT_(kdT_From_NN), 
					o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),size_(size), 
					fp_ref(fix_pos_ref), o_full_(octotree_full_), m_r_(map_resolution), w_d_(write_data), use_dist_func(use_dist_func_), user_(user_name)
		{
			nhP = nhP_;
			
			bisection_catenary.reset(new ceres::CostFunctionToFunctor<6,4,4,4,2>(
							  new ceres::NumericDiffCostFunction<BisectionTether, ceres::CENTRAL,6,4,4,4,2>
							  (new BisectionTether(nhP_, g_3D_, o_full_, sb_, kdT_, o_p_, kdT_trav_, pc_trav_, size_, m_r_, use_dist_func))));
		}

		template <typename T>
		// bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const stateCat, T* residual) const 
		// {
			/*----------------------  Define Variables  ----------------------*/

		bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const param, T* residual) const 
		{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				T d_[1];
				T np_; 
				T u_x , u_y;
				T fix_value = T{0.01};
				bool x_const, y_const;
				x_const = y_const = false;

				T delta_x = (pUAV[1] - ugv_reel[1]);
				T delta_y = (pUAV[2] - ugv_reel[2]);
				T dist_ = sqrt(delta_x * delta_x + delta_y * delta_y );
				if (dist_ < 0.0001){
					u_x = u_y = T{0.0};
					d_[0] = sqrt(pow(pUAV[3]-ugv_reel[3],2)); 
				} else{
					u_x = delta_x /dist_;
					u_y = delta_y /dist_;
					d_[0] = dist_; 
				}
				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in parable
					d_[0] = T{1.0};

        		_numPointFunctor(d_, &np_); // To get the values and parameters needed for computing the parable interpolation

				// Here is compute the parable point and it cost 
				T point[3];
				T parable_cost_;	
				T cost_state_parable = T{0.0};
				T x_  =  T{0.0};
				T tetha_;
				for(int i = 0; i < np_; i++, x_ += d_[0]/ np_ ){  
					if (!(dist_ < 0.0001)){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
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
        			_parableCostFunctor(point, &parable_cost_);

					cost_state_parable = cost_state_parable + parable_cost_; // To get point parable cost
				}
				residual[0] = wf * 100.0 * cost_state_parable;
					
				return true;
			}

			bool w_d_;
			double wf, sb;
			geometry_msgs::Vector3 pos_reel_ugv;
			std::string user;
			Grid3d* g_3D;
	    	ceres::CostFunctionToFunctor<1, 3> _parableCostFunctor;
	    	ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};

private:

};

#endif