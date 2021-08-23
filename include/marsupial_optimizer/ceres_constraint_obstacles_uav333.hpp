#ifndef CERES_CONSTRAINS_OBSTACLES_UAV_HPP
#define CERES_CONSTRAINS_OBSTACLES_UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "misc/near_neighbor.hpp"
#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class ObstacleDistanceFunctorUAV {

public:
    ObstacleDistanceFunctorUAV(){}
    
    struct ComputeDistanceObstaclesUAV 
    {
        ComputeDistanceObstaclesUAV (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *state1, double *near_) const 
        {
            NearNeighbor nn;

            nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2], state1[3], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

        double f_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

    struct RayCastingThroughUAV 
    {
        RayCastingThroughUAV (octomap::OcTree* o_tree_): o_t_(o_tree_)
        {
    
        }

        bool operator()(const double *state1, const double *state2, double *end_) const 
        {
	        octomap::point3d r_;
			octomap::point3d s_(state1[1] , state1[2] , state1[3] ); //direction for rayCast
			octomap::point3d d_(state2[1]-state1[1] , state2[2]-state1[2] , state2[3]-state1[3] ); //direction for rayCast
            bool r_cast_coll = o_t_->castRay(s_, d_, r_);
            end_[0]= r_.x();
            end_[1]= r_.y();
            end_[2]= r_.z();

            if (r_cast_coll)
                end_[3] = -1.0; // in case rayCast with collision
            else
                end_[3] = 1.0;  // in case rayCast without collision

            return true;
        }

        octomap::OcTree *o_t_;
    };


    struct ObstaclesFunctor 
    {
        ObstaclesFunctor(double weight_factor, double safty_bound, 
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, octomap::OcTree* octotree_)
            : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points), oc_tree_(octotree_)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstaclesUAV(kdT_, o_p_))));
            
            ray_casting.reset(new ceres::CostFunctionToFunctor<4,4,4>(
                                    new ceres::NumericDiffCostFunction<RayCastingThroughUAV, ceres::CENTRAL,4,4,4>( 
                                    new RayCastingThroughUAV(oc_tree_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
        {
            // To avoid obstacles
            T d_ugv_, d_uav_1, d_uav_2;
            T arg_d_uav_1, arg_d_uav_2;
            T arg_dw12_, arg_do1_, arg_do2_; 
            T n1_[4];
            T n2_[4];
            // To avoid obstacle between two waypoints
			T dw12_, do1_, do2_, d12_;
            T end_ray_1_obs[4];    
            T end_ray_2_obs[4];    
            
            (*compute_nearest_distance)(statePos1, n1_);
            (*compute_nearest_distance)(statePos2, n2_);
            arg_d_uav_1 = (statePos1[1]-n1_[0])*(statePos1[1]-n1_[0]) + (statePos1[2]-n1_[1])*(statePos1[2]-n1_[1]) + (statePos1[3]-n1_[2])*(statePos1[3]-n1_[2]);
            arg_d_uav_2 = (statePos2[1]-n2_[0])*(statePos2[1]-n2_[0]) + (statePos2[2]-n2_[1])*(statePos2[2]-n2_[1]) + (statePos2[3]-n2_[2])*(statePos2[3]-n2_[2]);
            
            if (arg_d_uav_1 < 0.0001 && arg_d_uav_1 > -0.0001)
                d_uav_1 = T{0.0};
            else
                d_uav_1 = sqrt(arg_d_uav_1);
            if (arg_d_uav_2 < 0.0001 && arg_d_uav_2 > -0.0001)
                d_uav_2 = T{0.0};
            else
                d_uav_2 = sqrt(arg_d_uav_2);

            (*ray_casting)(statePos1, statePos2, end_ray_1_obs);
            (*ray_casting)(statePos2, statePos1, end_ray_2_obs);
            
            
            
            arg_dw12_ = (statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1])+
                        (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2])+
                        (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]);
            if (arg_dw12_ < 0.0001 && arg_dw12_ > -0.0001)
                dw12_ = T{0.0};
            else
                dw12_ = sqrt(arg_dw12_);

            // Compute distance to Obstacles    
            arg_do1_ =  (statePos1[1]-end_ray_1_obs[0])*(statePos1[1]-end_ray_1_obs[0])+
                        (statePos1[2]-end_ray_1_obs[1])*(statePos1[2]-end_ray_1_obs[1])+
                        (statePos1[3]-end_ray_1_obs[2])*(statePos1[3]-end_ray_1_obs[2]);
            if (arg_do1_ < 0.0001 && arg_do1_ > -0.0001)
                do1_ = T{0.0};
            else
                do1_ = sqrt(arg_do1_);
            
            arg_do2_ =  (statePos2[1]-end_ray_2_obs[0])*(statePos2[1]-end_ray_2_obs[0])+
                        (statePos2[2]-end_ray_2_obs[1])*(statePos2[2]-end_ray_2_obs[1])+
                        (statePos2[3]-end_ray_2_obs[2])*(statePos2[3]-end_ray_2_obs[2]);
            if (arg_do2_ < 0.0001 && arg_do2_ > -0.0001)
                do2_ = T{0.0};
            else
                do2_ = sqrt(arg_do2_);


            T arg_diffZ = (statePos2[3]-statePos1[3])*(statePos2[3]-statePos1[3]);
            T diffZ;
            if (arg_diffZ < 0.0001 && arg_diffZ > -0.0001)
                diffZ = T{0.0};
            else
                diffZ =  sqrt(arg_diffZ);

            if ( (dw12_ >= do1_) && diffZ < 0.01){  
                d12_ = (dw12_ - do1_);
            }
			else{
                d12_ = T{0.0};
                do1_ = T{0.0};
                do2_ = T{0.0};
            }
				
            residual[0] = wf_ *( exp(4.0*(sb_ - d_uav_1)) + exp(4.0*(sb_ - d_uav_2)) + exp(0.5*(d_uav_1-3.0)) + exp(0.5*(d_uav_2-3.0)));
            residual[1] = wf_ *( log(1.0+exp(do1_-2.0))+ log(1.0+exp(do2_-2.0)) );

            return true;
        }

        double wf_, sb_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4,4> > ray_casting;
        octomap::OcTree *oc_tree_;
    };


private:

};

#endif