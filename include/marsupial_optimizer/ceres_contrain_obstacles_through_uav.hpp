#ifndef CERES_CONTRAIN_OBSTACLES_THROUGH_UAV_HPP
#define CERES_CONTRAIN_OBSTACLES_THROUGH_UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include "misc/near_neighbor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class ObstacleDistanceThroughFunctorUAV {

public:
    ObstacleDistanceThroughFunctorUAV(){}
    
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

    struct ObstaclesThroughFunctor 
    {
        ObstaclesThroughFunctor(double weight_factor, octomap::OcTree* octotree_): wf_(weight_factor), oc_tree_(octotree_)
        {         
            ray_casting.reset(new ceres::CostFunctionToFunctor<4,4,4>(
                                    new ceres::NumericDiffCostFunction<RayCastingThroughUAV, ceres::CENTRAL,4,4,4>( 
                                    new RayCastingThroughUAV(oc_tree_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
        {
            T end_ray_12[4];
			T dw12_, do12_, d12_; 

            (*ray_casting)(statePos1, statePos2, end_ray_12);
           
            dw12_ = sqrt((statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1])+
                         (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2])+
                         (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]));

            do12_ = sqrt((statePos1[1]-end_ray_12[0])*(statePos1[1]-end_ray_12[0])+
                         (statePos1[2]-end_ray_12[1])*(statePos1[2]-end_ray_12[1])+
                         (statePos1[3]-end_ray_12[2])*(statePos1[3]-end_ray_12[2]));

            if ( (dw12_<= do12_) && (end_ray_12[3]==1.0) )
                d12_ = T{0.0};
			else
                d12_ = (dw12_ - do12_);
				
            residual[0] = wf_ * d12_ ;

            return true;
        }

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4,4> > ray_casting;
        double wf_;
        octomap::OcTree *oc_tree_;
    };

private:

};

#endif