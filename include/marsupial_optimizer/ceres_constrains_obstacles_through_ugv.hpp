#ifndef CERES_CONSTRAINS_OBSTACLES_THROUGH_UGV_HPP
#define CERES_CONSTRAINS_OBSTACLES_THROUGH_UGV_HPP

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


class ObstacleDistanceThroughFunctorUGV {

public:
    ObstacleDistanceThroughFunctorUGV(){}
    
    struct RayCastingThroughUGV 
    {
        RayCastingThroughUGV (octomap::OcTree* o_tree_): o_t_(o_tree_)
        {
    
        }

        bool operator()(const double *state1, const double *state2, double *end_) const 
        {
	        octomap::point3d r_;
			octomap::point3d s_(state1[1] , state1[2] , 0.4+state1[3] ); //direction for rayCast
			octomap::point3d d_(state2[1]-state1[1] , state2[2]-state1[2] , 0.4+(state2[3]-state1[3]) ); //direction for rayCast
            bool r_cast_coll = o_t_->castRay(s_, d_, r_);
            end_[0]= r_.x();
            end_[1]= r_.y();
            end_[2]= r_.z();

            if (r_cast_coll)
                end_[3] = 1.0; // in case rayCast with collision
            else
                end_[3] = -1.0;  // in case rayCast without collision

            return true;
        }

        octomap::OcTree *o_t_;
    };

    struct ObstaclesThroughFunctorUGV 
    {
        ObstaclesThroughFunctorUGV(double weight_factor, octomap::OcTree* octotree_): wf_(weight_factor), oc_tree_(octotree_)
        {         
            ray_casting.reset(new ceres::CostFunctionToFunctor<4,4,4>(
                                    new ceres::NumericDiffCostFunction<RayCastingThroughUGV, ceres::CENTRAL,4,4,4>( 
                                    new RayCastingThroughUGV(oc_tree_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
        {
            T end_ray_12[4];
			T dw12_, do12_, d12_; 
            T arg_dw12_, arg_do12_;

            arg_dw12_ = (statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1])+
                        (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2])+
                        (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]);
            if (arg_dw12_ < 0.0001 && arg_dw12_ > -0.0001)
                dw12_ = T{0.0};
            else
                dw12_ = sqrt(arg_dw12_);

            if (dw12_ < 0.001){
                residual[0] = T{0.0} ;
                // printf("ObstaclesThroughFunctorUGV , ");
                // std::cout << "IF residual[0]= " << residual[0] << std::endl;
			    // std::cout << "statePos1[0]= " << statePos1[0] << " , statePos2[0]= " << statePos2[0] << std::endl;
                return true;
            }
            else{
                (*ray_casting)(statePos1, statePos2, end_ray_12);
                
                arg_do12_ = (statePos1[1]-end_ray_12[0])*(statePos1[1]-end_ray_12[0])+
                            (statePos1[2]-end_ray_12[1])*(statePos1[2]-end_ray_12[1])+
                            (statePos1[3]-end_ray_12[2])*(statePos1[3]-end_ray_12[2]);
                if (arg_do12_ < 0.0001 && arg_do12_ > -0.0001)
                    do12_ = T{0.0};
                else
                    do12_ = sqrt(arg_do12_);


                if ( (dw12_<= do12_) ){
                    residual[0] = T{0.0} ;
                }
                else{
                    d12_ = (dw12_ - do12_);
                
                residual[0] = wf_ * 100.0 * (exp(2.0*d12_)-1.0) ;
                }

                // printf("ObstaclesThroughFunctorUGV , ");
                // std::cout << "dw12_= " << dw12_ << " , do12_=" << do12_ << std::endl;
                // std::cout << "end_ray_12[3]= " << end_ray_12[3] << std::endl;
                // std::cout << "ELSE residual[0]= " << residual[0] << std::endl;
			    // std::cout << "statePos1[0]= " << statePos1[0] << " , statePos2[0]= " << statePos2[0] << std::endl;

                return true;
            }
        }

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4,4> > ray_casting;
        double wf_;
        octomap::OcTree *oc_tree_;
    };

private:

};

#endif