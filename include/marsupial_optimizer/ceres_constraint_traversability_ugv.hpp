#ifndef CERES_CONSTRAINS_TRAVERSABILITY_UGV_HPP
#define CERES_CONSTRAINS_TRAVERSABILITY_UGV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include <iostream>
#include <fstream>
#include <string>

class TraversabilityDistanceFunctorUGV {

public:
    TraversabilityDistanceFunctorUGV(){}
    
    struct ComputeDistanceTraversability 
    {
        ComputeDistanceTraversability (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *state1, 
                        double *near_) const 
        {
            NearNeighbor nn;

            nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2], state1[3], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

        double f_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };


    struct TraversabilityFunctor 
    {
        TraversabilityFunctor(double weight_factor, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, bool write_data, std::string user_name)
            : wf_(weight_factor), kdT_(kdT_From_NN), o_p_(obstacles_Points), w_d_(write_data), user_(user_name)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceTraversability,
                                                                        ceres::FORWARD, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceTraversability(kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const statePos,  T* residual) const 
        {
            T d_ugv_, n_[3], bound;
            (*compute_nearest_distance)(statePos, n_);

            d_ugv_ = (statePos[3]-n_[2])*(statePos[3]-n_[2]); // To the closest point to traversavility PC is only consider distance in Z axe
            bound = T{0.001};  // Bound maximum distance above UGV traversability map
            
            T max_value_residual = T{20.0};
            T min_value_residual = T{0.0};
            T value_dependent2 = T{0.05};
            T value_dependent1 = T{0.0};
            T m ;

            m = (max_value_residual- min_value_residual)/(value_dependent2 - value_dependent1);

            residual[0] =  wf_ *( m * (d_ugv_ - value_dependent1) + min_value_residual);

		    // std::cout << "TraversabilityFunctor["<<statePos[0] <<"] , residual[0]= "<< residual[0] << " , d_ugv_= " << d_ugv_ << " , bound= " << bound <<
            //              " , statePos[3]= " << statePos[3] << " , n_[2]= "<< n_[2] << std::endl;
            
	        if(w_d_){
                std::ofstream ofs;
                std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/traversability_ugv.txt";
                ofs.open(name_output_file.c_str(), std::ofstream::app);
                if (ofs.is_open()) 
                    ofs << residual[0] << "/" <<std::endl;
                ofs.close();
            }
            
            return true;
        }

        bool w_d_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        double wf_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
        std::string user_;
    };


private:

};

#endif