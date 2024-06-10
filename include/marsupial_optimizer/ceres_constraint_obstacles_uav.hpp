#ifndef CERES_CONSTRAINS_OBSTACLES_UAV_HPP
#define CERES_CONSTRAINS_OBSTACLES_UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/grid3d.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include <iostream>
#include <fstream>
#include <string>

class ObstacleDistanceFunctorUAV {

public:
    ObstacleDistanceFunctorUAV(){}
    
    struct ComputeDistanceObstaclesUAV 
    {
        ComputeDistanceObstaclesUAV (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* g_3D_, bool use_dist_func_)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points), g_3D(g_3D_), use_dist_func(use_dist_func_)
        {}

        bool operator()(const double *state1, double *dist_) const 
        {
            NearNeighbor nn;

			double d_obs;
            if(use_dist_func){
                float x_ = state1[1];
				float y_ = state1[2];
				float z_ = state1[3];
// std::cout << "  UAV pos:"<< x_ <<"," << y_ << "," << z_ <<std::endl;  
				bool is_into_ = g_3D->isIntoMap(x_,y_,z_);
				if (is_into_){
					TrilinearParams d = g_3D->getPointDistInterpolation(x_, y_, z_);
					d_obs= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);
				}
				else
					d_obs = -1.0;
			}
			else{
				geometry_msgs::Vector3 near_;
            	nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2],state1[3], o_p_, near_.x, near_.y, near_.z);
				double arg_d_obs = pow(near_.x-state1[1],2) + pow(near_.y-state1[2],2) + pow(near_.z-state1[3],2);
                if (arg_d_obs < 0.0001 && arg_d_obs > -0.0001)
                    d_obs = 0.0;
                else
                    d_obs = sqrt(arg_d_obs);
			}

            // nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2], state1[3], o_p_, near_[0], near_[1], near_[2]);

            dist_[0] = d_obs; 
            
            return true;
        }

        double f_, sb_;
        bool use_dist_func;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
		Grid3d* g_3D;
    };

    struct ObstaclesFunctor 
    {
        ObstaclesFunctor(double weight_factor, double safty_bound, 
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, std::string path_, bool write_data, Grid3d* g_3D_, bool use_dist_func_, std::string user_name)
            : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points), p_(path_), w_d_(write_data), g_3D(g_3D_), use_dist_func(use_dist_func_), user_(user_name)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<1,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::FORWARD, 
                                                                        1,
                                                                        4>( 
                                    new ComputeDistanceObstaclesUAV(kdT_, o_p_, g_3D, use_dist_func))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T d_[1], arg_d, Diff_;
            // T d_;
            double f_slope_ = 5.0;
            
            (*compute_nearest_distance)(statePos1, d_);

            T d_sb_ = sb_ * T{1.05};
            T max_value_residual = T{100.0};
            T min_value_residual = T{0.0};
            T max_value_dependent = T{0.0};
            T m ;
            if (d_[0] > d_sb_)
                m = T{0.0};
            else
                m = (max_value_residual- min_value_residual)/(max_value_dependent - d_sb_);
            
            residual[0] = wf_ * m *(d_[0] - d_sb_);

            // std::cout << "ObstacleDistanceFunctorUAV: residual[0]= " << residual[0] << " , d_= " << d_[0] << std::endl;

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

        bool w_d_, use_dist_func;
        double wf_, sb_;
        std::string p_;
        std::string user_;
        std::unique_ptr<ceres::CostFunctionToFunctor<1,4> > compute_nearest_distance;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
		Grid3d* g_3D;
    };

private:

};

#endif