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
#include "misc/grid3d.hpp"
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
        ComputeDistanceObstaclesUAV (float step_, float step_inv_, Grid3d* _grid_3D, 
                                     pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : s_(step_), s_i_(step_inv_), _g_3D(_grid_3D), kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
        }
        bool operator()(const double *state1, double *dist_) const 
        {
            printf(" ======= obstacleDistanceFunctorUAV  state=[%f]\n",state1[0]);
            printf("ObstacleDistanceFunctorUAV : state=[%f %f %f]\n",state1[1], state1[2], state1[3]);
            int x_, y_, z_;
            x_ = round(state1[1]*s_i_); y_= round(state1[2]*s_i_); z_ = round(state1[3]*s_i_);
            printf("ObstacleDistanceFunctorUAV : int=[%i %i %i]\n",x_, y_, z_);
            double X_, Y_, Z_;
            X_ = x_*s_; Y_ = y_*s_; Z_ = z_*s_;
            printf("ObstacleDistanceFunctorUAV : double=[%f %f %f]\n",X_, Y_, Z_);

            bool is_into_ = _g_3D->isIntoMap(X_, Y_, Z_);
			double d_[1];
            if (is_into_)
				d_[0] = _g_3D->getPointDist(X_, Y_, Z_);
			else
				d_[0] = -1.0;

            dist_[0] = d_[0];

            NearNeighbor nn;
            double near1_[3];
            nn.nearestObstacleStateCeres(kdT_,X_, Y_, Z_, o_p_, near1_[0], near1_[1], near1_[2]);
            double dist = sqrt(pow(X_-near1_[0],2)+pow(Y_-near1_[1],2)+pow(Z_-near1_[2],2));

            printf("ObstacleDistanceFunctorUAV : grid3d d=[%f]   kdtree d=[%f]\n",dist_[0], dist);

            return true;
        }

        double f_, sb_;
        float s_, s_i_;
        Grid3d* _g_3D;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

    struct ObstaclesFunctor 
    {
        ObstaclesFunctor(double weight_factor, double safty_bound, float step_, float step_inv_, Grid3d* grid_3D_, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, 
                         pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
                         : wf_(weight_factor), sb_(safty_bound), s_(step_), s_i_(step_inv_), g_3D_(grid_3D_) , kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,1>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::FORWARD, 
                                                                        4,
                                                                        1>( 
                                    new ComputeDistanceObstaclesUAV(s_, s_i_, g_3D_, kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T d_uav_1;
            // T arg_d_uav_1;
            T n1_[1];

            std::cout << "\t\tObstaclesFunctor: statePos1["<<statePos1[0]<<"]=["<<statePos1[1]<<" "<<statePos1[2]<<" "<<statePos1[3]<<"]"<<std::endl;
            (*compute_nearest_distance)(statePos1, n1_);
            // arg_d_uav_1 = (statePos1[1]-n1_[0])*(statePos1[1]-n1_[0]) + (statePos1[2]-n1_[1])*(statePos1[2]-n1_[1]) + (statePos1[3]-n1_[2])*(statePos1[3]-n1_[2]);
            
            // arg_d_uav_1 = n1_[0];

            // if (arg_d_uav_1 < 0.0001 && arg_d_uav_1 > -0.0001)
            //     d_uav_1 = T{0.0};
            // else
            //     d_uav_1 = sqrt(arg_d_uav_1);

            d_uav_1 = n1_[0];

            T Diff_ = (sb_ - d_uav_1);
            residual[0] = wf_ * 100.0 * log(1.0 + exp(4.0*Diff_) ) ;


            return true;
        }

        double wf_, sb_;
        float s_, s_i_;
        Grid3d* g_3D_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,1> > compute_nearest_distance;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };


private:

};

#endif