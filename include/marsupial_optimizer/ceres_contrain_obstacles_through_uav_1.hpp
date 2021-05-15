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
    
    struct ComputeDistanceObstaclesThroughUAV 
    {
        ComputeDistanceObstaclesThroughUAV (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *state1, double *near_) const 
        {
            NearNeighbor nn;

            nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2], state1[3], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

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
            
            std::cout << 
            bool r_cast_coll = o_t_->castRay(s_, d_, r_);
            end_[0]= r_.x();
            end_[1]= r_.y();
            end_[2]= r_.z();

            return true;
        }

        octomap::OcTree *o_t_;
    };


    struct ObstaclesThroughFunctor 
    {
        ObstaclesThroughFunctor(double weight_factor, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, octomap::OcTree* octotree_)
            : wf_(weight_factor), kdT_(kdT_From_NN), o_p_(obstacles_Points), oc_tree_(octotree_)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesThroughUAV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstaclesThroughUAV(kdT_, o_p_))));
            
            ray_casting.reset(new ceres::CostFunctionToFunctor<4,4,4>(
                                    new ceres::NumericDiffCostFunction<RayCastingThroughUAV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4,
                                                                        4>( 
                                    new RayCastingThroughUAV(oc_tree_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
        {
            T near_[4];
            T near_min[4];
            T statePos[4];
            T statePos_[4];
            T _sum_obstacle_x, _sum_obstacle_y, _sum_obstacle_z; 
            T _main_obstacle[3];
            T count_obs = T{0};
            bool inside_x, inside_y, inside_z;
            int _size_vector;
            double factor_ = 10.0;
            double approximation_ = 0.2;

            int num_interv = 5;
            T _xyz[3*(num_interv+1)];
            T num_interv_T = T{5};

			T n_ = T{0.0};

            for(int i = 0 ; i< num_interv+1 ; i++)
			{
                _xyz[0+i*3] = statePos1[1] + n_ * ((statePos2[1]-statePos1[1])/num_interv_T);
				_xyz[1+i*3] = statePos1[2] + n_ * ((statePos2[2]-statePos1[2])/num_interv_T);
				_xyz[2+i*3] = statePos1[3] + n_ * ((statePos2[3]-statePos1[3])/num_interv_T);
				n_ = n_ + T{1.0};
            }

			_sum_obstacle_x = _sum_obstacle_y = _sum_obstacle_z = T{0.0};
             T _do_min;
            _do_min = T{1000.0};
			
            	for (size_t i =0; i <num_interv+1;i++){
					inside_x = false;
					inside_y = false;
					inside_z = false;
					T obstacles_[3];

                    statePos[0] = T{0};                    
                    statePos[1] =_xyz[0+i*3];                    
                    statePos[2] =_xyz[1+i*3];                    
                    statePos[3] =_xyz[2+i*3]; 

                    statePos_[0] = T{0};       
                    statePos_[1] =_xyz[0+i*3]; 
                    statePos_[2] =_xyz[1+i*3]; 
                    statePos_[3] =_xyz[2+i*3];                    

                    (*compute_nearest_distance)(statePos, near_);
                    (*ray_casting)(statePos,statePos_, near_);

					if (statePos1[1] < statePos2[1]){
						if ((statePos1[1] - approximation_) < near_[0] && (statePos2[1] + approximation_) > near_[0]){
							inside_x = true;
							obstacles_[0] = near_[0];					
						}
					}
					else if (statePos1[1] > statePos2[1]){
						if ((statePos1[1] + approximation_) > near_[0] && (statePos2[1] - approximation_) < near_[0]){
							inside_x = true;
							obstacles_[0] = near_[0];					
						}
					}
					else if (statePos1[1] == statePos2[1]){
						if ((statePos1[1] + approximation_) > near_[0] && (statePos1[1] - approximation_) < near_[0]){
							inside_x = true;
							obstacles_[0] = near_[0];					
						}
					}
					else
						inside_x = false;


					if (statePos1[2] < statePos2[2]){
						if ((statePos1[2] - approximation_) < near_[1] && (statePos2[2] + approximation_) > near_[1]){
							inside_y = true;
							obstacles_[1] = near_[1];					
						}
					}
					else if (statePos1[2] > statePos2[2]){
						if ((statePos1[2] + approximation_) > near_[1] &&  (statePos2[2] - approximation_) < near_[1]){
							inside_y = true;
							obstacles_[1] = near_[1];					
						}
					}
					else if (statePos1[2] == statePos2[2]){
						if ((statePos1[2] + approximation_) > near_[1] && (statePos1[2] - approximation_) < near_[1]){
							inside_y = true;
							obstacles_[1] = near_[1];					
						}
					}
					else
						inside_y = false;

					
					if (statePos1[3] < statePos2[3]){
						if ((statePos1[3] - approximation_) < near_[2] && (statePos2[3] + approximation_) > near_[2]){
							inside_z = true;
							obstacles_[2] = near_[2];					
						}
					}
					else if (statePos1[3] > statePos2[3]){
						if ((statePos1[3] + approximation_) > near_[2] && (statePos2[3] - approximation_) < near_[2]){
							inside_z = true;
							obstacles_[2] = near_[2];					
						}
					}
					else if (statePos1[3] == statePos2[3]){
						if ((statePos1[3] + approximation_) > near_[2] && (statePos1[3] - approximation_) < near_[2]){
							inside_z = true;
							obstacles_[2] = near_[2];					
						}
					}
					else
						inside_z = false;

					// if (inside_x && inside_y && inside_z){
                    //     _sum_obstacle_x = obstacles_[0] + _sum_obstacle_x;
				    //     _sum_obstacle_y = obstacles_[1] + _sum_obstacle_y;
				    //     _sum_obstacle_z = obstacles_[2] + _sum_obstacle_z;
                    //     count_obs = count_obs + T{1};
					// }
                    
                    T _do;
                    _do = T{0.0};
                    if (inside_x && inside_y && inside_z){
                        _do = sqrt( (statePos[1]-near_[0])*(statePos[1]-near_[0]) + (statePos[2]-near_[1])*(statePos[2]-near_[1]) + (statePos[3]-near_[2])*(statePos[3]-near_[2]));
                        if (_do < _do_min){
                            near_min[0]=near_[0];
                            near_min[1]=near_[1];
                            near_min[2]=near_[2];
                            _do_min = _do;
                        }
                    }
				}

            if (count_obs > T{0}){
    		    _main_obstacle[0] = (_sum_obstacle_x) / count_obs;
    		    _main_obstacle[1] = (_sum_obstacle_y) / count_obs;
    		    _main_obstacle[2] = (_sum_obstacle_z) / count_obs;
            }

            T _do1, _do2;
            _do1 = _do2 = T{0.0};
         
			if (inside_x && inside_y && inside_z)
			{
                // _do1 = sqrt((statePos1[1]-_main_obstacle[0])*(statePos1[1]-_main_obstacle[0]) + 
                //             (statePos1[2]-_main_obstacle[1])*(statePos1[2]-_main_obstacle[1]) + 
                //             (statePos1[3]-_main_obstacle[2])*(statePos1[3]-_main_obstacle[2]));
			    // _do2 = sqrt((statePos2[1]-_main_obstacle[0])*(statePos2[1]-_main_obstacle[0]) + 
                //             (statePos2[2]-_main_obstacle[1])*(statePos2[2]-_main_obstacle[1]) + 
                //             (statePos2[3]-_main_obstacle[2])*(statePos2[3]-_main_obstacle[2]));

                _do1 = sqrt((statePos1[1]-near_min[0])*(statePos1[1]-near_min[0]) + 
                            (statePos1[2]-near_min[1])*(statePos1[2]-near_min[1]) + 
                            (statePos1[3]-near_min[2])*(statePos1[3]-near_min[2]));
			    _do2 = sqrt((statePos2[1]-near_min[0])*(statePos2[1]-near_min[0]) + 
                            (statePos2[2]-near_min[1])*(statePos2[2]-near_min[1]) + 
                            (statePos2[3]-near_min[2])*(statePos2[3]-near_min[2]));

                // std::cout << "Obstacle Between P1 =["<<statePos1[1]<<","<<statePos1[2]<<","<<statePos1[3]<<"] , P2=["<<statePos2[1]<<","<<statePos2[2]<<","<<statePos2[3]<<"]"<<std::endl;
                std::cout << "Obstacle Between P1 =["<<statePos1[0]<<"] , P2=["<<statePos2[0]<<"]"<<std::endl;
            
            }
				
            residual[0] = factor_ * (exp(_do1+_do2) - 1.0); 
            // residual[0] = factor_ * (exp(-1.0*_do) - 1.0); 

            return true;
        }

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4,4> > ray_casting;
        double wf_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
        octomap::OcTree *oc_tree_;
        
    };


private:

};

#endif