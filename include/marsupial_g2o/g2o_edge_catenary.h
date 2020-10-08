#pragma once

#ifndef G2O_CATENARY_EDGE_H
#define G2O_CATENARY_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include <vector>

// #include <pcl/search/kdtree.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include "marsupial_g2o/nearNeighbor.hpp"
#include "marsupial_g2o/bisection_catenary_3D.h"

#include "g2o_vertex_catenary_length.h"

#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"

#include <stdio.h>

namespace g2o {


class G2OCatenaryEdge : public BaseBinaryEdge<3, vector<double>, VertexPointXYZ, VertexCatenaryLength>
{
	public:
		G2OCatenaryEdge(ros::NodeHandlePtr nhP);
        
		struct points_obs_close_cat
        {
            Eigen::Vector3d obs;
            Eigen::Vector3d cat;
        };

		double _length_catenary;
		// double _porcentage = 1.1;

		std::vector<geometry_msgs::Point> _points_catenary;
		visualization_msgs::MarkerArray _catenary_marker;
		ros::Publisher catenary_marker_pub_;

		bisectionCatenary bc;

		void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v);	// Print the catenary between UGV and vertex 
		void clearMarkers(visualization_msgs::MarkerArray _marker,auto _s);

		int _id_marker;		// Related with Marker frame.id
		int _n_vertices;	// Number of Vertices that no are fixed
		int _prev_size_marker = 0; // Save the previus size value of the catenary for vertex optimized To delete marker 

		NearNeighbor _nn;
		Eigen::Vector3d _obstacles_near_catenary;
		Eigen::Vector3d _obstacles_nearest_catenary;
		Eigen::Vector3d obstacles_;
		Eigen::Vector3d prev_point_catenary;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
		double _radius; 
		double _radius_security; 

		double _d_curr, _x, _y, _z;
		bool _debug;
		double approximation_;
		double _length_cat_;
		double bound_bisection_a,bound_bisection_b;

		double bound_z_negative = 0.1;
		double lower_point_cat_z;

		geometry_msgs::Point _initial_pos_cat_ugv;

		int count_coll_cat_obs;
		std::vector<double> _v_count_close_cat_obs;
	

		std::vector<points_obs_close_cat> _v_obs_collision_cat;
		std::vector<points_obs_close_cat> _v_obs_close_cat;
		// std::vector<Eigen::Vector3d> _v_p_cat_collision;

		int n_points_cat_dis;
		double multiplicative_factor_error;
		double _d_cat_obs_prev;
		

		void computeError()
		{
    		const VertexPointXYZ * pose = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexCatenaryLength * l_catenary = static_cast<const VertexCatenaryLength*> (_vertices[1]);
	
			lower_point_cat_z = bound_z_negative;

			// Calculate Catenary
			_x = (pose->estimate().x()-_initial_pos_cat_ugv.x);
			_y = (pose->estimate().y()-_initial_pos_cat_ugv.y);
			_z = (pose->estimate().z()-_initial_pos_cat_ugv.z);
			_d_curr = sqrt( _x * _x + _y * _y + _z * _z); // Distance UGV and Vertex

			if (l_catenary->estimate() > _d_curr)
				_length_catenary = l_catenary->estimate();
			else
				_length_catenary = _d_curr*1.001;

			if (_length_catenary > 100.0 || pose->estimate().z() < bound_z_negative){
				ROS_ERROR ("OUT OF RANGE: Not posible to get Catenary for vertex[%i] = [%f %f %f] and length_catenary =[%f]", pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),_length_catenary);
				_error[0] = 100000000.0 * (pose->estimate().z() - bound_z_negative); 
				_error[1] = 100000000.0 * (pose->estimate().z() - bound_z_negative);
				_error[2] = 100000000.0 * (pose->estimate().z() - bound_z_negative);
			}
			else{
				bc.setNumberPointsCatenary(_length_catenary*10.0);
				bc.setFactorBisection(bound_bisection_a,bound_bisection_b);
				bc.configBisection(_length_catenary,_initial_pos_cat_ugv.x,_initial_pos_cat_ugv.y,_initial_pos_cat_ugv.z,pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),pose->id());

				_points_catenary.clear();
				// printf(PRINTF_RED);
				bc.getPointCatenary3D(_points_catenary);

				if (_points_catenary.size()<1.0){
					ROS_ERROR ("Not posible to get Catenary for vertex[%i] = [%f %f %f]", pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
				}
				
				_id_marker = pose->id();
				// Print Catenary Marker between each vertex and UGV
				if (_prev_size_marker >= _points_catenary.size() )
					clearMarkers(_catenary_marker,_prev_size_marker);
				else
					clearMarkers(_catenary_marker,_points_catenary.size());
				
				markerPoints(_catenary_marker ,_points_catenary,_id_marker,_n_vertices);
				_prev_size_marker = _points_catenary.size();

				// To Obtain KdTree for each point in every calculated catenary 
				int count = 0 ; 
				int count_negative_z = 0;
				bool first_point_cat = true;
				bool inside_x, inside_y, inside_z;
				_v_obs_collision_cat.clear();
				// _v_p_cat_collision.clear();
				_v_obs_close_cat.clear();

				points_obs_close_cat _points1, _points2;

				count_coll_cat_obs = 0;	
				_v_count_close_cat_obs.clear();
				_d_cat_obs_prev = 10000.0;
				// printf("CATENARY SIZE =[%lu] VERTEX=[%i]!!!!\n",_points_catenary.size(),pose->id());
				for (size_t i = 0 ; i < _points_catenary.size() ; i++){
					Eigen::Vector3d p_cat;
					p_cat.x() = _points_catenary[i].x;
					p_cat.y() = _points_catenary[i].y;
					p_cat.z() = _points_catenary[i].z;
					
					// printf("verterx[%i][%f %f %f] p_cat=[%lu][%f %f %f]\n",pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),i,p_cat.x(),p_cat.y(),p_cat.z());
					
					_obstacles_near_catenary = _nn.nearestObstacleVertex(kdT_From_NN , p_cat,obstacles_Points);
					double _d_cat_obs = (p_cat-_obstacles_near_catenary).norm();

					if (_d_cat_obs < _d_cat_obs_prev){
						_obstacles_nearest_catenary.x() = _obstacles_near_catenary.x();
						_obstacles_nearest_catenary.y() = _obstacles_near_catenary.y();
						_obstacles_nearest_catenary.z() = _obstacles_near_catenary.z();
						_d_cat_obs_prev = _d_cat_obs;
					}
					
					// Get Data I Analysis Catenary: To check if there is obstacle between two catenary points
					// int n_points_cat_dis = 6; // parameter to ignore collsion points in the begining and in the end of catenary
					n_points_cat_dis = ceil(1.5*ceil(_length_catenary)); // parameter to ignore collsion points in the begining and in the end of catenary
					if (n_points_cat_dis < 5)
						n_points_cat_dis = 5;
					// if(pose->id() == 10 ||pose->id() == 11)
					// 	printf("Dentro FOR[%lu]: _d_cat_obs=[%f] p_cat=[%f %f %f] _obstacles_near_catenary=[%f %f %f] _points_catenary.size=[%lu] n_points_cat_dis=[%i] ingnored_after_pos_in_the_end=[%lu]\n",
					// 	i,_d_cat_obs,p_cat.x(),p_cat.y(),p_cat.z(),_obstacles_near_catenary.x(),_obstacles_near_catenary.y(),_obstacles_near_catenary.z(),
					// 	_points_catenary.size(),n_points_cat_dis,(_points_catenary.size()- n_points_cat_dis));
					// }
					// if (_d_cat_obs < _radius && (i > n_points_cat_dis && i < (_points_catenary.size()- n_points_cat_dis))){
					if (_d_cat_obs < _radius && (i > n_points_cat_dis )){
						_points1.obs = _obstacles_near_catenary;
						_points1.cat = p_cat;
						// printf("Dentro IF vertex[%i] pointCat[%lu]: _d_cat_obs=[%f] p_cat=[%f %f %f] _obstacles_near_catenary=[%f %f %f]\n",pose->id(),i,_d_cat_obs,p_cat.x(),p_cat.y(),p_cat.z(),_obstacles_near_catenary.x(),_obstacles_near_catenary.y(),_obstacles_near_catenary.z());
						_v_obs_close_cat.push_back(_points1);
						count_coll_cat_obs++;
						_v_count_close_cat_obs.push_back(i);
					}
					
					// double _d_points_cat = (p_cat-prev_point_catenary).norm();
					// approximation_ = _d_points_cat;
					
					// Get Data II Analysis Catenary: To check if there is obstacle between two catenary points
					approximation_ = _radius;
					inside_x = inside_y = inside_z = false;
					if (!first_point_cat){
						if (prev_point_catenary.x() < p_cat.x()){
							if ((prev_point_catenary.x() - approximation_) < _obstacles_near_catenary.x() && (p_cat.x() + approximation_) > _obstacles_near_catenary.x()){
								inside_x = true;
								obstacles_.x()=_obstacles_near_catenary.x();					
							}
						}
						else if (prev_point_catenary.x() > p_cat.x()){
							if ((prev_point_catenary.x() + approximation_) > _obstacles_near_catenary.x() && (p_cat.x() - approximation_) < _obstacles_near_catenary.x()){
								inside_x = true;
								obstacles_.x()=_obstacles_near_catenary.x();					
							}
						}
						else if (prev_point_catenary.x() == p_cat.x()){
							if ((prev_point_catenary.x() + approximation_) > _obstacles_near_catenary.x() && (prev_point_catenary.x() - approximation_) < _obstacles_near_catenary.x()){
								inside_x = true;
								obstacles_.x()=_obstacles_near_catenary.x();					
							}
						}
						else
							inside_x = false;

						if (prev_point_catenary.y() < p_cat.y()){
							if ((prev_point_catenary.y() - approximation_) < _obstacles_near_catenary.y() && (p_cat.y() + approximation_) > _obstacles_near_catenary.y()){
								inside_y = true;
								obstacles_.y()=_obstacles_near_catenary.y();					
							}
						}
						else if (prev_point_catenary.y() > p_cat.y()){
							if ((prev_point_catenary.y() + approximation_) > _obstacles_near_catenary.y() &&  (p_cat.y() - approximation_) < _obstacles_near_catenary.y()){
								inside_y = true;
								obstacles_.y()=_obstacles_near_catenary.y();					
							}
						}
						else if (prev_point_catenary.y() == p_cat.y()){
							if ((prev_point_catenary.y() + approximation_) > _obstacles_near_catenary.y() && (prev_point_catenary.y() - approximation_) < _obstacles_near_catenary.y()){
								inside_y = true;
								obstacles_.y()=_obstacles_near_catenary.y();					
							}
						}
						else
							inside_y = false;

						if (prev_point_catenary.z() < p_cat.z()){
							if ((prev_point_catenary.z() - approximation_) < _obstacles_near_catenary.z() && (p_cat.z() + approximation_) > _obstacles_near_catenary.z()){
								inside_z = true;
								obstacles_.z()=_obstacles_near_catenary.z();					
							}
						}
						else if (prev_point_catenary.z() > p_cat.z()){
							if ((prev_point_catenary.z() + approximation_) > _obstacles_near_catenary.z() && (p_cat.z() - approximation_) < _obstacles_near_catenary.z()){
								inside_z = true;
								obstacles_.z()=_obstacles_near_catenary.z();					
							}
						}
						else if (prev_point_catenary.z() == p_cat.z()){
							if ((prev_point_catenary.z() + approximation_) > _obstacles_near_catenary.z() && (prev_point_catenary.z() - approximation_) < _obstacles_near_catenary.z()){
								inside_z = true;
								obstacles_.z()=_obstacles_near_catenary.z();					
							}
						}
						else
							inside_z = false;

						if (inside_x && inside_y && inside_z){
							count++;
							_points2.cat =  p_cat;
							_points2.obs = _obstacles_near_catenary;
							_v_obs_collision_cat.push_back(_points2);
						}
					}
					prev_point_catenary.x() = _points_catenary[i].x;
					prev_point_catenary.y() = _points_catenary[i].y;
					prev_point_catenary.z() = _points_catenary[i].z;
					first_point_cat = false;

					// Analysis II: To check if there are catenary points above z= 0.0
					if (p_cat.z() < bound_z_negative){
						count_negative_z++;
						if ( p_cat.z() < lower_point_cat_z)
							lower_point_cat_z = p_cat.z();
					}
				}
		
				// Computed Error[0]: Collision catenary with obstacle
				// Computed Error[1]: To straight the Cable

				

				// if (_v_obs_close_cat.size() > 0){
				// if (_v_obs_collision_cat.size() > 0){
					//Get average of points from catenaty and obstacles close each other. 
					points_obs_close_cat _v_av_close_oc;
					computeAveragePoint(_v_obs_close_cat,_v_av_close_oc);

					multiplicative_factor_error = getMultiplicativeFactorError(_d_curr);
					double dis_z_cat_obs = _v_av_close_oc.cat.z() - _v_av_close_oc.obs.z();
					double dist_close_cat_obs = (_v_av_close_oc.cat  - _v_av_close_oc.obs).norm();

					double _sigm = 1.0 /(1.0 + exp(-dis_z_cat_obs));

					// double dis_z_cat_init = 2.0*(_v_av_close_oc.cat.z() - _initial_pos_cat_ugv.z);
					
					// double factor_exp;
					// if (dis_z_cat_obs > 0.3)
					// 	factor_exp = 0.4;
					// else
					// 	factor_exp = 1.0;

					// _error[0] = 100000000.0/pow(_length_catenary,6) * exp(_d_curr) * exp (2.0*(_v_av_close_oc.cat.z()+0.1 - _v_av_close_oc.obs.z() ));
					// _error[0] = multiplicative_factor_error * pow (2.0,_d_curr*1.5) /pow(_length_catenary,5) * exp (factor_exp*dis_z_cat_obs);
					// _error[0] = multiplicative_factor_error * pow (2.0,_d_curr*1.5) /pow(_length_catenary,5) * (double)count_coll_cat_obs;
					// _error[0] = multiplicative_factor_error * pow (2.0,_d_curr*1.5) /pow(_length_catenary,5) * 15*dist_close_cat_obs;
					// _error[0] = multiplicative_factor_error * pow (2.0,_d_curr*1.5) /pow(_length_catenary,5) * 15*dis_z_cat_obs;
					// _error[0] = multiplicative_factor_error * pow (2.0,_d_curr*1.5) /pow(_length_catenary,5) * dis_z_cat_init ;
					// _error[0] = multiplicative_factor_error /pow(_length_catenary,4)*exp(count_coll_cat_obs/_points_catenary.size());
					double factor_exp;
					if (dis_z_cat_obs > 0.0)
						factor_exp = 2.0;
					else
						factor_exp = 1.0;
					double diff_dis = _length_catenary-_d_curr;
					// double _exponent = 10.0 + _length_catenary/100;
					double _exponent = 10.0 + diff_dis*10.0;
					// _error[0] =  1000.0 *(pow(_d_curr,10) / pow(_length_catenary,_exponent)) *exp(2.0*count_coll_cat_obs/_points_catenary.size()) * exp(factor_exp*_sigm);
					// _error[0] = 5000.0 + 1000.0 *(pow(_d_curr,10) / pow(_length_catenary,_exponent)) *exp(2.0*count_coll_cat_obs/_points_catenary.size()) * exp(factor_exp*_sigm);
					_error[0] =  10000.0 * exp(_radius -_d_cat_obs_prev);

					//Get distance between average points in collision from obstacles and catenary. 
					// points_obs_close_cat _v_av_coll_oc;	
					// computeAveragePoint(_v_obs_collision_cat, _v_av_coll_oc);
					// _error[0] = 10000000.0/pow(_length_catenary,6) * exp ( _v_av_coll_oc.obs.z() - 1.0 - _v_av_coll_oc.cat.z());

					_error[1] = 0.0;
					// printf("vertex[%i] length=[%f] d_curr=[%f] count_coll_cat_obs=[%i/%lu] dis_z_cat_obs=[%f] v_av_close_oc.cat.z=[%f] >? v_av_close_oc.obs.z=[%f] n_points_cat_dis=[%i] error[0]=[%f]\n",
					// pose->id(),_length_catenary,_d_curr,count_coll_cat_obs,_points_catenary.size(), dis_z_cat_obs, _v_av_close_oc.cat.z(),_v_av_close_oc.obs.z(),n_points_cat_dis,_error[0]);
					// for(size_t j = 0; j< _v_count_close_cat_obs.size() ; j++){
					// 	printf("_v_pos_cat_coll_obs=[%f]\n",_v_count_close_cat_obs[j]);
					// }
				// }
				// else{
				// 	_error[0] = 0.0;
				// 	_error[1] = 100.0*(_length_catenary -  _d_curr); 
				// 	// printf("Streaching tether error[1]=[%f] vertex=[%i]\n",_error[1],pose->id());
				// } 
				// Computed Error[2]: To Avoid -z valus for catenary
				_error[2] = 100000000.0*(double)(count_negative_z)*_length_catenary;
			}
		}

		virtual bool read(std::istream& is);

		virtual bool write(std::ostream& os) const;

		inline void readKDTree(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_){
			kdT_From_NN = kdT_;
			obstacles_Points = o_p_;
		}

		//Take the position value of the UGV
		virtual void setMeasurement(const vector<double> _p) {_measurement = _p;}
		
		virtual void numberVerticesNotFixed(const int& _n_v) {_n_vertices = _n_v;}

		inline void setRadius(const double& _s_r){_radius = _s_r;}

		inline void setInitialPosCatUGV(const geometry_msgs::Point& _p) {_initial_pos_cat_ugv = _p;}

		// inline void setLengthCatenary(double _v){_length_catenary = _v;} // Set the catenary length related with distance between UGV and vertex

		inline void setBoundForBisection(double _fa,double _fb){
			bound_bisection_a = _fa; 
			bound_bisection_b = _fb;
		}

		virtual void computeAveragePoint(std::vector<points_obs_close_cat> _vp, points_obs_close_cat &_struc){
			double xc,yc,zc,xo,yo,zo;
			xc=yc=zc=xo=yo= 0.0;
			zc = 0.0;
			zo = 100.0;
			for(size_t i=0 ; i < _vp.size() ; i++){
				xc = (_vp[i].cat.x() + xc);
				yc = (_vp[i].cat.y() + yc);
				// zc = (_vp[i].cat.z() + zc);
				xo = (_vp[i].obs.x() + xo);
				yo = (_vp[i].obs.y() + yo);
				// zo = (_vp[i].obs.z() + zo);
				if (_vp[i].cat.z() > zc)
					zc = _vp[i].cat.z();
				if (_vp[i].obs.z() < zo)
					zo = _vp[i].obs.z();
				// printf("computeAveragePoint print _vp[%lu/%lu].obs.z() =[%f] zo=[%f]\n",i,_vp.size(),_vp[i].obs.z(),zo);
			}	
			double _s = _vp.size();
			// double _s;
			// if (_vp.size() > 0.0)
			// 	_s = _vp.size();
			// else
			// 	_s = 1.0;
			_struc.cat = Eigen::Vector3d(xc/_s,yc/_s,zc);
			_struc.obs = Eigen::Vector3d(xo/_s,yo/_s,zo);
		}

		virtual double getMultiplicativeFactorError(double _d){
			double ret;
			if (_d >= 0.01 && _d < 1.0){ret = 100*_d;}
			else if (_d >= 1.0 && _d < 1.5){ret = 50000*_d-45000;}
			else if (_d >= 1.5 && _d < 2.0){ret = 140000*_d-180000;}
			else if (_d >= 2.0 && _d < 2.5){ret = 200000*_d-300000;}
			else if (_d >= 2.5 && _d < 4.0){ret = 400000*_d-800000;}
			else if (_d >= 4.0 && _d < 5.0){ret = 200000*_d;}
			else if (_d >= 4.0 && _d < 5.0){ret = 200000*_d;}
			else if (_d >= 5.0){ret = 1000000;}

			return (ret);
		}

	protected: 
};
}

#endif