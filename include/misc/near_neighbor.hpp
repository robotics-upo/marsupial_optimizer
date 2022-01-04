#ifndef NEAR_NEIGHBOR_HPP
#define NEAR_NEIGHBOR_HPP

#include <iostream>
#include <fstream>
#include <vector>

// #include <g2o/core/factory.h>
#include <iomanip>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"

class NearNeighbor
{
	public:

		NearNeighbor();

	    virtual void setInput(const std::vector<Eigen::Vector3d> &obs_);
		virtual void setInput(const sensor_msgs::PointCloud2 &pc2);

		virtual void setKDTree(const std::vector<Eigen::Vector3d> &obs_);
		virtual void setKDTree(const sensor_msgs::PointCloud2 &pc2_);
	    virtual Eigen::Vector3d nearestObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_);
	    virtual Eigen::Vector3d nearestObstacleMarsupial(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_);
		virtual Eigen::Vector3d nearestTraversabilityUGVMarsupial(const pcl::KdTreeFLANN <pcl::PointXYZ> kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ , double radius_);
		virtual bool nearestObstacleStateCeres(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, double x_, double y_, double z_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_, double &ret_x_, double &ret_y_, double &ret_z_);
		virtual bool radiusNearestObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ , float _radius);
		virtual double distanceObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_);

		pcl::PointCloud<pcl::PointXYZ>::Ptr obs_points;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
		int K = 1 ;

	protected:

	private:


};

inline NearNeighbor::NearNeighbor(){}

inline void NearNeighbor::setInput(const std::vector<Eigen::Vector3d> &obs_){setKDTree(obs_);}
inline void NearNeighbor::setInput(const sensor_msgs::PointCloud2 &pc2) {setKDTree(pc2);}

inline void NearNeighbor::setKDTree(const std::vector<Eigen::Vector3d> &obs_)
{
	obs_points.reset(new pcl::PointCloud <pcl::PointXYZ>);
	
	// Convert from Eigen::Vector3d to pcl::PointXYZ
	// obs_points -> width = obs_.size() ;
	// obs_points -> height = 1 ;
	obs_points -> points.resize(obs_.size());
	for (size_t i = 0 ; i < obs_points->size() ; i ++)
	{
		obs_points->points[i].x = obs_[i].x();
		obs_points->points[i].y = obs_[i].y();
		obs_points->points[i].z = obs_[i].z();
	}
	kdtree.setInputCloud(obs_points);
}

inline void NearNeighbor::setKDTree(const sensor_msgs::PointCloud2 &pc2_)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Executed SetKDTree PointCloud2");
	obs_points.reset(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::fromROSMsg(pc2_,*obs_points);

	ROS_INFO(PRINTF_MAGENTA "KDTree : Size obs_points = [%lu]",obs_points->size());

	kdtree.setInputCloud(obs_points);
}

inline Eigen::Vector3d NearNeighbor::nearestObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ )
{
	Eigen::Vector3d ret_;
	
	pcl::PointXYZ searchPoints(vert_.x(),vert_.y(),vert_.z());

	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	// Get closest point
	kdT_.nearestKSearch(searchPoints, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	ret_.x() = o_p_->points[pointIdxNKNSearch[0]].x;
	ret_.y() = o_p_->points[pointIdxNKNSearch[0]].y;
	ret_.z() = o_p_->points[pointIdxNKNSearch[0]].z;

	return ret_;
}

inline Eigen::Vector3d NearNeighbor::nearestObstacleMarsupial(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ )
{
	Eigen::Vector3d ret_;
	
	pcl::PointXYZ searchPoints(vert_.x(),vert_.y(),vert_.z());

	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	// Get closest point
	kdT_.nearestKSearch(searchPoints, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	ret_.x() = o_p_->points[pointIdxNKNSearch[0]].x;
	ret_.y() = o_p_->points[pointIdxNKNSearch[0]].y;
	ret_.z() = o_p_->points[pointIdxNKNSearch[0]].z;

	return ret_;
}

inline Eigen::Vector3d NearNeighbor::nearestTraversabilityUGVMarsupial(const pcl::KdTreeFLANN <pcl::PointXYZ> kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ , double radius_)
{
	Eigen::Vector3d ret_;
	double z_aux_ = vert_.z();
	
	pcl::PointXYZ searchPoint_(vert_.x(),vert_.y(),vert_.z());

	// K nearest neighbor search
	std::vector<int> pointIdxRadiusSearch_;
	std::vector<float> pointRadiusSquaredDistance_;
	int count = 0;
	// Get closest point
	if ( kdT_.radiusSearch (searchPoint_, radius_, pointIdxRadiusSearch_, pointRadiusSquaredDistance_) > 0 )
  	{
  	  for (std::size_t i = 0; i < pointIdxRadiusSearch_.size (); ++i){
		if (o_p_->points[ pointIdxRadiusSearch_[i] ].x < vert_.x() + 0.1 && o_p_->points[ pointIdxRadiusSearch_[i] ].x > vert_.x() - 0.1 && 
		    o_p_->points[ pointIdxRadiusSearch_[i] ].y < vert_.y() + 0.1 && o_p_->points[ pointIdxRadiusSearch_[i] ].y > vert_.y() - 0.1 &&
			o_p_->points[ pointIdxRadiusSearch_[i] ].z > z_aux_){

			ret_.x() = o_p_->points[ pointIdxRadiusSearch_[i] ].x;
			ret_.y() = o_p_->points[ pointIdxRadiusSearch_[i] ].y;
			ret_.z() = o_p_->points[ pointIdxRadiusSearch_[i] ].z;
			z_aux_ = ret_.z();
			// ROS_ERROR("   nearestTraversabilityUGVMarsupial - radiusSearch: [%f %f %f]",ret_.x(),ret_.y(),ret_.z());
			count++;
			}
		}
  	}
	if (count == 0){
		ret_.x() = vert_.x();
		ret_.y() = vert_.y();
		ret_.z() = vert_.z();
	}
	return ret_;
}

inline bool NearNeighbor::nearestObstacleStateCeres(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, double x_, double y_, double z_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_, double &ret_x_, double &ret_y_, double &ret_z_)
{
	pcl::PointXYZ searchPoints(x_,y_,z_);

	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	// Get closest point
	kdT_.nearestKSearch(searchPoints, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	ret_x_ = o_p_->points[pointIdxNKNSearch[0]].x;
	ret_y_ = o_p_->points[pointIdxNKNSearch[0]].y;
	ret_z_ = o_p_->points[pointIdxNKNSearch[0]].z;

	return true;
}

inline bool NearNeighbor::radiusNearestObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ , float _radius)
{
	pcl::PointXYZ searchPoints(vert_.x(),vert_.y(),vert_.z());

	// K nearest neighbor search
	std::vector<int> pointIdxRadiusSearch;
 	std::vector<float> pointRadiusSquaredDistance;
	
	// Get closest in a radius point
	if (kdT_.radiusSearch(searchPoints, _radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
		// ret_.x() = o_p_->points[pointIdxRadiusSearch[0]].x;
		// ret_.y() = o_p_->points[pointIdxRadiusSearch[0]].y;
		// ret_.z() = o_p_->points[pointIdxRadiusSearch[0]].z;
		return true;
	}
	else
		return false;	
}

inline double NearNeighbor::distanceObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_)
{
	pcl::PointXYZ searchPoints(vert_.x(),vert_.y(),vert_.z());

	// K nearest neighbor search
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	// Get closest point

	kdT_.nearestKSearch(searchPoints, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	double dist_ = pointNKNSquaredDistance[0];

	return dist_;
}

#endif