#ifndef NEAR_NEIGHBOR_HPP
#define NEAR_NEIGHBOR_HPP

#include <iostream>
#include <fstream>
#include <vector>

#include <g2o/core/factory.h>
#include <iomanip>
#include <Eigen/StdVector>


#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>


class NearNeighbor
{
	public:

		NearNeighbor();

	    NearNeighbor(const std::vector<Eigen::Vector3d> &obs_);

		virtual void setKDTree(const std::vector<Eigen::Vector3d> &obs_);
	    virtual Eigen::Vector3d nearestObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_);
		virtual double distanceObstacleVertex(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, Eigen::Vector3d vert_);

		pcl::PointCloud <pcl::PointXYZ>::Ptr obs_points;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
		int K = 1 ;

	protected:

	private:



};

inline NearNeighbor::NearNeighbor(){}

inline NearNeighbor::NearNeighbor(const std::vector<Eigen::Vector3d> &obs_)
{
	// obs_points.reset(new pcl::PointCloud <pcl::PointXYZ>);
	setKDTree(obs_);
}

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