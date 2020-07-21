#ifndef MARSUPIAL_CFG_HPP
#define MARSUPIAL_CFG_HPP

#include <iostream>
#include <fstream>
#include <vector>

// #include <g2o/core/factory.h>
#include <iomanip>
#include <Eigen/StdVector>

#include "marsupial_g2o/bisection3D.hpp"

// #include "marsupial_g2o/g2o_vertex_timediff.h"
// #include "marsupial_g2o/g2o_vertex_pointxyz.h"

#include "ros/ros.h"


using namespace std;

struct structPose
{
	int id;
	Eigen::Vector3d position;
    g2o::VertexPointXYZ vertex;
};

struct structTime
{
	int id_from;
	int id_to;
	double time;
	g2o::VertexTimeDiff vertex;
};

struct structDistance
{
	int id_from;
	int id_to;
	double dist;
};

class MarsupialCfg
{
	public:
		MarsupialCfg(ros::NodeHandle &nh, ros::NodeHandle &pnh);
		// ~MarsupialCfg();
		void loadDataCatenary(vector<structPose> &poses, vector<structDistance> &distances);	//Function to create a catenary from point A to B
		void loadData(double x1, double y1, double z1, double x2, double y2, double z2);	//Function to create a straight line from point A to B
		void loadData();
		void setNumVertUnit(int n_);
		void setDistance3D(float x1, float y1, float z1, float x2, float y2, float z2);
		void setNumPoints(float d_, int n_v);	// To define approximately the number of points to analyze per length unity
		void setVelocity(double v_);
		void getSlopXYZAxes( vector<float> &m_);
		bool getParameterOptimizer(std::string fileName);
		// VertexPointXYZ poseVertex(int index);
		// VertexTimeDiff timeDiffVertex(int index) ;
		//! Container of poses that represent the spatial part of the trajectory
		typedef vector<structPose> PoseSequence;
		//! Container of time differences that define the temporal of the trajectory
		typedef vector<structTime> TimeSequence;
		//! Container of distances between two consecutive vertices of the trajectory
		typedef vector<structDistance> DistanceSequence;
		PoseSequence pose_vec_; //!< Internal container storing the sequence of optimzable pose vertices
		DistanceSequence dist_vec_; //!< Internal container storing the distances between two consecutive vertices
		TimeSequence time_vec_;  //!< Internal container storing the sequence of optimzable timediff vertices
		int n_vert_unit;	//number of Vertex per unit of lenght
		float d3D_;
		float n_points; 
		double velocity;
		vector<float> slopeXYZ;
	protected:
	private:
};


MarsupialCfg::MarsupialCfg(ros::NodeHandle &nh, ros::NodeHandle &pnh){
}

// MarsupialCfg::~MarsupialCfg(){
// }

inline void MarsupialCfg::setNumVertUnit(int n_) {n_vert_unit = n_;}
inline void MarsupialCfg::setDistance3D(float x1, float y1, float z1, float x2, float y2, float z2) {d3D_= sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));}
inline void MarsupialCfg::setVelocity(double v_) {velocity = v_;}
inline void MarsupialCfg::setNumPoints(float d_, int n_v_u_) {n_points = ceil(d_*n_v_u_);}	
inline void MarsupialCfg::getSlopXYZAxes(vector<float> &m_) {m_=slopeXYZ;}	

void MarsupialCfg::loadDataCatenary(vector<structPose> &poses, vector<structDistance> &distances)
{
	bisection B;
	structPose ps;
	structDistance pd;
	for (size_t i = 0; i < B.catenary_chain_points_3D.size(); i++)
	{
		ps.id = i;
		ps.position = Eigen::Vector3d(B.catenary_chain_points_3D[i].x_,B.catenary_chain_points_3D[i].y_,B.catenary_chain_points_3D[i].z_);
		poses.push_back(ps);
	}
	for (size_t i = 0; i < B.catenary_chain_points_3D.size()-1; i++)
	{
		pd.id_from = i;
		pd.id_to = i+1;
		pd.dist = sqrt(pow(B.catenary_chain_points_3D[i].x_-B.catenary_chain_points_3D[i+1].x_,2)+
					pow(B.catenary_chain_points_3D[i].y_-B.catenary_chain_points_3D[i+1].y_,2)+
					pow(B.catenary_chain_points_3D[i].z_-B.catenary_chain_points_3D[i+1].z_,2));
		distances.push_back(pd);
	}
}

void MarsupialCfg::loadData(double x1, double y1, double z1, double x2, double y2, double z2)
{
	structPose ps;
	structDistance pd;
	structTime pt;
	double x_, y_, z_;
	// d3D_= sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));  
	ROS_INFO("Initial 3D distance of trayectory = %f",d3D_);
	// n_points = ceil(d3D_*n_vert_unit);   
	double distance_xy = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
	double distance_xz = sqrt(pow(x2-x1,2)+pow(z2-z1,2));
	double interval_xy = distance_xy/n_points;		// Size of the interval between points in axes xy
	double interval_xz = distance_xz/n_points;		// Size of the interval between points in axes xz
	slopeXYZ.push_back((x2-x1)/n_points);	//save the value of distance between two vertex in the X axes
	slopeXYZ.push_back((y2-y1)/n_points);	//save the value of distance between two vertex in the Y axes
	slopeXYZ.push_back((z2-z1)/n_points);	//save the value of distance between two vertex in the Z axes
	for(int i = 0 ; i< n_points+1 ; i++)
	{
		x_ = x1 + i*interval_xy * ((x2-x1)/distance_xy);
		y_ = y1 + i*interval_xy * ((y2-y1)/distance_xy);
		z_ = z1 + i*interval_xz * ((z2-z1)/distance_xz);
		// x_z = x1 + i*interval_xz * ((x2-x1)/distance_xz);
		ps.id = i;
		ps.position = Eigen::Vector3d(x_,y_,z_);
		ps.vertex.setEstimate(ps.position);
		pose_vec_.push_back(ps);
	}
	for(int i = 0 ; i< n_points ; i++)
	{
		pd.id_from = i;
		pd.id_to = i+1;
		pd.dist =  sqrt(pow(pose_vec_[i].position.x()-pose_vec_[i+1].position.x(),2)+
						pow(pose_vec_[i].position.y()-pose_vec_[i+1].position.y(),2)+
						pow(pose_vec_[i].position.z()-pose_vec_[i+1].position.z(),2));
		pt.id_from = i;
		pt.id_to = i+1;
		pt.time = pd.dist / velocity;
		dist_vec_.push_back(pd);
		time_vec_.push_back(pt);
	}
}

void MarsupialCfg::loadData()
{
	structDistance pd;
	for(unsigned i = 0 ; i< pose_vec_.size()-1 ; i++)
	{
		pd.id_from = i;
		pd.id_to = i+1;
		pd.dist =  sqrt(pow(pose_vec_[i].position.x()-pose_vec_[i+1].position.x(),2)+
						pow(pose_vec_[i].position.y()-pose_vec_[i+1].position.y(),2)+
						pow(pose_vec_[i].position.z()-pose_vec_[i+1].position.z(),2));
		dist_vec_.push_back(pd);
	}
}
// VertexPointXYZ MarsupialCfg::poseVertex(int index)
// {
// return pose_vec_.at(index).vertex;
// }
// VertexTimeDiff MarsupialCfg::timeDiffVertex(int index) 
// {
// return time_vec_.at(index).vertex;
// }

#endif