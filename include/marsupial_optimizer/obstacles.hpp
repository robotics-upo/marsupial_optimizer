#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <iostream>
#include <fstream>
#include <vector>

#include <g2o/core/factory.h>
#include <iomanip>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;


class Obstacles
{

public:
    Obstacles();

    void simulateObstacles(float x, float y, float z, float dx, float dy, float dz, vector<Eigen::Vector3d> &v_, bool beggining);
	void setObstacles(vector<Eigen::Vector3d> &vector_in);

	ofstream file_obs;
    string path= "/home/simon/";

protected:

};

Obstacles::Obstacles(){}

void Obstacles::simulateObstacles(float x, float y, float z, float dx, float dy, float dz,vector<Eigen::Vector3d> &v_, bool beggining)
{
	
	Eigen::Vector3d pose;
	float count = 0.0;
	float m_ = 2;
	for (float i = 0; i < dx*m_; i++)
	{
		for (float j = 0; j < dy*m_; j++)
		{
			for (float k = 0; k < dz*m_; k++)
			{
				pose.x()= x + i/m_;
				pose.y()= y + j/m_;
				pose.z()= z + k/m_;
				v_.push_back(pose);
				count ++;
			}
		}
	}
	if (beggining){
		file_obs.open (path+"obstacles.txt");
	}
	else
	{
		file_obs.open (path+"obstacles.txt",ios::app);
	}
	int b_ = v_.size() - count;
	for (unsigned i = b_; i < v_.size(); i++)
	{
		file_obs << setprecision(4) << v_[i].x() << ";" << v_[i].y() << ";" << v_[i].z()<< endl;
	}
	file_obs.close();

}



#endif