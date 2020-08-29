#ifndef _CATENARY_MISC_H_
#define _CATENARY_MISC_H_

#include <iostream>
#include <fstream>
#include <vector>

#include <iomanip>
#include <Eigen/StdVector>

#include "ros/ros.h"

using namespace std;

struct structCatenary
{
	int count_;
	int count_negative_z_;
	double d_average_collision_;
	double l_catenary_;
};

class CatenaryMisc
{
	public:
	CatenaryMisc();
	void SetValueCount(int pos, int value);
	void SetValueCountNegativeZ(int pos, int value);
	void SetValueLengthCatenary(int pos, double value);
	void SetValueDistanceAverage(int pos, double value);

	std::vector<structCatenary> *_v_states_catenary;

	protected:

	private:

};


#endif