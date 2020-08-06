#include "marsupial_g2o/g2o_edge_obstacles_through.h"

namespace g2o {

	G2OThroughObstaclesEdge::G2OThroughObstaclesEdge() :
		BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>() 
	{
		
		_information.setIdentity();
		_error.setZero();
	}

	bool G2OThroughObstaclesEdge::read(std::istream& is)
	{
		double p;
		is >> p;
		setMeasurement(p);
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
		}
		return true;
	}

	bool G2OThroughObstaclesEdge::write(std::ostream& os) const
	{
		double p = measurement();
		os << p;
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}

} // end namespace