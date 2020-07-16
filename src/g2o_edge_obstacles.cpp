#include "marsupial_g2o/g2o_edge_obstacles.h"

namespace g2o {

	G2OObstaclesEdge::G2OObstaclesEdge() :
		BaseUnaryEdge<1, double, VertexPointXYZ>() 
	{

		_information.setIdentity();
		_error.setZero();
	}

	bool G2OObstaclesEdge::read(std::istream& is)
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

	bool G2OObstaclesEdge::write(std::ostream& os) const
	{
		double p = measurement();
		os << p;
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}

} // end namespace