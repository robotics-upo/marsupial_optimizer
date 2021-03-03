#include "marsupial_optimizer/g2o_edge_distance_xyz.h"

namespace g2o {

	G2ODistanceXYZEdge::G2ODistanceXYZEdge() :
		BaseBinaryEdge<1, std::vector<float>, VertexPointXYZ, VertexPointXYZ>()
	{
		_information.setIdentity();
		_error.setZero();
	}

	bool G2ODistanceXYZEdge::read(std::istream& is)
	{
		std::vector<float> p;
		is >> p[0] >> p[1] >> p[2];
		setMeasurement(p);
		for (int i = 0; i < 3; ++i)
		for (int j = i; j < 3; ++j) {
			is >> information()(i, j);
			if (i != j)
			information()(j, i) = information()(i, j);
		}
		return true;
	}

	bool G2ODistanceXYZEdge::write(std::ostream& os) const
	{
		std::vector<float> p = measurement();
		os << p[0] << " " << p[1] << " " << p[2];
		for (int i = 0; i < 3; ++i)
		for (int j = i; j < 3; ++j)
			os << " " << information()(i, j);
		return os.good();
	}


} // end namespace
