#include "marsupial_optimizer/g2o_edge_time.h"

namespace g2o {

	G2OTimeEdge::G2OTimeEdge() :
		BaseUnaryEdge<1, double, VertexTimeDiff>()
	{
		_information.setIdentity();
		_error.setZero();
	}

	bool G2OTimeEdge::read(std::istream& is)
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

	bool G2OTimeEdge::write(std::ostream& os) const
	{
		double p = measurement();
		os << p;
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}


} // end namespace
