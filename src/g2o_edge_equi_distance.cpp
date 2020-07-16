#include "marsupial_g2o/g2o_edge_equi_distance.h"

namespace g2o {

	G2OEquiDistanceEdge::G2OEquiDistanceEdge():
		BaseMultiEdge<3, double>()
	{
		this->resize(3); 

		_information.setIdentity();
		_error.setZero();
	}


	bool G2OEquiDistanceEdge::read(std::istream& is)
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

	bool G2OEquiDistanceEdge::write(std::ostream& os) const
	{
		double p = measurement();
		os << p;
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}


} // end namespace
