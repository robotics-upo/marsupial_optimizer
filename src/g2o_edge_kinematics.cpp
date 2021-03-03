#include "marsupial_optimizer/g2o_edge_kinematics.h"

namespace g2o {

	G2OKinematicsEdge::G2OKinematicsEdge():
		BaseMultiEdge<3, double>()
	{
		this->resize(3); 

		_information.setIdentity();
		_error.setZero();
	}


	bool G2OKinematicsEdge::read(std::istream& is)
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

	bool G2OKinematicsEdge::write(std::ostream& os) const
	{
		double p = measurement();
		os << p;
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}


} // end namespace
