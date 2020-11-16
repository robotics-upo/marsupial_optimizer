#ifndef CERES_CONTRAIN_KINEMATICS_HPP
#define CERES_CONTRAIN_KINEMATICS_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class KinematicsFunctor {

public:
  KinematicsFunctor(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

  template <typename T>
  bool operator()(const T* const pose1, const T* const pose2, const T* const pose3, T* residual) const {

      Eigen::Matrix<T, 3, 1> vector1(pose2[0]-pose1[0],pose2[1]-pose1[1],pose2[2]-pose1[2]);
      Eigen::Matrix<T, 3, 1> vector2(pose2[0]-pose3[0],pose2[1]-pose3[1],pose2[2]-pose3[2]);

			T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]) + (vector2[2] * vector1[2]);
			T norm_vector1 = sqrt((vector1[0] * vector1[0]) + (vector1[1] * vector1[1]) + (vector1[2] * vector1[2]));
			T norm_vector2 = sqrt((vector2[0] * vector2[0]) + (vector2[1] * vector2[1]) + (vector2[2] * vector2[2]));

			T angle = acos(dot_product / (norm_vector1*norm_vector2));

			T bound1 = T(M_PI) - T(ang_);
			T bound2 = T(M_PI) + T(ang_);

			// if (dot_product > 0.0 && ( (angle < bound1) || (angle > bound2)  ) )
			if ( (angle < bound1) || (angle > bound2) ) 
				 residual[0] = wf_ / angle ;
			else
				 residual[0] = T(0.0);

    return true;
  }

 double wf_, ang_;

 private:
};


#endif