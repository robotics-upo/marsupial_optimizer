#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include "marsupial_g2o/ceres_contrain_equidistance.hpp"
#include "marsupial_g2o/ceres_contrain_distance_obstacles.hpp"
#include "marsupial_g2o/ceres_contrain_kinematics.hpp"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


// class EquiDistanceFunctor {

// public:
//   EquiDistanceFunctor(double weight_factor, double initial_distance): wf_(weight_factor), int_d_(initial_distance) {}

//   template <typename T>
//   bool operator()(const T* const pose1, const T* const pose2, T* residual) const {
//     residual[0] = wf_ * ( (pow(int_d_,2) - pow(pose1[0]-pose2[0],2)) + (pow(int_d_,2) - pow(pose1[1]-pose2[1],2)) + (pow(int_d_,2) - pow(pose1[2]-pose2[2],2)) );
//     return true;
//   }

//  double wf_, int_d_;

//  private:
// };

// class ObstacleDistanceFunctor {

// public:
//   ObstacleDistanceFunctor(double factor, double safty_bound): f_(factor), sb_(safty_bound) {}
  
// 	NearNeighbor nn;
// 	Eigen::Vector3d near_;
// 	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
// 	pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
  
//   double f_, sb_;

//   template <typename T>
//   bool operator()(const T* const pose1, T* residual) const {

//     T  d_;

//     Eigen::Vector3d pose;
//     pose = Eigen::Vector3d(pose1[0],pose1[1],pose1[2]);
//     near_ = nn.nearestObstacleVertex(kdT_From_NN , pose, obstacles_Points);
// 		d_ = (pose -near_).norm();
    
//     if (d_ < sb_)
//       residual[0] =  f_ * exp(sb_ - 4.0*d_);
//     else
//       residual[0] = T(0.0);
//     return true;
//   }

//  private:

// };


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  std::vector <Eigen::Vector3d> path;  
  std::vector <double> length;  

  double pose1[4];
  double pose2[4];
  double pose3[4];

  Problem problem;

  for (int i = 0; i < path.size() - 1 ; ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    pose2[0] = path[i+1].x(); pose2[1] = path[i+1].y(); pose2[2] = path[i+1].z(); pose2[3] = length[i+1];
    CostFunction* cost_function1  = new AutoDiffCostFunction<EquiDistanceFunctor, 1, 4, 4>(new EquiDistanceFunctor(1.0, 2.0)); 
    problem.AddResidualBlock(cost_function1, NULL, pose1, pose2);
  }

  // for (int i = 0; i < path.size(); ++i) {
  //   pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
  //   CostFunction* cost_function2  = new AutoDiffCostFunction<ObstacleDistanceFunctor, 1, 4>(new ObstacleDistanceFunctor(1.0, 1.0)); 
  //   problem.AddResidualBlock(cost_function2, NULL, pose1);
  // }

  for (int i = 0; i < path.size() - 2 ; ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    pose2[0] = path[i+1].x(); pose2[1] = path[i+1].y(); pose2[2] = path[i+1].z(); pose2[3] = length[i+1];
    pose3[0] = path[i+2].x(); pose3[1] = path[i+2].y(); pose3[2] = path[i+2].z(); pose3[3] = length[i+2];
    CostFunction* cost_function3  = new AutoDiffCostFunction<KinematicsFunctor, 1, 4, 4, 4>(new KinematicsFunctor(1.0, 2.0)); 
    problem.AddResidualBlock(cost_function3, NULL, pose1, pose2, pose3);
  }

  Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}
