#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include "marsupial_g2o/ceres_contrain_equidistance.hpp"
#include "marsupial_g2o/ceres_contrain_obstacles.hpp"
#include "marsupial_g2o/ceres_contrain_kinematics.hpp"
#include "marsupial_g2o/ceres_contrain_time.hpp"
#include "marsupial_g2o/ceres_contrain_velocity.hpp"
#include "marsupial_g2o/ceres_contrain_acceleration.hpp"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  std::vector <Eigen::Vector3d> path;  
  std::vector <double> length;  
  std::vector <double> initial_time;  
  double initial_velocity;

	NearNeighbor nn_;

  double pose1[4];
  double pose2[4];
  double pose3[4];
  double time1[1];
  double time2[1];

  // pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN = nn_.kdtree;
	// pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points = nn_.obs_points;

  Problem problem;

  for (int i = 0; i < path.size() - 1 ; ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    pose2[0] = path[i+1].x(); pose2[1] = path[i+1].y(); pose2[2] = path[i+1].z(); pose2[3] = length[i+1];
    CostFunction* cost_function1  = new AutoDiffCostFunction<EquiDistanceFunctor, 1, 4, 4>(new EquiDistanceFunctor(1.0, 2.0)); 
    problem.AddResidualBlock(cost_function1, NULL, pose1, pose2);
  }

  // for (int i = 0; i < path.size(); ++i) {
  //   pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
  //   CostFunction* cost_function2  = new AutoDiffCostFunction<ObstacleDistanceFunctor, 1, 4>(new ObstacleDistanceFunctor(1.0, 1.0, nn_.kdtree, nn_.obs_points)); 
  //   problem.AddResidualBlock(cost_function2, NULL, pose1);
  // }
  for (int i = 0; i < path.size(); ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    CostFunction* cost_function2  = new AutoDiffCostFunction<ObstacleDistanceFunctor::Affine4DWithDistortion, 1, 4>(new ObstacleDistanceFunctor::Affine4DWithDistortion(1.0, 1.0, nn_.kdtree, nn_.obs_points)); 
    problem.AddResidualBlock(cost_function2, NULL, pose1);
  }

  for (int i = 0; i < path.size() - 2 ; ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    pose2[0] = path[i+1].x(); pose2[1] = path[i+1].y(); pose2[2] = path[i+1].z(); pose2[3] = length[i+1];
    pose3[0] = path[i+2].x(); pose3[1] = path[i+2].y(); pose3[2] = path[i+2].z(); pose3[3] = length[i+2];
    CostFunction* cost_function3  = new AutoDiffCostFunction<KinematicsFunctor, 1, 4, 4, 4>(new KinematicsFunctor(1.0, 25.0)); 
    problem.AddResidualBlock(cost_function3, NULL, pose1, pose2, pose3);
  }

  for (int i = 0; i < initial_time.size() ; ++i) {
    time1[0] = initial_time[i];
    CostFunction* cost_function4  = new AutoDiffCostFunction<TimeFunctor, 1, 1>(new TimeFunctor(1.0, initial_time[i])); 
    problem.AddResidualBlock(cost_function4, NULL, time1);
  }

  for (int i = 0; i < initial_time.size() ; ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    pose2[0] = path[i+1].x(); pose2[1] = path[i+1].y(); pose2[2] = path[i+1].z(); pose2[3] = length[i+1];
    time1[0] = initial_time[i];
    CostFunction* cost_function5  = new AutoDiffCostFunction<VelocityFunctor, 1, 4, 4, 1>(new VelocityFunctor(1.0, initial_velocity)); 
    problem.AddResidualBlock(cost_function5, NULL, pose1, pose2, time1);
  }

  for (int i = 0; i < initial_time.size() - 1; ++i) {
    pose1[0] = path[i].x(); pose1[1] = path[i].y(); pose1[2] = path[i].z(); pose1[3] = length[i];
    pose2[0] = path[i+1].x(); pose2[1] = path[i+1].y(); pose2[2] = path[i+1].z(); pose2[3] = length[i+1];
    pose3[0] = path[i+2].x(); pose3[1] = path[i+2].y(); pose3[2] = path[i+2].z(); pose3[3] = length[i+2];
    time1[0] = initial_time[i];
    time2[0] = initial_time[i+1];
    CostFunction* cost_function6  = new AutoDiffCostFunction<AccelerationFunctor, 1, 4, 4, 4, 1, 1>(new AccelerationFunctor(1.0, initial_velocity)); 
    problem.AddResidualBlock(cost_function6, NULL, pose1, pose2, pose3, time1, time2);
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
