#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include "optimizer_local_planner.h"
#include <Eigen/Dense>

// double factor;

// struct EquiDistanceFunctor {
//   template <typename T>
//   bool operator()(const T* const pos1, const T* const pos2, const T* const pos3, const T* const pos4, T* residual) const {
//     residual[0] = factor * ( (10.0 - (pos1[0]-pos2[0].norm())) + (10.0 - (pos2[0]-pos3[0]).norm()) + (10.0 - (pos3[0]-pos4[0]).norm) );
//     return true;
//   }
// };

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  std::vector <Eigen::Vector3d> path;  
  Eigen::Vector3d pos1,pos2,pos3,pos4;

  Problem problem;

  for (size_t i=0 ; i < path.size()-3 ; i++)
  {
      problem.AddResidualBlock(new AutoDiffCostFunction<EquiDistanceFunctor, 1, 3, 3, 3, 3>(new EquiDistanceFunctor), NULL, &pos1, &pos2, &pos3, &pos4);
  //     CostFunction* cost_function = new AutoDiffCostFunction<EquiDistanceFunctor, 1, 3, 3, 3, 3>(new EquiDistanceFunctor);
  //     problem.AddResidualBlock(cost_function, nullptr, &pos1, &pos2, &pos3, &pos4);
  }


  Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}
