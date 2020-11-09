#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


std::vector<double> vector_example;

// for (size_t i=0 ; i < vector_example.size() ; i++){
  struct ConstrainEquiDistancePos {
    template <typename T>
    bool operator()(const T* const ED_pos1, const T* const ED_pos2, const T* const ED_pos3, T* residual) const {
      residual[0] = factor * ( (10.0 - ED_pos1[0]) + (10.0 - ED_pos1[0]) + (10.0 - ED_pos3[0]) );
      return true;
    }
  };
// }


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double ED_pos1 = 0.5;
  const double initial_x = ED_pos1;
  // Build the problem.
  Problem problem;
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<ConstrainEquiDistancePos, 1, 1>(new ConstrainEquiDistancePos);
  problem.AddResidualBlock(cost_function, nullptr, &ED_pos1);
  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "ED_pos1 : " << initial_x << " -> " << ED_pos1 << "\n";
  return 0;
}