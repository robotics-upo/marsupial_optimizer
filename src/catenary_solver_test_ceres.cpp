#include "catenary_solver_test.hpp"
// #include "catenary_solver_FC.hpp"

int main(int argc, char** argv) {

  CatenarySolver cS;

  std::vector<Eigen::Vector3d> v_points_catenary_3D;

  double a_, x0_, y0_;
  double x1_, y1_, z1_, x2_, y2_, z2_, l_;
  x1_ = -3.2;
  y1_ = -0.2;
  z1_ = 2.25;
  x2_ = -3.053719;
  y2_ = -0.199929;
  z2_ = 2.917853;
  l_ = 0.77;
  // x2_ = -3.600000;
  // y2_ = -0.200000;
  // z2_ = 3.000000;
  // l_ = 0.851700;
  
  cS.setMaxNumIterations(200);
  cS.solve(x1_, y1_, z1_, x2_, y2_, z2_, l_, a_, x0_, y0_, v_points_catenary_3D);

  // std::cout << std::endl <<"a_: " << a_ << " , x0: " << x0_ << " , y0: "<< y0_ << std::endl;

  // double h_ = fabs(y0_)- a_;
  // double xc_ = x1_ + x0_;
  // double yc_ = y1_ - h_;  

  // std::cout <<"h: " << h_ << " , xc: " << xc_ << " , yc: "<< yc_ << std::endl;

  return 0;
}