#include "catenary_solver_SM.hpp"
// #include "catenary_solver_FC.hpp"

int main(int argc, char** argv) {

  CatenarySolver cS;

  double a_, x0_, y0_;
  double x1_,y1_, x2_, y2_, l_;
  x1_ = 10.0;
  y1_ = 30.0;
  x2_ = 50.0;
  y2_ = 40.0;
  l_ = 50.0;
  cS.setMaxNumIterations(200);
  cS.solve(x1_,y1_,x2_,y2_, l_, a_, x0_, y0_);

  std::cout << std::endl <<"a_: " << a_ << " , x0: " << x0_ << " , y0: "<< y0_ << std::endl;

  double h_ = fabs(y0_)- a_;
  double xc_ = x1_ + x0_;
  double yc_ = y1_ - h_;  

    // h_value = fabs(bs_Y0)-c_value;
    // Xc=X1+bs_X0;
    // Yc=Y1-h_value;

  std::cout <<"h: " << h_ << " , xc: " << xc_ << " , yc: "<< yc_ << std::endl;

  return 0;
}