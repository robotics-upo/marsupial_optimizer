#ifndef MISC_HPP
#define MISC_HPP

#include <Eigen/Core>
// #include <boost/utility.hpp>
// #include <boost/type_traits.hpp>

       
double fastSigmoid(double x);
double penaltyBoundToInterval(const double& v, const double m, const double& e); //v: The scalar that should be bounded, a: lower bound, b: upper bound,  e: safty margin 
double penaltyBoundAngle(double a1, double a2, double a3);

inline double fastSigmoid(double x)
{
  return x / (1 + fabs(x));
}

inline double penaltyBound(const double& v, const double m, const double& e)
{
  if ( (v > m+e)  || (v <= m-e) )
  {
    return (1.0*(v + (m + e)));
  }
  else
  {
    return 0.0;
  }
}


inline double penaltyBoundTime(const double& x, const double m, const double& e)
{
  if ( (x < m)  || (x >= m*e) )
  {
    return 1.0*(x  + m);
  }
  else
  {
    return 0.0;
  }
}

inline double penaltyBoundAngle(double a1, double a2, double a3)
{
  double ret1 , ret2;
  if ( a1 - a2 > M_PI /6 )
    ret1 = (a1 - a2);
  else
    ret1 = 0.;
  
  if ( a2 - a3 > M_PI /6 )
    ret2 = (a2 - a3);
  else
    ret2 = 0.;

  return (ret1+ret2);
}


#endif 