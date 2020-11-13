/*
Optimizer Local Planner based in g2o for Marsupial Robotics Configuration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */


#ifndef _OPTIMIZER_LOCAL_PLANNER_H_
#define _OPTIMIZER_LOCAL_PLANNER_H_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include "ceres_constrain_equidistance.hpp"



#endif