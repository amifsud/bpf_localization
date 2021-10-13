#ifndef UTILS
#define UTILS

#include "ibex/ibex.h"
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>
#include <memory>
#include <math.h>
//#include <Eigen/Core>
//#include <Eigen/Cholesky>

#define NaN std::numeric_limits<double>::quiet_NaN()

using namespace ibex;

inline bool eps_equals( IntervalVector v1, IntervalVector v2, double eps = 1e-13)
{
    return v1.is_subset(IntervalVector(v2).inflate(eps)) && v2.is_subset(v1.inflate(eps));
}

#endif
