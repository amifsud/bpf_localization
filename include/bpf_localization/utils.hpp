#ifndef UTILS
#define UTILS

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

#ifndef WO_IBEX

#include "ibex/ibex.h"
using namespace ibex;

inline bool eps_equals( IntervalVector v1, IntervalVector v2, double eps = 1e-13)
{
    return v1.is_subset(IntervalVector(v2).inflate(eps)) && v2.is_subset(v1.inflate(eps));
}

class IntervalVectorXYZ
{
    public:
        Interval x;
        Interval y;
        Interval z;

    public:
        IntervalVectorXYZ()
        {
            setZero();
        }

        void setZero()
        {
            x = Interval(0.,0.);
            y = Interval(0.,0.);
            z = Interval(0.,0.);
        }
};

ostream& operator<<(ostream& os, const IntervalVectorXYZ& i)
{
    os << i.x << i.y << i.z;
    return os;
}
#endif

#endif
