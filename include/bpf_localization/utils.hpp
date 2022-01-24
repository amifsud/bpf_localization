#ifndef UTILS
#define UTILS

#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>
#include <memory>
#include <math.h>

#define NaN std::numeric_limits<double>::quiet_NaN()

#ifndef WO_IBEX

#include "ibex/ibex.h"
using namespace ibex;

#include "bpf_localization/dynamical_systems/functions.hpp"

std::random_device rd;

class UniformDistribution 
{
    public:
        UniformDistribution(double lower = 0.0, double upper = 1.0):
            uniform_distribution_(lower, upper),
            generator_(rd())
        {
        }

        UniformDistribution(std::random_device device, double lower = 0.0, double upper = 1.0):
            uniform_distribution_(lower, upper),
            generator_(device())
        {
        }

        double get()
        {
            return uniform_distribution_(generator_);
        }

    protected:
        std::default_random_engine generator_;
        std::uniform_real_distribution<double> uniform_distribution_;
};

class ReturnIMU
{
    public:
        ReturnIMU(  const ExprNode& f1, const ExprNode& f2, const ExprNode& f3, 
                    const ExprNode& f4, const ExprNode& f5, const ExprNode& f6, 
                    const ExprNode& f7, const ExprNode& f8, const ExprNode& f9, 
                    const ExprNode& f10, bool in_rows=false) :
            vec(ExprVector::new_(Array<const ExprNode>(f1,f2,f3,f4,f5,f6,f7,f8,f9,f10),
                in_rows)) 
        {}

        operator const ExprVector&() const     { return vec; }
        operator const ExprNode&() const       { return vec; }
        const ExprIndex& operator[](int index) { return vec[index]; }
        const ExprVector& vec;
};

inline bool eps_equals( IntervalVector v1, IntervalVector v2, double eps = 1e-13)
{
    return v1.is_subset(IntervalVector(v2).inflate(eps)) && v2.is_subset(v1.inflate(eps));
}

#endif

#endif
