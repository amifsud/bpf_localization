#include "ibex/ibex.h"

using namespace ibex;

inline bool eps_equals( IntervalVector v1, IntervalVector v2, double eps = 1e-13)
{
    return v1.is_subset(IntervalVector(v2).inflate(eps)) && v2.is_subset(v1.inflate(eps));
}
