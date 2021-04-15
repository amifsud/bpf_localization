#include "ibex/ibex.h"

using namespace ibex;

class BoxParticleFilter
{
    public:
        int a = 1;
        
        IntervalVector yinit;

    public:
        BoxParticleFilter(): yinit(3)
        {
            yinit[0]= Interval(1.0);
            yinit[1]= 1.0;
            yinit[2]= 3.0;
        }

        float get()
        {
            return float(yinit.lb()[2]);
        }
};
