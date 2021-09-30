#include "bpf_localization/box_particle_filter.hpp"

class BoatBPFLocalization : public BoxParticleFilter
{
    BoatBPFLocalization(unsigned int N, 
                        IntervalVector& initial_box,
                        DynamicalModel* dynamical_model)
        :BoxParticleFilter(N, initial_box, dynamical_model)
    {

    }
};
