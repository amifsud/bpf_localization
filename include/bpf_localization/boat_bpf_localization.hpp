#define INIT_METHOD 0
#define RESAMPLING_METHOD 1 
#define RESAMPLING_DIRECTION 1

#include "bpf_localization/box_particle_filter.hpp"

class BoatBPFLocalization : public BoxParticleFilter
{
    protected:
        IntervalVector imu_measures_;
        IntervalVector gps_measures_;

    public:
        BoatBPFLocalization(IntervalVector& initial_box,
                            unsigned int N = 20,
                            double dt = 1e-3)
            :BoxParticleFilter(N, initial_box),
             imu_measures_(IMUDynamicalModel::control_size), 
             gps_measures_(IMUDynamicalModel::measures_size)
        {
            dynamical_model_ = std::shared_ptr<DynamicalModel>(new IMUDynamicalModel(dt));
        }

        void IMUCallback(IntervalVector& imu)
        {
            imu_measures_ = imu;
            prediction(imu_measures_);
        }

        void GPSCallback(IntervalVector& gps)
        {
            gps_measures_ = gps;
            correction(gps_measures_);
        }

        IntervalVector contract(IntervalVector& innovation, IntervalVector& box)
        {
            return box;
        }
};
