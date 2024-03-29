#define INIT_METHOD 0
#define RESAMPLING_METHOD 1 
#define RESAMPLING_DIRECTION 1

#include "bpf_localization/box_particle_filter.hpp"
#include "bpf_localization/dynamical_systems/ins.hpp"

#define DEFAULT_PARTICLES_NUMBER 16

namespace bpf
{
    class BoatBPFLocalization : public BoxParticleFilter
    {
        protected:
            IntervalVector imu_measures_;
            IntervalVector gps_measures_;

        public:
            BoatBPFLocalization(const IntervalVector& initial_box,
                                bool parallelize = false,
                                unsigned int N = DEFAULT_PARTICLES_NUMBER,
                                double dt = 1.)
                :BoxParticleFilter(N, initial_box, parallelize),
                 imu_measures_(dynamical_systems::INS::control_size), 
                 gps_measures_(dynamical_systems::INS::measures_size)
            {
                dynamical_model_ 
                    = std::shared_ptr<dynamical_systems::DynamicalSystem>(
                            new dynamical_systems::INS(dt));
            }

            BoatBPFLocalization(double pos, double vel, double theta,
                                bool parallelize = false, 
                                unsigned int N = DEFAULT_PARTICLES_NUMBER,
                                double dt = 1.)
                :BoxParticleFilter(N, get_init(pos, vel, theta), parallelize),
                 imu_measures_(dynamical_systems::INS::control_size), 
                 gps_measures_(dynamical_systems::INS::measures_size)
            {
                dynamical_model_ 
                    = std::shared_ptr<dynamical_systems::DynamicalSystem>(
                            new dynamical_systems::INS(dt));
            }

            BoatBPFLocalization(Particles& particles,
                                bool parallelize = false, 
                                unsigned int N = DEFAULT_PARTICLES_NUMBER,
                                double dt = 1.)
                :BoxParticleFilter(N, particles, parallelize),
                 imu_measures_(dynamical_systems::INS::control_size), 
                 gps_measures_(dynamical_systems::INS::measures_size)
            {
                dynamical_model_ 
                    = std::shared_ptr<dynamical_systems::DynamicalSystem>(
                            new dynamical_systems::INS(dt));
            }

            const IntervalVector get_init(double pos, double vel, double theta)
            {
                assert(pos > 0. && "initial incertitude on position must be > 0");
                assert(vel > 0. && "initial incertitude on velocity must be > 0");
                assert(theta > 0. && "initial incertitude on orientation must be > 0");

                IntervalVector initial_box(dynamical_systems::INS::state_size);
                initial_box[0] = Interval(cos(theta), 1.);
                initial_box[1] = Interval(sin(-theta)/sqrt(3), sin(theta)/sqrt(3));
                initial_box[2] = Interval(sin(-theta)/sqrt(3), sin(theta)/sqrt(3));
                initial_box[3] = Interval(sin(-theta)/sqrt(3), sin(theta)/sqrt(3));
                initial_box[4] = Interval(-pos, pos);
                initial_box[5] = Interval(-pos, pos);
                initial_box[6] = Interval(-pos, pos);
                initial_box[7] = Interval(-vel, vel);
                initial_box[8] = Interval(-vel, vel);
                initial_box[9] = Interval(-vel, vel);
                return IntervalVector(initial_box);
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
}
