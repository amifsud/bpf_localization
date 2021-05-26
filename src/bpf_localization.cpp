#define GUARANTED_RESAMPLING
#define UNIFORMLY_CHOOSEN_PAVING_INIT

#include <bpf_localization/box_particle_filter.hpp>
#include <ros/ros.h>

class LocalizationBoxParticleFilter: public BoxParticleFilter
{
    protected:
        void setDynamicalModel()
        {
            ROS_DEBUG_STREAM("set dynamics begin");
            dynamics_model_ 
                = new Function(state_variable_, 
                        Return( Interval(1.),
                                Interval(1.),
                                (*control_)[0]));

            measures_model_ = new Function(state_variable_, Return( state_variable_[0],
                                                                    state_variable_[1]
                                                                    +state_variable_[2]));
            ROS_DEBUG_STREAM("set dynamics end");
        }

    public:
        LocalizationBoxParticleFilter(  unsigned int N, unsigned int state_size, 
                                        unsigned int control_size, float dt, 
                                        IntervalVector initial_box)
            : BoxParticleFilter(N, state_size, control_size, dt, initial_box)
        {
            // If ivp
            integration_method_ = RK4;
            precision_ = 1e-4;

            #if RESAMPLING_DIRECTION == 1
            geometrical_subdivision_map.insert(std::pair<int, int>(0, 1));
            geometrical_subdivision_map.insert(std::pair<int, int>(1, 1));
            geometrical_subdivision_map.insert(std::pair<int, int>(2, 1));
            #endif

            setDynamicalModel();
        }
};

int main(int argc, char **argv) 
{

    unsigned int state_size = 3;
    unsigned int control_size = 1;
    unsigned int measures_size = 2;
    float dt = 1.0; // sampling time

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-10.0, 10.0);
    initial_box[1]= Interval(-10.0, 10.0);
    initial_box[2]= Interval(-10.0, 10.0);

    unsigned int N = 18; // number of boxes

    LocalizationBoxParticleFilter bpf(N, state_size, control_size, dt, initial_box);

    Particles particles = bpf.getParticles();

    IntervalVector control(control_size);
    control[0]= Interval(2.78, 2.79);
    bpf.prediction(control);

    IntervalVector measures(measures_size);
    measures[0] = Interval(1., 2.);
    measures[1] = Interval(2., 3.);
    bpf.correction(measures);

    ros::init(argc, argv, "bpf_localization");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
