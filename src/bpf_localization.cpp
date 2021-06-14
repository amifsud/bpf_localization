#define GUARANTED_RESAMPLING
#define UNIFORMLY_CHOOSEN_PAVING_INIT

#include <bpf_localization/box_particle_filter.hpp>
#include <bpf_localization/dynamical_systems.hpp>
#include <ros/ros.h>

class LocalizationBoxParticleFilter: public BoxParticleFilter
{
    public:
        LocalizationBoxParticleFilter(  unsigned int N, IntervalVector initial_box,
                                        DynamicalModel* dynamical_model)
            : BoxParticleFilter(N, initial_box, dynamical_model)
        {
            #if RESAMPLING_DIRECTION == 1
            geometrical_subdivision_map.insert(std::pair<int, int>(0, 1));
            geometrical_subdivision_map.insert(std::pair<int, int>(1, 1));
            geometrical_subdivision_map.insert(std::pair<int, int>(2, 1));
            #endif
        }
};

int main(int argc, char **argv) 
{

    IntervalVector initial_box(TurtleBotDynamicalModel::state_size);
    initial_box[0]= Interval(-10.0, 10.0);
    initial_box[1]= Interval(-10.0, 10.0);
    initial_box[2]= Interval(-10.0, 10.0);

    unsigned int N = 18; // number of boxes

    TurtleBotDynamicalModel* dynamical_model = new TurtleBotDynamicalModel();

    LocalizationBoxParticleFilter bpf(N, initial_box, dynamical_model);

    Particles particles = bpf.getParticles();

    IntervalVector control(TurtleBotDynamicalModel::control_size);
    control[0]= Interval(2.78, 2.79);
    control[1]= Interval(2.78, 2.79);
    bpf.prediction(control);

    IntervalVector measures(TurtleBotDynamicalModel::measures_size);
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
