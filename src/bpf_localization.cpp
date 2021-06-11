#define GUARANTED_RESAMPLING
#define UNIFORMLY_CHOOSEN_PAVING_INIT

#include <bpf_localization/box_particle_filter.hpp>
#include <ros/ros.h>

class DynamicalModel: public AbstractDynamicalModel
{
    public:
        DynamicalModel( unsigned int state_size, unsigned int control_size, 
                        unsigned int measures_size, double dt,
                        Vector measures_noise_diams, Vector process_noise_diams,
                        Method method, double precision)
            :AbstractDynamicalModel(state_size, control_size, measures_size, dt, 
                                    measures_noise_diams, process_noise_diams,
                                    method, precision)
        {}

    protected:
        void setDynamicalModel(IntervalVector& control)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            dynamical_model_             
                = new Function(state_variable_, 
                        Return( Interval(1.),
                                Interval(1.),
                                control[0]));
            ROS_DEBUG_STREAM("set dynamical model end");
        }

        void setMeasuresModel(IntervalVector& measures)
        {
            ROS_DEBUG_STREAM("set dynamical model begin");
            measures_model_ 
                = new Function(state_variable_, Return( state_variable_[0],
                                                        state_variable_[1]
                                                        +state_variable_[2]));
            ROS_DEBUG_STREAM("set dynamical model end");
        }
};


class LocalizationBoxParticleFilter: public BoxParticleFilter
{
    public:
        LocalizationBoxParticleFilter(  unsigned int N, IntervalVector initial_box,
                                        AbstractDynamicalModel* dynamical_model)
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

    unsigned int state_size = 3;
    unsigned int control_size = 1;
    unsigned int measures_size = 2;
    double dt = 1.0; // sampling time

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-10.0, 10.0);
    initial_box[1]= Interval(-10.0, 10.0);
    initial_box[2]= Interval(-10.0, 10.0);

    unsigned int N = 18; // number of boxes

    Vector process_noise_diams(6);
    process_noise_diams[0] = 1e-2;
    process_noise_diams[1] = 1e-2;
    process_noise_diams[2] = 1e-2;
    process_noise_diams[3] = 1e-2;
    process_noise_diams[4] = 1e-2;
    process_noise_diams[5] = 1e-2;

    Vector measures_noise_diams(6);
    measures_noise_diams[0] = 1e-2;
    measures_noise_diams[1] = 1e-2;
    measures_noise_diams[2] = 1e-2;
    measures_noise_diams[3] = 1e-2;
    measures_noise_diams[4] = 1e-2;
    measures_noise_diams[5] = 1e-2;

    DynamicalModel* dynamical_model 
        = new DynamicalModel(   state_size, control_size, measures_size, dt, 
                                measures_noise_diams, process_noise_diams, RK4, 1e-6);

    LocalizationBoxParticleFilter bpf(N, initial_box, dynamical_model);

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
