#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <stdlib.h>
#include <ros/console.h>
#include <gtest/gtest.h>
#include <bpf_localization/box_particle_filter.hpp>

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
                                                                    state_variable_[0]
                                                                    +state_variable_[0]));
            

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
            geometrical_subdivision_map[0] = std::pair<int, double>(2, 1e-4);
            geometrical_subdivision_map[2] = std::pair<int, double>(2, 1e-4);
            geometrical_subdivision_map[4] = std::pair<int, double>(2, 1e-4);
            #endif

            setDynamicalModel();
        }
};

// Declare a test
TEST(UniformPavingInitTest, testCase1)
{
    unsigned int state_size = 6;
    unsigned int N = pow(pow(2,state_size),1);
    unsigned int control_size = 2;
    float dt = 1.;
    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

    LocalizationBoxParticleFilter bpf(N, state_size, control_size, dt, initial_box);
    Particles particles = bpf.getParticles(); 

    EXPECT_TRUE(particles.wellPavedTest(initial_box))
        << "boxes not well pave initial box";

    EXPECT_TRUE(particles.subdiviseOverAllDimensionsTest())
        << "the paving is not uniform";
}

// Declare another test
TEST(UniformPavingInitTest, testCase2)
{
    unsigned int state_size = 6;
    unsigned int N = pow(pow(2,state_size),2) + 10;
    unsigned int control_size = 2;
    float dt = 1.;
    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

    LocalizationBoxParticleFilter bpf(N, state_size, control_size, dt, initial_box);
    Particles particles = bpf.getParticles(); 

    EXPECT_TRUE(particles.wellPavedTest(initial_box))
        << "Boxes don't well pave initial box";

    EXPECT_LT(particles.size(), N);

    EXPECT_TRUE(particles.subdiviseOverAllDimensionsTest())
        << "the paving is not uniform";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "uniform_paving_init_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
