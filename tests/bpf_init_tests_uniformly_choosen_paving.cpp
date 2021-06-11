#define LOG_LEVEL Info
//#define INIT_METHOD    0

#include <gtest/gtest.h>
#include <bpf_localization/box_particle_filter.hpp>
#include <bpf_localization/tests.hpp>

// Declare a test
TEST(UniformlyChoosenPavingInitTest, testCase1)
{
    unsigned int state_size = 6;
    unsigned int N = 1234;
    unsigned int control_size = 2;
    unsigned int measures_size = 2;
    float dt = 1.;
    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

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

    TestBoxParticleFilter bpf(N, initial_box, dynamical_model);
    Particles particles = bpf.getParticles(); 

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "boxes not well pave initial box";

    EXPECT_EQ(particles.size(), N) << "In this test case we should have exactly N boxes";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "uniformly_choosen_paving_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
