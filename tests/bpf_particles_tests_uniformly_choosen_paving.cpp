#define LOG_LEVEL Info
//#define INIT_METHOD     0

#include <gtest/gtest.h>
#include <bpf_localization/box_particle_filter.hpp>

// Declare a test
TEST(UniformlyChoosenPavingParticlesTest, testCase1)
{
    unsigned int state_size = 6;
    unsigned int N = 1234;

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise(SUBDIVISION_TYPE::RANDOM, N));


    EXPECT_TRUE(particles.wellPavedTest(initial_box))
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
    ros::init(argc, argv, "uniformly_choosen_particles_paving_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
