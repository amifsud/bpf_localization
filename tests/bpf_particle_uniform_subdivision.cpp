#define LOG_LEVEL Info
#define INIT_METHOD   1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

// Declare a test
TEST(UniformSubdivisionTest, testCase1)
{
    unsigned int state_size = 6;

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise(SUBDIVISION_TYPE::ALL_DIMENSIONS));

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "boxes not well pave initial box";

    EXPECT_TRUE(subdiviseOverAllDimensionsTest(&particles))
        << "the paving is not uniform";

    unsigned int N = pow(2,state_size);
    for(auto it = particles.begin(); it != particles.end(); ++it)
        EXPECT_TRUE(std::abs(it->weight() - 1./N) < 1e-7);
}

// Declare another test
TEST(UniformSubdivisionTest, testCase2)
{
    unsigned int state_size = 2;

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise(SUBDIVISION_TYPE::ALL_DIMENSIONS));

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "Boxes don't well pave initial box";

    EXPECT_TRUE(subdiviseOverAllDimensionsTest(&particles))
        << "the paving is not uniform";

    unsigned int N = pow(2,state_size);
    for(auto it = particles.begin(); it != particles.end(); ++it)
        EXPECT_TRUE(std::abs(it->weight() - 1./N) < 1e-7);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "uniform_subdivision_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
