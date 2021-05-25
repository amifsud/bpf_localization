#define LOG_LEVEL Info
//#define INIT_METHOD     0

#include <gtest/gtest.h>
#include <bpf_localization/box_particle_filter.hpp>

// Declare a test
TEST(RandomSubdivisionTest, testCase1)
{
    unsigned int state_size = 2;
    unsigned int N = 1234;

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise(SUBDIVISION_TYPE::RANDOM, N));

    std::map<int, std::pair<int, double>> geometrical_subdivision_map;
    geometrical_subdivision_map[0] = std::pair<int, double>(1, 1);
    geometrical_subdivision_map[1] = std::pair<int, double>(1, 1);

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
    ros::init(argc, argv, "random_subdivision_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
