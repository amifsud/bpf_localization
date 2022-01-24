#define LOG_LEVEL Info
//#define INIT_METHOD     0

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

// Declare a test
TEST(RandomSubdivisionTest, testCase1)
{
    unsigned int state_size = 2;
    unsigned int N = pow(2,7); // = 128

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise(SUBDIVISION_TYPE::RANDOM, N));

    std::map<int, std::pair<int, double>> geometrical_subdivision_map;
    geometrical_subdivision_map[0] = std::pair<int, double>(1, 1);
    geometrical_subdivision_map[1] = std::pair<int, double>(1, 1);

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "boxes not well pave initial box";

    EXPECT_TRUE(subdiviseOverRandomDimensionsTest
            (&particles, initial_box, 
             geometrical_subdivision_map));

    EXPECT_EQ(particles.size(), N) << "In this test case we should have exactly N boxes";

    for(auto it = particles.begin(); it != particles.end(); ++it)
        EXPECT_TRUE(std::abs(it->weight() - 1./N) < 1e-7);
}

TEST(RandomSubdivisionTest, testCase2)
{
    unsigned int state_size = 2;
    unsigned int N = pow(2,7); // = 128
    unsigned int offset = 50; // has to be < N

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise(SUBDIVISION_TYPE::RANDOM, N+offset));

    std::map<int, std::pair<int, double>> geometrical_subdivision_map;
    geometrical_subdivision_map[0] = std::pair<int, double>(1, 1);
    geometrical_subdivision_map[1] = std::pair<int, double>(1, 1);

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "boxes not well pave initial box";

    EXPECT_TRUE(subdiviseOverRandomDimensionsTest
            (&particles, initial_box, 
             geometrical_subdivision_map));

    EXPECT_EQ(particles.size(), N+offset) << "In this test case we should have exactly N boxes";

    unsigned int i = 0;
    for(auto it = particles.begin(); it != particles.end(); ++it, ++i)
    {
        if(i < N-offset)
        {
            EXPECT_TRUE(std::abs(it->weight() - 1./N) < 1e-7);
        }
        else
        {
            EXPECT_TRUE(std::abs(it->weight() - 1./N/2.) < 1e-7);
        }
    }
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
