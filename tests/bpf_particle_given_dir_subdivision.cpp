#define LOG_LEVEL Info
#define SUBDIVISE_OVER_GIVEN_DIRECTION

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

void test(  Particles* particles, unsigned int N, 
            IntervalVector* initial_box, unsigned int subdivision_dir)
{
    EXPECT_TRUE(wellPavedTest(particles, *initial_box))
        << "boxes don't well pave initial box";

    EXPECT_TRUE(subdiviseOverGivenDirectionTest(particles, *initial_box, subdivision_dir))
        << "test of the subdivision over given direction failed";

    EXPECT_EQ(particles->size(), N) << "In this test case we should have exactly N boxes";

    for(auto it = particles->begin(); it != particles->end(); it++)
        EXPECT_TRUE(std::abs(it->weight() - 1./N) < 1e-7);
}


// Declare a test
TEST(GivenDirSubdivisionTest, testCase1)
{
    unsigned int state_size = 6;
    unsigned int N = 1234;
    unsigned int subdivision_dir = 1;

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise( SUBDIVISION_TYPE::GIVEN, 
                                            N, subdivision_dir));

    test(&particles, N, &initial_box, subdivision_dir);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "given_dir_subdivision_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
