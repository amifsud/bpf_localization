#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

// Declare a test
TEST(UniformPavingInitTest, testCase1)
{
    auto dynamical_model = std::shared_ptr<TurtleBotDynamicalModel>(
            new TurtleBotDynamicalModel());

    unsigned int N = pow(pow(2,dynamical_model->stateSize()),1);

    IntervalVector initial_box(dynamical_model->stateSize());
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);

    BoxParticleFilter bpf(N, initial_box, dynamical_model);
    Particles particles = bpf.getParticles(); 

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "boxes not well pave initial box";

    EXPECT_TRUE(subdiviseOverAllDimensionsTest(&particles))
        << "the paving is not uniform";
}

// Declare another test
TEST(UniformPavingInitTest, testCase2)
{
    auto dynamical_model = std::shared_ptr<TurtleBotDynamicalModel>(
            new TurtleBotDynamicalModel());

    unsigned int N = pow(pow(2,dynamical_model->stateSize()),2) + 10;

    IntervalVector initial_box(dynamical_model->stateSize());
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);

    BoxParticleFilter bpf(N, initial_box, dynamical_model);
    Particles particles = bpf.getParticles(); 

    EXPECT_TRUE(wellPavedTest(&particles, initial_box))
        << "Boxes don't well pave initial box";

    EXPECT_LT(particles.size(), N);

    EXPECT_TRUE(subdiviseOverAllDimensionsTest(&particles))
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
