#define LOG_LEVEL Info
#define INIT_METHOD   1

#include <stdlib.h>
#include <ros/console.h>
#include <gtest/gtest.h>
#include <bpf_localization/box_particle_filter.hpp>

// Declare a test
TEST(UniformPavingParticlesTest, testCase1)
{
    unsigned int state_size = 6;
    unsigned int N = pow(2,state_size);

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);
    initial_box[2]= Interval(-2.0, 2.0);
    initial_box[3]= Interval(-2.0, 2.0);
    initial_box[4]= Interval(-2.0, 2.0);
    initial_box[5]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise());

    EXPECT_TRUE(particles.wellPavedTest(initial_box))
        << "boxes not well pave initial box";

    EXPECT_EQ(particles.size(), N) << "In this test case we should have exactly N boxes";

    bool equal_volume = true;
    bool hypercube = true;
    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        if(particles[0].box_.volume() != particles[i].box_.volume()) equal_volume = false;
        for(unsigned int u = 0; u < particles[i].box_.size(); ++u)
            if(particles[i].box_[0].diam() != particles[i].box_[u].diam()) hypercube 
                = false;
    }

    EXPECT_TRUE(hypercube)    << "Each box should be an hypercube";
    EXPECT_TRUE(equal_volume) << "Volume of each box should be equal to the others";
}

// Declare another test
TEST(UniformPavingParticlesTest, testCase2)
{
    unsigned int state_size = 2;
    unsigned int N = pow(2,state_size);

    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);

    Particle particle(initial_box, 1.);
    Particles particles(particle.subdivise());

    EXPECT_TRUE(particles.wellPavedTest(initial_box))
        << "Boxes don't well pave initial box";

    EXPECT_EQ(particles.size(), N);

    bool equal_volume = true;
    bool hypercube = true;
    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        if(particles[0].box_.volume() != particles[i].box_.volume()) equal_volume = false;
        for(unsigned int u = 0; u < particles[i].box_.size(); ++u)
            if(particles[i].box_[0].diam() != particles[i].box_[u].diam()) hypercube 
                = false;
    }

    EXPECT_TRUE(hypercube)    << "Each box should be an hypercube";
    EXPECT_TRUE(equal_volume) << "Volume of each box should be equal to the others";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "uniform_paving_particles_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
