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

    EXPECT_EQ(N, particles.size()) << "In this test case we should have exactly N boxes";

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
TEST(UniformPavingInitTest, testCase2)
{
    unsigned int state_size = 2;
    unsigned int N = pow(pow(2,state_size),2) + 10;
    unsigned int control_size = 2;
    float dt = 1.;
    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-2.0, 2.0);
    initial_box[1]= Interval(-2.0, 2.0);

    LocalizationBoxParticleFilter bpf(N, state_size, control_size, dt, initial_box);
    Particles particles = bpf.getParticles(); 

    EXPECT_TRUE(particles.wellPavedTest(initial_box))
        << "Boxes don't well pave initial box";

    EXPECT_NE(particles.size(), N);
    EXPECT_EQ(particles.size(), pow(pow(2,state_size),2));

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
    ros::init(argc, argv, "uniform_paving_init_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
