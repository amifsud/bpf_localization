#define MULTINOMIAL_RESAMPLING
#define UNIFORMLY_CHOOSEN_PAVING_INIT

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
                                                                    state_variable_[1]
                                                                    +state_variable_[2]));
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
TEST(UniformlyChoosenInitTest, testCase1)
{
    unsigned int state_size = 6;
    unsigned int N = 12738;
    unsigned int control_size = 2;
    float dt = 1.;
    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-10.0, 10.0);
    initial_box[1]= Interval(-10.0, 10.0);
    initial_box[2]= Interval(-10.0, 10.0);

    LocalizationBoxParticleFilter bpf(N, state_size, control_size, dt, initial_box);

    EXPECT_EQ(N, bpf.getBoxes().size());
}

// Declare another test
TEST(UniformlyChoosenInitTest, testCase2)
{
    unsigned int state_size = 6;
    unsigned int N = pow(pow(2,state_size),2)+24;
    unsigned int control_size = 2;
    float dt = 1.;
    IntervalVector initial_box(state_size);
    initial_box[0]= Interval(-10.0, 10.0);
    initial_box[1]= Interval(-10.0, 10.0);
    initial_box[2]= Interval(-10.0, 10.0);

    LocalizationBoxParticleFilter bpf(N, state_size, control_size, dt, initial_box);

    EXPECT_EQ(N, bpf.getBoxes().size());
    EXPECT_NE(pow(pow(2,state_size),2), bpf.getBoxes().size());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "uniformly_choosen_paving_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
