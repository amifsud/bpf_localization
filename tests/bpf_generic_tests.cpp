#define LOG_LEVEL Info
#define INIT_METHOD 1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>
#include <bpf_localization/dynamical_systems/turtlebot2.hpp>

TEST(GenericTests, UniformDistribution1)
{
    unsigned int N = 1000;
    UniformDistribution distrib1, distrib2;

    for(auto i = 0; i < N; i++)
        EXPECT_TRUE(distrib1.get() != distrib2.get());
}

TEST(GenericTests, testUniformDistribution2)
{
    double lower = 0.0;
    double upper = 1.0;
    UniformDistribution distrib(lower, upper);

    unsigned int nb = 10;
    double size = (upper-lower)/nb;
    std::map<double, int> results;

    for(auto i = 0; i < nb; ++i)
        results.emplace(i*size, 0);

    double real;
    unsigned int N = 2e7;
    for(auto i = 0; i < N; ++i)
    {
        real = distrib.get();
        for(auto interval = results.begin(); interval != results.end(); ++interval)
        {
            if(Interval(interval->first, interval->first+size).contains(real))
            {
                interval->second += 1;
                break;
            }
        }
    }

    for(auto interval = results.begin(); interval != results.end(); ++interval)
        EXPECT_TRUE(std::abs(interval->second - double(N)/double(nb)) < 1e4);
}

TEST(GenericTests, testCase1)
{
    unsigned int N = pow(pow(2,dynamical_systems::TurtleBot::state_size),1);
    double dt              = 0.1;

    IntervalVector control(dynamical_systems::TurtleBot::control_size);
    control[0] = Interval(0.0, 1.0);
    control[1] = Interval(0.0, 2.0);

    IntervalVector initial_box(dynamical_systems::TurtleBot::state_size);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);
    initial_box[2] = Interval(0.0, 1.0);

    Particle particle(initial_box, 1.0);

    Particles turtlebot_particles_in 
        = particle.subdivise(SUBDIVISION_TYPE::ALL_DIMENSIONS);
    deque<IntervalVector> turtlebot_boxes;

    auto turtlebot = std::shared_ptr<dynamical_systems::TurtleBot>(
            new dynamical_systems::TurtleBot(dt = dt));

    unsigned int i = 0;
    for(auto it = turtlebot_particles_in.begin();
        it != turtlebot_particles_in.end(); ++it, ++i)
    {
        turtlebot_boxes.push_back(turtlebot->applyDynamics(*it, control));
    }

    BoxParticleFilter bpf(N, initial_box, turtlebot);

    bpf.prediction(control);
    Particles particles = bpf.getParticles();

    EXPECT_TRUE(particles.size() == N) << "Wrong particles number";

    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        if(turtlebot_boxes[i] != particles[i])
        {
            ROS_INFO_STREAM(i);
            ROS_INFO_STREAM(turtlebot_boxes[i]);
            ROS_INFO_STREAM(particles[i]);
        }
        EXPECT_TRUE(turtlebot_boxes[i] == particles[i])
                        << "Error in the turtlebot dynamical system";
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "generic_bpf_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
