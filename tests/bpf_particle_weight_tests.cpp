#define LOG_LEVEL Info

#include <gtest/gtest.h>

#include "bpf_localization/utils.hpp"
#include "bpf_localization/particles.hpp"

// Declare a test
TEST(WeightTests, testCaseGetSet)
{
    IntervalVector initial_box(3);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);
    initial_box[2] = Interval(0.0, 1.0);

    bpf::Particle particle(initial_box, 1.0);

    EXPECT_TRUE(particle.weight() == 1.0) << "Weight initialization failed";

    particle.updateWeight(0.7);

    EXPECT_TRUE(particle.weight() == 0.7) << "Weight update failed";
}

TEST(WeightTests, testSumOfWeight)
{
    IntervalVector initial_box(3);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);
    initial_box[2] = Interval(0.0, 1.0);

    bpf::Particle particle(initial_box, 1.0);

    unsigned int N = 100;
    bpf::Particles particles = particle.subdivise(bpf::SUBDIVISION_TYPE::RANDOM, N);

    float sum = 0;
    for(auto it = particles.begin(); it != particles.end(); it++)
        sum += it->weight();

    EXPECT_TRUE(std::abs(particles.sumOfWeights() - 1.0) < 1e-7);

    particle = bpf::Particle(initial_box, 2.0);

    particles = particle.subdivise(bpf::SUBDIVISION_TYPE::RANDOM, N);

    sum = 0;
    for(auto it = particles.begin(); it != particles.end(); it++)
        sum += it->weight();

    EXPECT_TRUE(std::abs(particles.sumOfWeights() - 2.0) < 1e-7);
}

TEST(WeightTests, testResetUniformly)
{
    IntervalVector initial_box(3);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);
    initial_box[2] = Interval(0.0, 1.0);

    bpf::Particle particle(initial_box, 1.0);

    unsigned int N = 100;
    bpf::Particles particles = particle.subdivise(bpf::SUBDIVISION_TYPE::RANDOM, N);

    particles.resetWeightsUniformly(4.5);

    for(auto it = particles.begin(); it != particles.end(); it++)
        EXPECT_TRUE(std::abs(it->weight() - 4.5/N) < 1e-7);

    EXPECT_TRUE(std::abs(particles.sumOfWeights() - 4.5));
}

TEST(WeightTests, testNormalization)
{
    IntervalVector initial_box(3);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);
    initial_box[2] = Interval(0.0, 1.0);

    bpf::Particle particle(initial_box, 2.0); // Sum initialization to 2.0

    unsigned int N = pow(2,7); // = 128
    unsigned int offset = 50; // has to be < N
    bpf::Particles particles = particle.subdivise(bpf::SUBDIVISION_TYPE::RANDOM, N+offset, 0);

    particles.weigthsNormalization(); // Sum should be 1.0 now

    EXPECT_TRUE(std::abs(particles.sumOfWeights() - 1.) < 1e-7);

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

TEST(WeightTests, testCumulativeWeights)
{
    bpf::Particles particles;

    IntervalVector box(3);
    box[0] = Interval(0., 1.);
    box[1] = Interval(1., 2.);
    box[2] = Interval(2., 3.);

    for(auto i = 0; i < 100; i++)
        particles.push_back(bpf::Particle(box, i));

    std::vector<float> cumulative_weights = particles.getCumulativeWeights();

    for(auto i = 1; i < cumulative_weights.size(); ++i)
        EXPECT_TRUE(cumulative_weights[i]-cumulative_weights[i-1] == i);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "particle_weight_bpf_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
