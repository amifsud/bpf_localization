#define LOG_LEVEL Info

#include <gtest/gtest.h>
#include <bpf_localization/box_particle_filter.hpp>

using namespace bpf;

class BPFTest : public BoxParticleFilter
{
    public:
        BPFTest(unsigned int N, Particles& particles):
            BoxParticleFilter(N, particles)
        {
        }

        BPFTest(unsigned int N, IntervalVector& box):
            BoxParticleFilter(N, box)
        {
        }

        std::vector<unsigned int> publicChooseSubdivisions(
                std::vector<double> uis = std::vector<double>(0))
        {
            return chooseSubdivisions(uis);
        }

        unsigned int publicGetDirection(IntervalVector& box)
        {
            return getDirection(box);
        }
};

TEST(ResamplingMultinomialTests, Test1)
{
    unsigned int N  = 4e2;

    IntervalVector initial_box(2);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);

    Particles particles;
    for(auto i = 0; i < N; i++)
        particles.push_back(Particle(initial_box, i+1));
    particles.weigthsNormalization();

    unsigned int N1 = 1./particles[0].weight()-N;
    std::vector<double> uis;
    for(auto i = 0; i < N+N1; ++i)
        uis.push_back((i+1)*particles[0].weight());

    BPFTest bpf(N+N1, particles);
    std::vector<unsigned int> subdivisions = bpf.publicChooseSubdivisions(uis);

    for(auto i = 1; i < subdivisions.size(); ++i)
        EXPECT_TRUE(subdivisions[i] == subdivisions[i-1] + 1);
}

// This test may be false because it depend on random things.
// The previous test and the UniformDistribution test replace it

/*TEST(ResamplingMultinomialTests, Test2)
{
    unsigned int N  = 100;
    unsigned int N1 = 1e7;

    IntervalVector initial_box(2);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);

    Particles particles;
    for(auto i = 0; i < N; i++)
        particles.push_back(Particle(initial_box, i));

    particles.weigthsNormalization();
    BPFTest bpf(N+N1, particles);

    std::vector<unsigned int> subdivisions = bpf.publicChooseSubdivisions();

    for(auto i = 1; i < subdivisions.size(); ++i)
    {
        //ROS_INFO_STREAM(subdivisions[i]);
        EXPECT_TRUE(subdivisions[i] > subdivisions[i-1]);
    }
}*/

TEST(ResamplingMultinomialTests, Test3)
{
    unsigned int N = 1e5;

    IntervalVector box(3);
    box[0] = Interval(0.0, 1.0);
    box[1] = Interval(0.0, 1.0);
    box[2] = Interval(0.0, 1.0);

    BPFTest bpf(N, box);

    std::vector<unsigned int> directions(3, 0.);
    for(auto i = 0; i < N; i++)
        directions[bpf.publicGetDirection(box)] += 1;

    for(auto i = 1; i < directions.size(); ++i)
        EXPECT_TRUE(int(directions[i]/double(N)*1e2) == int(directions[i-1]/double(N)*1e2));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "multinomial_resampling_bpf_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
