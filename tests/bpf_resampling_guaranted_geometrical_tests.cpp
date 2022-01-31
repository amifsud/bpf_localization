#define LOG_LEVEL Info

#define RESAMPLING_METHOD       1
#define RESAMPLING_DIRECTION    1

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

        BPFTest(unsigned int N, IntervalVector& box, 
                std::shared_ptr<dynamical_systems::DynamicalSystem> dynamical_systems):
            BoxParticleFilter(N, box, dynamical_systems)
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

TEST(ResamplingGuarantedGeometricalTests, Test1)
{
    unsigned int N  = 1e2;

    IntervalVector initial_box(2);
    initial_box[0] = Interval(0.0, 2.0);
    initial_box[1] = Interval(0.0, 3.0);

    Particles particles;
    for(auto i = 0; i < N; i++)
        particles.push_back(Particle(initial_box, i+1));
    particles.weigthsNormalization();

    double min_size = particles[0].weight();
    unsigned int N1 = 1./min_size;;

    for(auto i = 0; i < N1; i++)
        particles.push_front(Particle(initial_box, 0.0));

    std::vector<double> uis;
    for(auto i = 0; i < N1; ++i) uis.push_back((i+1)*min_size);

    BPFTest bpf(N+N1, particles);
    std::vector<unsigned int> subdivisions = bpf.publicChooseSubdivisions(uis);

    for(auto i = 0; i < subdivisions.size(); ++i)
    {
        EXPECT_TRUE(subdivisions[i] == 0 | 
                    subdivisions[i] == 2 |
                    subdivisions[i] == subdivisions[i-1] + 1);
    }
}

TEST(ResamplingGuarantedGeometricalTests, Test2)
{
    unsigned int N = 1e5;

    IntervalVector box(2);
    box[0] = Interval(0.0, 1.0);
    box[1] = Interval(0.0, 16.0);

    std::shared_ptr<dynamical_systems::DoubleIntegrator> 
        double_integrator(new dynamical_systems::DoubleIntegrator(0.1));

    BPFTest bpf(N, box, double_integrator);
    
    unsigned int direction;
    for(auto i = 0; i < 3; i++)
    {
        direction = bpf.publicGetDirection(box);
        EXPECT_TRUE(direction == 1);

        box[1] = Interval(box[1].lb(), box[1].ub()/2.);
    }

    direction = bpf.publicGetDirection(box);
    EXPECT_TRUE(direction == 0);

    box[1] = Interval(box[1].lb(), box[1].ub()+1e-4);

    direction = bpf.publicGetDirection(box);
    EXPECT_TRUE(direction == 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "guaranted_geometrical_resampling_bpf_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
