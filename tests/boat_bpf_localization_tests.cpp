#define LOG_LEVEL Info

#include <gtest/gtest.h>
#include "bpf_localization/boat_bpf_localization.hpp"

TEST(BoatBPFLocalizationTest, testBoatBPFLocalization)
{
    double pos = 1.;
    double vel = 0.1;
    double theta = 10./180.*3.14;

    IntervalVector imu(IMUDynamicalModel::control_size);
    imu[0] = Interval(0., 0.); // gyrometer
    imu[1] = Interval(0., 0.);
    imu[2] = Interval(0., 0.);
    imu[3] = Interval(0., 0.); // accelerometer
    imu[4] = Interval(0., 0.);
    imu[5] = Interval(-100.81, -100.81);

    auto boat = std::shared_ptr<BoatBPFLocalization>(
            new BoatBPFLocalization(pos, vel, theta, false));
    Particles initial_particles = Particles(boat->getParticles());
 
    boat->IMUCallback(imu);

    Particles mono_threaded_particles 
        = Particles(boat->getParticles(BOXES_TYPE::PREDICTION));
 
    boat = std::shared_ptr<BoatBPFLocalization>(
            new BoatBPFLocalization(initial_particles, true));
 
    boat->IMUCallback(imu);
 
    Particles multi_threaded_particles 
        = Particles(boat->getParticles(BOXES_TYPE::PREDICTION));

    bool equal_particles 
        = compareParticles(&mono_threaded_particles, &multi_threaded_particles);


    EXPECT_TRUE(equal_particles)
        << "Multi-threaded and no multi-threaded predicted particles not the same";

    /*auto particles = boat->getParticles(BOXES_TYPE::PREDICTION);
    for(auto particle = particles.begin(); particle != particles.end(); particle++)
        std::cout << *particle << std::endl;;*/
}

TEST(BoatBPFLocalizationTest, testBoatBPFLocalization1)
{
    IntervalVector gps(IMUDynamicalModel::measures_size);
    gps[0] = Interval(-0.2, 0.2); // gps
    gps[1] = Interval(-0.2, 0.2);
    gps[2] = Interval(-0.2, 0.2);

    //boat->GPSCallback(gps);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "boat_bpf_localization_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
