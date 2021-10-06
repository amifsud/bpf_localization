#define LOG_LEVEL Info

#include <gtest/gtest.h>
#include "bpf_localization/boat_bpf_localization.hpp"

TEST(BoatBPFLocalizationTest, testBoatBPFLocalization)
{
    IntervalVector initial_box(IMUDynamicalModel::state_size);
    initial_box[0] = Interval(-2., 2.);
    initial_box[1] = Interval(-2., 2.);
    initial_box[2] = Interval(-2., 2.);
    initial_box[3] = Interval(-2., 2.);
    initial_box[4] = Interval(-2., 2.);
    initial_box[5] = Interval(-2., 2.);
    initial_box[6] = Interval(-2., 2.);
    initial_box[7] = Interval(-2., 2.);
    initial_box[8] = Interval(-2., 2.);
    initial_box[9] = Interval(-2., 2.);

    BoatBPFLocalization* boat = new BoatBPFLocalization(initial_box, 2);
 
    IntervalVector imu(IMUDynamicalModel::control_size);
    imu[0] = Interval(0., 0.);     // gyrometer
    imu[1] = Interval(0., 0.);
    imu[2] = Interval(0., 0.);
    imu[3] = Interval(0., 0.);     // accelerometer
    imu[4] = Interval(0., 0.);
    imu[5] = Interval(-100.81, -100.81);

    boat->IMUCallback(imu);
 
    IntervalVector gps(IMUDynamicalModel::measures_size);
    gps[0] = Interval(-0.2, 0.2); // gps
    gps[1] = Interval(-0.2, 0.2);
    gps[2] = Interval(-0.2, 0.2);

    boat->GPSCallback(gps);
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
