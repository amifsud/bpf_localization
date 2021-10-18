#define LOG_LEVEL Info

#include <gtest/gtest.h>

#include "bpf_localization/sensors.hpp"

TEST(IMU_standalone, basicTest)
{
    ros::NodeHandle nh;

    auto imu = IMUInterface(&nh, "boat_imu", 3);

    ros::Rate r(1./5);

    std::deque<IntervalVector> data;

    r.sleep();

    data = imu.getIntervalData();
    ROS_INFO_STREAM("SIZE = " << data.size());
    
    ros::spinOnce();

    r.sleep();

    data = imu.getIntervalData();
    ROS_INFO_STREAM("SIZE = " << data.size());
    for(auto it = data.begin(); it != data.end(); it++)
        ROS_INFO_STREAM(*it);
    ros::spinOnce();
  
    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "imu_standalone_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
