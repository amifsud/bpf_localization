#define LOG_LEVEL Info

#include <gtest/gtest.h>
#include "bpf_localization/boat_bpf_localization.hpp"

TEST(BoatBPFLocalizationTest, testBoatBPFLocalization)
{
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
