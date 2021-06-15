#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

TEST(TurtlebotTest, testCase1)
{
    TurtleBotDynamicalModel* dynamical_model = new TurtleBotDynamicalModel();

    IntervalVector initial_box(TurtleBotDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 0.0);
    initial_box[1]= Interval(0.0, 0.0);
    initial_box[2]= Interval(0.0, 0.0);

    IntervalVector control(TurtleBotDynamicalModel::control_size);
    control[0] = Interval(0.0, 0.1);
    control[1] = Interval(0.0, 0.1);

    IntervalVector box = dynamical_model->applyDynamics(initial_box, control);

    ROS_INFO_STREAM(box);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtlebot_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
