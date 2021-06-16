#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

TEST(TurtlebotTest, testDynamics)
{
    double dt              = 0.1;
    double wheels_distance = 23e-2;
    double wheels_radius   = 3.5e-2;

    IntervalVector initial_box(TurtleBotDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 0.0);
    initial_box[1]= Interval(0.0, 0.0);
    initial_box[2]= Interval(0.0, 1.0); //if we let zero we don't test y dynamics

    IntervalVector control(TurtleBotDynamicalModel::control_size);
    control[0] = Interval(0.0, 1.0);
    control[1] = Interval(0.0, 2.0);

    Variable state(TurtleBotDynamicalModel::state_size);
    Function dynamical_model             
        = Function(state, 
                Return(dt*wheels_radius/2*(control[0]+control[1])*cos(state[2])+state[0],
                       dt*wheels_radius/2*(control[0]+control[1])*sin(state[2])+state[1],
                       dt*wheels_radius/wheels_distance*(control[0]-control[1])+state[2]
                       )
                    );

    IntervalVector function_box = dynamical_model.eval_vector(initial_box);

    TurtleBotDynamicalModel turtlebot
        = TurtleBotDynamicalModel(  dt              = dt,
                                    wheels_radius   = wheels_radius,
                                    wheels_distance = wheels_distance);

    IntervalVector turtlebot_box = turtlebot.applyDynamics(initial_box, control);

    EXPECT_TRUE(function_box == turtlebot_box)
                    << "Error in the turtlebot dynamicqal system";

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
