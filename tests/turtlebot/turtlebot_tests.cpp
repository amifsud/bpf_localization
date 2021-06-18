#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

TEST(TurtlebotTest, testDynamics1)
{
    double dt              = 0.1;
    double wheels_distance = 23e-2;
    double wheels_radius   = 3.5e-2;
    bool   ivp             = false;

    IntervalVector initial_box(TurtleBotDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 2.0);
    initial_box[1]= Interval(0.0, 3.0);
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
        = TurtleBotDynamicalModel(  dt,
                                    wheels_radius,
                                    wheels_distance,
                                    ivp);

    IntervalVector turtlebot_box = turtlebot.applyDynamics(initial_box, control);

    ROS_INFO_STREAM(function_box);

    EXPECT_TRUE(function_box == turtlebot_box)
                    << "Error in the turtlebot dynamicqal system";

    IntervalVector wo_ivp_result(3);
    wo_ivp_result[0] = Interval(0.0, 2.00525);
    wo_ivp_result[1] = Interval(0.0, 3.00442);
    wo_ivp_result[2] = Interval(-0.0304348, 1.01522);

    EXPECT_TRUE(distance(wo_ivp_result, turtlebot_box) < 1e-5);

}

TEST(TurtlebotTest, testDynamics2)
{
    double dt              = 0.1;
    double wheels_distance = 23e-2;
    double wheels_radius   = 3.5e-2;
    bool   ivp             = true;

    IntervalVector initial_box(TurtleBotDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 2.0);
    initial_box[1]= Interval(0.0, 3.0);
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
        = TurtleBotDynamicalModel(  dt,
                                    wheels_radius,
                                    wheels_distance,
                                    ivp);

    IntervalVector turtlebot_box = turtlebot.applyDynamics(initial_box, control);

    IntervalVector ivp_result(3);
    ivp_result[0] = Interval(-0.00156997, 2.00589);
    ivp_result[1] = Interval(-0.00251862, 3.00486);
    ivp_result[2] = Interval(-0.0304348, 1.01522);

    EXPECT_TRUE(distance(ivp_result, turtlebot_box) < 1e-5);
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
