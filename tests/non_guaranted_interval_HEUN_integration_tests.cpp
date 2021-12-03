#define LOG_LEVEL Info
#define INTEGRATION_METHOD  1

#include <gtest/gtest.h>
#include "bpf_localization/dynamical_systems.hpp"

TEST(GuarantedIntegrationTest, testDynamics1)
{
    double dt              = 0.1;

    IntervalVector initial_box(DoubleIntegratorDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 0.0);
    initial_box[1]= Interval(0.0, 0.0);

    IntervalVector control(DoubleIntegratorDynamicalModel::control_size);
    control[0] = Interval(1.0, 1.0);

    DoubleIntegratorDynamicalModel double_integrator(dt);

    IntervalVector double_integrator_box 
        = double_integrator.applyDynamics(initial_box, control);

    IntervalVector result(DoubleIntegratorDynamicalModel::state_size);
    result[0] = Interval(0.005, 0.005);
    result[1] = Interval(0.1, 0.1);

    EXPECT_TRUE(eps_equals(double_integrator_box, result)) << "Integration error";
}

TEST(GuarantedIntegrationTest, testDynamics2)
{
    double dt              = 0.1;

    IntervalVector initial_box(DoubleIntegratorDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 0.0);
    initial_box[1]= Interval(0.0, 0.0);

    IntervalVector control(DoubleIntegratorDynamicalModel::control_size);
    control[0] = Interval(1.0, 2.0);

    DoubleIntegratorDynamicalModel double_integrator(dt);

    IntervalVector double_integrator_box 
        = double_integrator.applyDynamics(initial_box, control);

    IntervalVector result(DoubleIntegratorDynamicalModel::state_size);
    result[0] = Interval(0.005, 0.01);
    result[1] = Interval(0.1, 0.2);

    EXPECT_TRUE(eps_equals(double_integrator_box, result)) << "Integration error";
}

TEST(GuarantedIntegrationTest, testDynamics3)
{
    double dt              = 0.1;

    IntervalVector initial_box(DoubleIntegratorDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 1.0);
    initial_box[1]= Interval(0.0, 0.0);

    IntervalVector control(DoubleIntegratorDynamicalModel::control_size);
    control[0] = Interval(1.0, 1.0);

    DoubleIntegratorDynamicalModel double_integrator(dt);

    IntervalVector double_integrator_box 
        = double_integrator.applyDynamics(initial_box, control);

    IntervalVector result(DoubleIntegratorDynamicalModel::state_size);
    result[0] = Interval(0.005, 1.005);
    result[1] = Interval(0.1, 0.1);

    EXPECT_TRUE(eps_equals(double_integrator_box, result)) << "Integration error";
}

TEST(GuarantedIntegrationTest, testDynamics4)
{
    double dt              = 0.1;

    IntervalVector initial_box(DoubleIntegratorDynamicalModel::state_size);
    initial_box[0]= Interval(0.0, 0.0);
    initial_box[1]= Interval(0.0, 1.0);

    IntervalVector control(DoubleIntegratorDynamicalModel::control_size);
    control[0] = Interval(1.0, 1.0);

    DoubleIntegratorDynamicalModel double_integrator(dt);

    IntervalVector double_integrator_box 
        = double_integrator.applyDynamics(initial_box, control);

    IntervalVector result(DoubleIntegratorDynamicalModel::state_size);
    result[0] = Interval(0.005, 0.105);
    result[1] = Interval(0.1, 1.1);

    EXPECT_TRUE(eps_equals(double_integrator_box, result)) << "Integration error";
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
