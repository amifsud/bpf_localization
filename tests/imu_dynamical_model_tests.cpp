#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <gtest/gtest.h>
#include <bpf_localization/utils.hpp>
#include <bpf_localization/tests.hpp>

TEST(IMUDynamicalModelTest, testDynamicsTranslationZ)
{
    double dt = 1e-0;
    IMUDynamicalModel imu(dt = dt);
    IntervalVector state1(IMUDynamicalModel::state_size);
    IntervalVector state_0(IMUDynamicalModel::state_size);
    IntervalVector state_k(IMUDynamicalModel::state_size);
    IntervalVector control(IMUDynamicalModel::control_size);

    // Linear acceleration Z axis
    control[0] = Interval(0., 0.); // gyrometer
    control[1] = Interval(0., 0.);
    control[2] = Interval(0., 0.);
    control[3] = Interval(0., 0.); // accelerometer
    control[4] = Interval(0., 0.);
    control[5] = Interval(-100.81, -100.81);

    state_0[0] = Interval(0., 0.); // quaternion
    state_0[1] = Interval(0., 0.);
    state_0[2] = Interval(0., 0.);
    state_0[3] = Interval(1., 1.);
    state_0[4] = Interval(0., 0.); // position
    state_0[5] = Interval(0., 0.);
    state_0[6] = Interval(0., 0.);
    state_0[7] = Interval(0., 0.); // linear velocity
    state_0[8] = Interval(0., 0.);
    state_0[9] = Interval(0., 0.);

    state_k = imu.applyDynamics(state_0, control);
    state1[0] = Interval(0., 0.);
    state1[1] = Interval(0., 0.);
    state1[2] = Interval(0., 0.);
    state1[3] = Interval(1., 1.);
    state1[4] = Interval(0., 0.);
    state1[5] = Interval(0., 0.);
    state1[6] = Interval(-45.5, -45.5);
    state1[7] = Interval(0., 0.);
    state1[8] = Interval(0., 0.);
    state1[9] = Interval(-91., -91.);
    EXPECT_TRUE(eps_equals(state_k, state1)) << "Error in IMU model";
}

TEST(IMUDynamicalModelTest, testDynamicsTranslationY)
{
    double dt = 1e-0;
    IMUDynamicalModel imu(dt = dt);
    IntervalVector state1(IMUDynamicalModel::state_size);
    IntervalVector state_0(IMUDynamicalModel::state_size);
    IntervalVector state_k(IMUDynamicalModel::state_size);
    IntervalVector control(IMUDynamicalModel::control_size);

    // Linear acceleration Y axis
    control[0] = Interval(0., 0.); // gyrometer
    control[1] = Interval(0., 0.);
    control[2] = Interval(0., 0.);
    control[3] = Interval(0., 0.); // accelerometer
    control[4] = Interval(-100.81, -100.81);
    control[5] = Interval(-9.81, -9.81);

    state_0[0] = Interval(0., 0.); // quaternion
    state_0[1] = Interval(0., 0.);
    state_0[2] = Interval(0., 0.);
    state_0[3] = Interval(1., 1.);
    state_0[4] = Interval(0., 0.); // position
    state_0[5] = Interval(0., 0.);
    state_0[6] = Interval(0., 0.);
    state_0[7] = Interval(0., 0.); // linear velocity
    state_0[8] = Interval(0., 0.);
    state_0[9] = Interval(0., 0.);

    state_k = imu.applyDynamics(state_0, control);
    state1[0] = Interval(0., 0.);
    state1[1] = Interval(0., 0.);
    state1[2] = Interval(0., 0.);
    state1[3] = Interval(1., 1.);
    state1[4] = Interval(0., 0.);
    state1[5] = Interval(50.405, 50.405);
    state1[6] = Interval(0., 0.);
    state1[7] = Interval(0., 0.);
    state1[8] = Interval(100.81, 100.81);
    state1[9] = Interval(0., 0.);
    EXPECT_TRUE(eps_equals(state_k, state1)) << "Error in IMU model";
}

TEST(IMUDynamicalModelTest, testDynamicsTranslationX)
{
    double dt = 1e-0;
    IMUDynamicalModel imu(dt = dt);
    IntervalVector state1(IMUDynamicalModel::state_size);
    IntervalVector state_0(IMUDynamicalModel::state_size);
    IntervalVector state_k(IMUDynamicalModel::state_size);
    IntervalVector control(IMUDynamicalModel::control_size);

    // Linear acceleration X axis
    control[0] = Interval(0., 0.); // gyrometer
    control[1] = Interval(0., 0.);
    control[2] = Interval(0., 0.);
    control[3] = Interval(-100.81, -100.81); // accelerometer
    control[4] = Interval(0., 0.);
    control[5] = Interval(-9.81, -9.81);

    state_0[0] = Interval(0., 0.); // quaternion
    state_0[1] = Interval(0., 0.);
    state_0[2] = Interval(0., 0.);
    state_0[3] = Interval(1., 1.);
    state_0[4] = Interval(0., 0.); // position
    state_0[5] = Interval(0., 0.);
    state_0[6] = Interval(0., 0.);
    state_0[7] = Interval(0., 0.); // linear velocity
    state_0[8] = Interval(0., 0.);
    state_0[9] = Interval(0., 0.);

    state_k = imu.applyDynamics(state_0, control);
    state1[0] = Interval(0., 0.);
    state1[1] = Interval(0., 0.);
    state1[2] = Interval(0., 0.);
    state1[3] = Interval(1., 1.);
    state1[4] = Interval(50.405, 50.405);
    state1[5] = Interval(0., 0.);
    state1[6] = Interval(0., 0.);
    state1[7] = Interval(100.81, 100.81);
    state1[8] = Interval(0., 0.);
    state1[9] = Interval(0., 0.);
    EXPECT_TRUE(eps_equals(state_k, state1)) << "Error in IMU model";
}

TEST(IMUDynamicalModelTest, testDynamicsRotationZ)
{
    double dt = 1e-3;
    IMUDynamicalModel imu(dt);
    IntervalVector state1(IMUDynamicalModel::state_size);
    IntervalVector state_0(IMUDynamicalModel::state_size);
    IntervalVector state_k(IMUDynamicalModel::state_size);
    IntervalVector control(IMUDynamicalModel::control_size);

    // Linear acceleration Z axis
    control[0] = Interval(0., 0.); // gyrometer
    control[1] = Interval(0., 0.);
    control[2] = Interval(10., 10.);
    control[3] = Interval(0., 0.); // accelerometer
    control[4] = Interval(0., 0.);
    control[5] = Interval(-9.81, -9.81);

    state_0[0] = Interval(1., 1.); // quaternion
    state_0[1] = Interval(0., 0.);
    state_0[2] = Interval(0., 0.);
    state_0[3] = Interval(1., 1.);
    state_0[4] = Interval(0., 0.); // position
    state_0[5] = Interval(0., 0.);
    state_0[6] = Interval(0., 0.);
    state_0[7] = Interval(0., 0.); // linear velocity
    state_0[8] = Interval(0., 0.);
    state_0[9] = Interval(0., 0.);

    state_k = imu.applyDynamics(state_0, control);
    state1[0] = Interval(0.994988, 0.994988);
    state1[1] = Interval(0., 0.);
    state1[2] = Interval(0., 0.);
    state1[3] = Interval(1.00499, 1.00499);
    state1[4] = Interval(0., 0.);
    state1[5] = Interval(0., 0.);
    state1[6] = Interval(0., 0.);
    state1[7] = Interval(0., 0.);
    state1[8] = Interval(0., 0.);
    state1[9] = Interval(0., 0.);
    EXPECT_TRUE(eps_equals(state_k, state1, 1e-4)) << "Error in IMU model";

}

TEST(IMUDynamicalModelTest, testDynamicsRotationY)
{
    double dt = 1e-3;
    IMUDynamicalModel imu(dt);
    IntervalVector state1(IMUDynamicalModel::state_size);
    IntervalVector state_0(IMUDynamicalModel::state_size);
    IntervalVector state_k(IMUDynamicalModel::state_size);
    IntervalVector control(IMUDynamicalModel::control_size);


    // Linear acceleration Y axis
    control[0] = Interval(0., 0.); // gyrometer
    control[1] = Interval(10., 10.);
    control[2] = Interval(0., 0.);
    control[3] = Interval(0., 0.); // accelerometer
    control[4] = Interval(0., 0.);
    control[5] = Interval(-9.81, -9.81);

    state_0[0] = Interval(1., 1.); // quaternion
    state_0[1] = Interval(0., 0.);
    state_0[2] = Interval(0., 0.);
    state_0[3] = Interval(0., 0.);
    state_0[4] = Interval(0., 0.); // position
    state_0[5] = Interval(0., 0.);
    state_0[6] = Interval(0., 0.);
    state_0[7] = Interval(0., 0.); // linear velocity
    state_0[8] = Interval(0., 0.);
    state_0[9] = Interval(0., 0.);

    state_k = imu.applyDynamics(state_0, control);
    state1[0] = Interval(0.999988, 0.999988);
    state1[1] = Interval(0., 0.);
    state1[2] = Interval(0.00499998, 0.00499998);
    state1[3] = Interval(0., 0.);
    state1[4] = Interval(0., 0.);
    state1[5] = Interval(0., 0.);
    state1[6] = Interval(0., 0.);
    state1[7] = Interval(0., 0.);
    state1[8] = Interval(0., 0.);
    state1[9] = Interval(0., 0.);
    EXPECT_TRUE(eps_equals(state_k, state1, 1e-4)) << "Error in IMU model";
}

TEST(IMUDynamicalModelTest, testDynamicsRotationX)
{
    double dt = 1e-3;
    IMUDynamicalModel imu(dt);
    IntervalVector state1(IMUDynamicalModel::state_size);
    IntervalVector state_0(IMUDynamicalModel::state_size);
    IntervalVector state_k(IMUDynamicalModel::state_size);
    IntervalVector control(IMUDynamicalModel::control_size);

    // Linear acceleration X axis
    control[0] = Interval(-10., -10.); // gyrometer
    control[1] = Interval(0., 0.);
    control[2] = Interval(0., 0.);
    control[3] = Interval(0., 0.); // accelerometer
    control[4] = Interval(0., 0.);
    control[5] = Interval(-9.81, -9.81);

    state_0[0] = Interval(1., 1.); // quaternion
    state_0[1] = Interval(0., 0.);
    state_0[2] = Interval(0., 0.);
    state_0[3] = Interval(0., 0.);
    state_0[4] = Interval(0., 0.); // position
    state_0[5] = Interval(0., 0.);
    state_0[6] = Interval(0., 0.);
    state_0[7] = Interval(0., 0.); // linear velocity
    state_0[8] = Interval(0., 0.);
    state_0[9] = Interval(0., 0.);

    state_k = imu.applyDynamics(state_0, control);
    state1[0] = Interval(0.999988, 0.999988);
    state1[1] = Interval(-0.00499998, -0.00499998);
    state1[2] = Interval(0., 0.);
    state1[3] = Interval(0., 0.);
    state1[4] = Interval(0., 0.);
    state1[5] = Interval(0., 0.);
    state1[6] = Interval(0., 0.);
    state1[7] = Interval(0., 0.);
    state1[8] = Interval(0., 0.);
    state1[9] = Interval(0., 0.);
    EXPECT_TRUE(eps_equals(state_k, state1, 1e-4)) << "Error in IMU model";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "imu_dynamical_model_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
