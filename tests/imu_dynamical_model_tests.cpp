#define LOG_LEVEL Info
#define INIT_METHOD     1

#include <gtest/gtest.h>
#include <bpf_localization/tests.hpp>

TEST(IMUDynamicalModelTest, testDynamics1)
{
    Variable  quaternion(4);
    Function t2 = Function(quaternion, quaternion[3]*quaternion[0]);
    Function t3 = Function(quaternion, quaternion[3]*quaternion[1]);
    Function t4 = Function(quaternion, quaternion[3]*quaternion[2]);
    Function t5 = Function(quaternion, -quaternion[0]*quaternion[0]);
    Function t6 = Function(quaternion, quaternion[0]*quaternion[1]);
    Function t7 = Function(quaternion, quaternion[0]*quaternion[2]);
    Function t8 = Function(quaternion, -quaternion[1]*quaternion[1]);
    Function t9 = Function(quaternion, quaternion[1]*quaternion[2]);
    Function t10 = Function(quaternion, -quaternion[2]*quaternion[2]);

    Variable control(3);
    Function rotate(quaternion, control, Return( 2*( (t8(quaternion) + t10(quaternion))
                                                *control[0] 
                                          + (t6(quaternion) -  t4(quaternion))
                                                *control[1] 
                                          + (t3(quaternion) + t7(quaternion))
                                                *control[2] ) + control[0],
                                        2*( (t4(quaternion) +  t6(quaternion))
                                                *control[0] 
                                          + (t5(quaternion) + t10(quaternion))
                                                *control[1] 
                                          + (t9(quaternion) - t2(quaternion))
                                                *control[2] ) + control[1],
                                        2*( (t7(quaternion) -  t3(quaternion))
                                                *control[0] 
                                          + (t2(quaternion) +  t9(quaternion))
                                                *control[1] 
                                          + (t5(quaternion) + t8(quaternion))
                                                *control[2] ) + control[2]));

    Variable stat(4);
    Variable control1(6);
    Function select_quaternion(stat, Return(stat[0], stat[1], stat[2], stat[3]));
    Function select_gyro(control1, Return(control1[0], control1[1], control1[2]));
    Function select_accelero(control1, Return(control1[3], control1[4], control1[5]));

    Variable stat1(4);
    Variable control2(6);
    Function rotated_accelero(stat1, control2, rotate(select_quaternion(stat1), select_accelero(control2)));
    Function rotated_gyro    (stat1, control2, rotate(select_quaternion(stat1), select_gyro(control2)));

    Vector quat(4);
    quat[0] = 1;
    quat[1] = 2;
    quat[2] = 3;
    quat[3] = 4;

    Vector cont(6);
    cont[0] = 1;
    cont[1] = 4;
    cont[2] = 3;
    cont[3] = 4;
    cont[4] = 5;
    cont[5] = 6;

    IntervalVector state(1);
    IntervalVector state1(3);
    state = t2.eval_vector(quat);
    ROS_INFO_STREAM(state);
    state = t3.eval_vector(quat);
    ROS_INFO_STREAM(state);

    Variable stat2(4);
    Function set_control_accelero(stat2, rotated_accelero(stat2, cont));
    Function set_control_gyroo(stat2, rotated_gyro(stat2, cont));

    state1 = set_control_accelero.eval_vector(quat);
    ROS_INFO_STREAM(state1);

    IMUDynamicalModel imu = IMUDynamicalModel();
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
