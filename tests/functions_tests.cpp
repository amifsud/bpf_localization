#define LOG_LEVEL Info

#include <gtest/gtest.h>
#include "bpf_localization/utils.hpp"

TEST(FunctionsTest, testFunctions)
{
    IntervalMatrix m(3,2);
    m[0][0] = 1.;
    m[0][1] = 2.;
    m[1][0] = 3.;
    m[1][1] = 4.;
    m[2][0] = 5.;
    m[2][1] = 6.;

    MatrixFunction matrix(m);

    SubVariable x(5, 2, 2);
    SubVariable* px = &x;

    IntervalVector x0(5);
    x0[0] = 1.;
    x0[1] = 2.;
    x0[2] = 1.;
    x0[3] = 2.;
    x0[4] = 5.;

    Function function(*px, matrix*px);
    IntervalVector x1 = function.eval_vector(x0);

    IntervalVector x1_result(3);
    x1_result[0] = Interval(5., 5.);
    x1_result[1] = Interval(11., 11.);
    x1_result[2] = Interval(17., 17.);

    EXPECT_TRUE(x1 == x1_result) << "wrong matrix multiplication";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "functions_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
