#define LOG_LEVEL Info

#include <stdio.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include "bpf_localization/interfaces/sensors.hpp"

std::string calibration_data_format[]={ "vector,component,data",
                                        "vector_1,x",
                                        "vector_1,y",
                                        "vector_2,x",
                                        "vector_2,y"};

class SensorInterfaceDerivation: public Interfaces::Sensors::SensorInterface
{
    public:
        SensorInterfaceDerivation(  std::string path, 
                                    std::string name, 
                                    unsigned int decimal):
            SensorInterface(path, name, size, decimal)
        {
        }

    public:
        static const unsigned int size = 4;

    protected:
        void calibrationDataFormat()
        {
            calibration_data_format_
                = std::vector<std::string>( calibration_data_format,
                                            calibration_data_format+size+1);
        }

    public:
        void setBuffer(std::deque<IntervalVector> buffer){ buffer_ = buffer; }
        IntervalVector publicIntervalFromVector(const Vector& data)
        {
            return intervalFromVector(data);
        }
};

TEST(SensorsTests, SensorsTest1)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    SensorInterfaceDerivation sensor(path, "sensor", 3);

    std::deque<IntervalVector> buffer, buffer1;
    for(auto i = 0; i < 10; ++i)
        buffer.push_back(IntervalVector(i, i+1));

    sensor.setBuffer(buffer);

    buffer1 = sensor.getIntervalData();
    EXPECT_TRUE(buffer == buffer1);

    buffer1 = sensor.getIntervalData();
    EXPECT_TRUE(buffer1 == std::deque<IntervalVector>());

    std::remove(&path[0]);
}

TEST(SensorsTests, SensorsTest2)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    SensorInterfaceDerivation sensor(path, "sensor", 3);

    std::deque<IntervalVector> buffer, buffer1;
    for(auto i = 0; i < 10; ++i)
        buffer.push_back(IntervalVector(i, i+1));

    sensor.setBuffer(buffer);

    IntervalVector data = sensor.getFirstIntervalValue();
    EXPECT_TRUE(data == buffer[0]);

    buffer.pop_front();
    buffer1 = sensor.getIntervalData();
    EXPECT_TRUE(buffer == buffer1);

    std::remove(&path[0]);
}

TEST(SensorsTests, SensorsTest3)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    SensorInterfaceDerivation sensor(path, "sensor", 3);

    std::vector<std::string> lines;
    lines.push_back("vector,component,data,");
    for(auto u = 1; u < SensorInterfaceDerivation::size+1; ++u)
    {
        lines.push_back(calibration_data_format[u] + ",lb,1.0,2.0");
        lines.push_back(calibration_data_format[u] + ",ub,3.0,4.0");
        lines.push_back(calibration_data_format[u] + ",mid,2.0,2.0");
        lines.push_back(calibration_data_format[u] + ",mean,2.0,2.0");
    }

    File file(path);
    file.write(&lines);

    Vector data(4, 2.);
    sensor.publicIntervalFromVector(data);

    std::remove(&path[0]);
}


TEST(SensorsTests, SensorsTest4)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    SensorInterfaceDerivation sensor(path, "sensor", 3);

    std::vector<std::string> lines;
    lines.push_back("vector,component,data,");
    for(auto u = 1; u < SensorInterfaceDerivation::size+1; ++u)
    {
        lines.push_back(calibration_data_format[u] + ",lb,1.0,2.0");
        lines.push_back(calibration_data_format[u] + ",ub,3.0,4.0");
        lines.push_back(calibration_data_format[u] + ",mid,2.0,2.0");
        lines.push_back(calibration_data_format[u] + ",mean,2.0,2.0");
    }

    File file(path);
    file.write(&lines);

    

    std::remove(&path[0]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "sensors_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
