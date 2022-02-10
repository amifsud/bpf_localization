#define LOG_LEVEL Info

#include <stdio.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include "bpf_localization/interfaces/calibrable.hpp"

std::string calibration_data_format[]={ "vector,component,data",
                                        "vector_1,x",
                                        "vector_1,y",
                                        "vector_2,x",
                                        "vector_2,y"};

class CalibrableDerivation: public Interfaces::Calibrable
{
    public:
        CalibrableDerivation(std::string path, std::string name, unsigned int decimal):
            Calibrable(path, name, size, decimal)
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
        // To access protected data for testing
        std::map<std::string, Vector> getMap(){ return data_map_; }
        void setMap(std::map<std::string, Vector> map){ data_map_ = map; }

        void publicCalibrableDataFormat(){ calibrationDataFormat(); }

        void publicComputeHalfDiameters(){ computeHalfDiameters(); }
        Vector getHalfDiameters(){ return half_diameters_; }

        void setData(std::vector<Vector> data){ data_ = data; }
        std::vector<Vector> getData(){ return data_; }

        void publicStartCalibration(unsigned int init_time, unsigned int until)
        {
            startCalibration(init_time, until);
        }

        std::vector<std::string> getCalibrationDataFormat()
        {
            return calibration_data_format_;
        }

        void publicSplitToDouble(std::string* line, std::string* format,
                                std::vector<double>* values)
        {
            splitToDouble(line, format, values);
        }

        void publicLoadFromFile(){ loadFromFile(); }

        void publicUpdate(){ update(); }

        Vector getSum(){ return sum_; }
        int getNb(){ return nb_; }
        int getInitTime() { return init_time_; }
        int getUntil(){ return until_; }

        void publicWriteCalibrationFile(){ writeToFile(); }
};


TEST(CalibrableTests, CalibrableTest0)
{
    ROS_INFO_STREAM("toto");
    std::string path = ros::package::getPath("bpf_localization");
    ROS_INFO_STREAM("toto");
    path += "/tests/interfaces/test.txt";

    CalibrableDerivation calibrable(path, "calibrable", 3);

    std::map<std::string, Vector> map;
    map.emplace("lb",   Vector(CalibrableDerivation::size, 1e7));
    map.emplace("ub",   Vector(CalibrableDerivation::size, -1e7));
    map.emplace("mid",  Vector(CalibrableDerivation::size, 1e7));
    map.emplace("mean", Vector(CalibrableDerivation::size, 1e7));

    EXPECT_TRUE(map == calibrable.getMap()); 
}

TEST(CalibrableTests, CalibrableTest1)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";

    CalibrableDerivation calibrable(path, "calibrable", 3);
    calibrable.publicCalibrableDataFormat();

    EXPECT_TRUE(
        std::vector<std::string>(   calibration_data_format, 
                                    calibration_data_format+CalibrableDerivation::size+1)
            == calibrable.getCalibrationDataFormat());
}

TEST(CalibrableTests, CalibrableTest2)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";

    std::vector<double> given_values = {10., 11., 12.}; 

    CalibrableDerivation calibrable(path, "calibrable", 3);
    calibrable.publicCalibrableDataFormat();

    std::vector<double> values;
    std::string line = calibration_data_format[1] + ",lb";

    for(double d : given_values) line += "," + std::to_string(d);
    std::string s = calibration_data_format[1]+",lb,";
    calibrable.publicSplitToDouble(&line, &s, &values);

    for(unsigned int i = 0; i < given_values.size(); ++i)
        EXPECT_TRUE(given_values[i] == values[i]);
}

TEST(CalibrableTests, CalibrableTest3)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";

    CalibrableDerivation calibrable(path, "calibrable", 3);

    std::map<std::string, Vector> map, map_out;
    map.emplace("lb",   Vector(CalibrableDerivation::size, 10.));
    map.emplace("ub",   Vector(CalibrableDerivation::size, 20.));
    map.emplace("mid",  Vector(CalibrableDerivation::size, 1e7));
    map.emplace("mean", Vector(CalibrableDerivation::size, 1e7));

    calibrable.setMap(map);
    calibrable.publicComputeHalfDiameters();

    map_out = calibrable.getMap();

    EXPECT_TRUE(map_out.at("mid") == Vector(CalibrableDerivation::size, 15.));

    EXPECT_TRUE(calibrable.getHalfDiameters() == Vector(CalibrableDerivation::size, 10.));
}

TEST(CalibrableTests, CalibrableTest4)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";

    CalibrableDerivation calibrable(path, "calibrable", 3);

    std::vector<Vector> data;
    for(auto i = 0; i < 5; ++i) data.push_back(Vector(2, 1.));

    calibrable.setData(data);

    calibrable.publicStartCalibration(10., 5.);

    EXPECT_TRUE(0. == calibrable.getNb());
    EXPECT_TRUE(Vector(CalibrableDerivation::size, 0.) == calibrable.getSum());
    EXPECT_TRUE(10. == calibrable.getInitTime());
    EXPECT_TRUE(5. == calibrable.getUntil());
    EXPECT_TRUE(std::vector<Vector>() == calibrable.getData());
}

TEST(CalibrableTests, CalibrableTest5)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/";
    std::string file_path = path + "calibrable.csv";
    std::remove(&file_path[0]);

    std::vector<std::string> lines;
    lines.push_back("vector,component,data,");
    for(auto u = 1; u < CalibrableDerivation::size+1; ++u)
    {
        lines.push_back(calibration_data_format[u] + ",lb,1.0,2.0");
        lines.push_back(calibration_data_format[u] + ",ub,3.0,4.0");
        lines.push_back(calibration_data_format[u] + ",mid,2.0,2.0");
        lines.push_back(calibration_data_format[u] + ",mean,2.0,2.0");
    }

    File file(file_path);
    file.write(&lines);

    CalibrableDerivation calibrable(path, "calibrable", 3);
    calibrable.publicLoadFromFile();

    std::map<std::string, Vector> map = calibrable.getMap();

    EXPECT_TRUE(calibrable.getHalfDiameters() == Vector(CalibrableDerivation::size, 3.));
    EXPECT_TRUE(map.at("lb") == Vector(CalibrableDerivation::size, 1.));
    EXPECT_TRUE(map.at("ub") == Vector(CalibrableDerivation::size, 4.));

    std::remove(&file_path[0]);
}

TEST(CalibrableTests, CalibrableTest6)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/";
    std::string file_path = path + "calibrable.csv";
    std::remove(&file_path[0]);

    std::vector<Vector> data;
    for(auto i = 0; i < 5; ++i)
        data.push_back(Vector(CalibrableDerivation::size, i));

    CalibrableDerivation calibrable(path, "calibrable", 3);
    calibrable.setData(data);
    calibrable.publicUpdate();

    std::map<std::string, Vector> map = calibrable.getMap();

    EXPECT_TRUE(eps_equals( calibrable.getHalfDiameters(), 
                        Vector(CalibrableDerivation::size, 4.002)));
    EXPECT_TRUE(eps_equals(map.at("lb"), Vector(CalibrableDerivation::size, -0.001)));
    EXPECT_TRUE(eps_equals(map.at("ub"), Vector(CalibrableDerivation::size, 4.001)));

    std::remove(&file_path[0]);
}

TEST(CalibrableTests, CalibrableTest7)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/";
    std::string file_path = path + "calibrable.csv";
    std::remove(&file_path[0]);

    std::vector<Vector> data;
    for(auto i = 0; i < 5; ++i)
        data.push_back(Vector(CalibrableDerivation::size, i));

    CalibrableDerivation calibrable(path, "calibrable", 3);

    calibrable.setData(data);
    calibrable.publicUpdate();
    calibrable.publicWriteCalibrationFile();

    File file(file_path);
    std::vector<std::string> lines, expected_lines;
    file.read(&lines);

    for(auto i = 1; i < CalibrableDerivation::size+1; ++i)
    {
        expected_lines.push_back(calibration_data_format[i] + ",lb,-0.001000");
        expected_lines.push_back(calibration_data_format[i] + ",mean,2.000000");
        expected_lines.push_back(calibration_data_format[i] + ",mid,2.000000");
        expected_lines.push_back(calibration_data_format[i] + ",ub,4.001000");
    }

    unsigned int i = 0;
    for(auto it = lines.begin()+1; it != lines.end(); ++it, ++i)
        EXPECT_TRUE(expected_lines[i] == *it);

    std::remove(&file_path[0]);
}

TEST(CalibrableTests, CalibrableTest8)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/";
    std::string file_path = path + "calibrable.csv";
    std::remove(&file_path[0]);

    std::vector<Vector> data;
    for(auto i = 0; i < 5; ++i)
        data.push_back(Vector(CalibrableDerivation::size, i));

    CalibrableDerivation calibrable(path, "calibrable", 3);

    calibrable.setData(data);
    calibrable.publicUpdate();
    calibrable.publicWriteCalibrationFile();

    calibrable.setData(data);
    calibrable.publicUpdate();
    calibrable.publicWriteCalibrationFile();

    File file(file_path);
    std::vector<std::string> lines, expected_lines;
    file.read(&lines);

    for(auto i = 1; i < CalibrableDerivation::size+1; ++i)
    {
        expected_lines.push_back(calibration_data_format[i] + ",lb,-0.001000,-0.001000");
        expected_lines.push_back(calibration_data_format[i] + ",mean,2.000000,2.000000");
        expected_lines.push_back(calibration_data_format[i] + ",mid,2.000000,2.000000");
        expected_lines.push_back(calibration_data_format[i] + ",ub,4.001000,4.001000");
    }

    unsigned int i = 0;
    for(auto it = lines.begin()+1; it != lines.end(); ++it, ++i)
        EXPECT_TRUE(expected_lines[i] == *it);

    std::remove(&file_path[0]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "calibrable_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
