#define LOG_LEVEL Info

#include <stdio.h>
#include <gtest/gtest.h>
#include "bpf_localization/interfaces/interfaces.hpp"

class PublicFile: public File
{
    public:
        PublicFile(std::string path): File(path)
        {
        }

        bool publicOpen(bool force_creation = true) { return open(force_creation); }
        bool publicExist() { return exist(); }
        bool publicIsEmpty() { return isEmpty(); }
        bool publicClose() { return close(); }

        std::fstream* getPointer()
        {
            return file_;
        }
};

TEST(FileTests, FileTest0)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);
    std::fstream* pfile = fileObject.getPointer();
    std::remove(&path[0]);
    std::fstream file(path, ios::app);
    pfile = &file;
    EXPECT_TRUE(fileObject.publicExist());

    std::remove(&path[0]);
    EXPECT_FALSE(fileObject.publicExist());
}

TEST(FileTests, FileTest1)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);

    EXPECT_TRUE(fileObject.publicExist());

    // Remove file
    std::remove(&path[0]);
}

TEST(FileTests, FileTest2)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);
    std::remove(&path[0]);
    EXPECT_TRUE(fileObject.publicOpen());

    fileObject = PublicFile(path);
    std::remove(&path[0]);
    EXPECT_FALSE(fileObject.publicOpen(false));

    fileObject = PublicFile(path);
    EXPECT_TRUE(fileObject.publicOpen());

    std::remove(&path[0]);
}

TEST(FileTests, FileTest3)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);
    EXPECT_TRUE(fileObject.publicIsEmpty());

    fileObject.publicOpen();
    std::fstream* file = fileObject.getPointer();
    *file << "toto" << std::endl;
    file->close();
    EXPECT_FALSE(fileObject.publicIsEmpty());

    std::remove(&path[0]);
}

TEST(FileTests, FileTest4)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);
    fileObject.publicOpen();
    std::fstream* file = fileObject.getPointer();
    EXPECT_TRUE(file->is_open());

    fileObject.publicClose();

    EXPECT_FALSE(file->is_open());
}

TEST(FileTests, FileTest5)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);

    std::vector<std::string> lines, read_lines;
    for(auto i = 0; i < 10; i++)
        lines.push_back("line " + std::to_string(i+1));

    // We write two times but we close and reopen the file 
    // (so we replace and not append)
    fileObject.write(&lines);
    fileObject.write(&lines);

    std::fstream file(path, ios::in);
    std::string line;
    for(auto i = 0; i < lines.size(); ++i)
    {
        std::getline(file, line);
        read_lines.push_back(line);
    }

    EXPECT_TRUE(read_lines.size() == lines.size());
    for(auto i = 0; i < lines.size(); ++i)
        EXPECT_TRUE(lines[i] == read_lines[i]);

    std::remove(&path[0]);
}

TEST(FileTests, FileTest6)
{
    std::vector<std::string> lines;
    for(auto i = 0; i < 10; i++)
        lines.push_back("line " + std::to_string(i+1));

    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    PublicFile fileObject(path);
    fileObject.write(&lines);

    std::fstream file(path, ios::in);
    std::string line;
    for(auto i = 0; i < lines.size(); ++i)
    {
        std::getline(file, line);
        EXPECT_TRUE(lines[i] == line);
    }

    std::remove(&path[0]);
}

TEST(FileTests, FileTest7)
{
    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    std::vector<std::string> lines;

    PublicFile fileObject(path);
    EXPECT_FALSE(fileObject.read(&lines));
}

TEST(FileTests, FileTest8)
{
    std::vector<std::string> lines, read_lines;
    for(auto i = 0; i < 10; i++)
        lines.push_back("line " + std::to_string(i+1));

    std::string path = ros::package::getPath("bpf_localization");
    path += "/tests/interfaces/test.txt";
    std::remove(&path[0]);

    std::fstream file(path, std::ios::out);
    file << lines[0];
    for(auto it = lines.begin()+1; it != lines.end(); ++it)
        file << std::endl << *it;
    file.close();

    PublicFile fileObject(path);
    fileObject.read(&read_lines);

    EXPECT_TRUE(read_lines.size() == lines.size());
    for(auto i = 0; i < lines.size(); ++i)
        EXPECT_TRUE(lines[i] == read_lines[i]);
}

TEST(GenericTests, UniformDistribution1)
{
    unsigned int N = 1000;
    UniformDistribution distrib1, distrib2;

    for(auto i = 0; i < N; i++)
        EXPECT_TRUE(distrib1.get() != distrib2.get());
}

TEST(GenericTests, testUniformDistribution2)
{
    double lower = 0.0;
    double upper = 1.0;
    UniformDistribution distrib(lower, upper);

    unsigned int nb = 10;
    double size = (upper-lower)/nb;
    std::map<double, int> results;

    for(auto i = 0; i < nb; ++i)
        results.emplace(i*size, 0);

    double real;
    unsigned int N = 2e7;
    for(auto i = 0; i < N; ++i)
    {
        real = distrib.get();
        for(auto interval = results.begin(); interval != results.end(); ++interval)
        {
            if(Interval(interval->first, interval->first+size).contains(real))
            {
                interval->second += 1;
                break;
            }
        }
    }

    for(auto interval = results.begin(); interval != results.end(); ++interval)
        EXPECT_TRUE(std::abs(interval->second - double(N)/double(nb)) < 5e3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                ros::console::levels::LOG_LEVEL) ) // LOG_LEVEL defined as macro 
       ros::console::notifyLoggerLevelsChanged();
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "file_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
