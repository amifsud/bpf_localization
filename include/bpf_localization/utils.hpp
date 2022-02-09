#ifndef UTILS
#define UTILS

#include <ros/ros.h>
#include <vector>
#include <utility>
#include <numeric>
#include <random>
#include <memory>
#include <math.h>

#define NaN std::numeric_limits<double>::quiet_NaN()

#ifndef WO_IBEX

#include "ibex/ibex.h"
using namespace ibex;

#include "bpf_localization/dynamical_systems/functions.hpp"

std::random_device rd;

class UniformDistribution 
{
    public:
        UniformDistribution(double lower = 0.0, double upper = 1.0):
            uniform_distribution_(lower, upper),
            generator_(rd())
        {
        }

        UniformDistribution(std::random_device device, double lower = 0.0, double upper = 1.0):
            uniform_distribution_(lower, upper),
            generator_(device())
        {
        }

        double get()
        {
            return uniform_distribution_(generator_);
        }

    protected:
        std::default_random_engine generator_;
        std::uniform_real_distribution<double> uniform_distribution_;
};

class File
{
    public:
        File(std::string path)
        {
            file_name_ = path;
            ROS_ASSERT_MSG(createFile(), "File not created");
        }

        bool isEmpty()
        {
            bool res;
            ROS_ASSERT_MSG(open(false), "file can't be open");
            if(file_->is_open())
            {
                unsigned int previous_line = file_->tellg();
                goReadEnd();
                res = file_->tellg() == 0;
                file_->seekg(previous_line);
            }
            else
            {
                ROS_INFO_STREAM("File not open");
            }
            close();

            return res;
        }

        bool read(std::vector<std::string>* lines)
        {
            if(!isEmpty())
            {
                open();
                std::string line;
                goReadBegin();
                while(!file_->eof())
                {
                    std::getline(*file_, line);
                    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
                    lines->push_back(line);
                }
                close();
                return true;
            }
            else
            {
                close();
                return false;
            }
        }

        bool write(std::vector<std::string>* lines)
        {
            if(open())
            {
                goWriteBegin();
                for(auto i = 0; i < lines->size(); ++i)
                {
                    *file_ << lines->operator[](i);
                    if(i != lines->size()-1) *file_ << "\n";
                }

                close();
                return true;
            }
            else
            {
                return false;
            }
        }

    protected:
        bool createFile()
        {
            if(file_name_ != "")
            {
                std::fstream file(file_name_, ios::app);
                file.close();
            }
            else
            {
                ROS_ASSERT_MSG(false, "No calibration file provided");
            }
            return exist();
        }

        bool exist()
        {
            std::fstream infile(file_name_, std::ios::in);
            return infile.good();
        }

        bool open(bool force_creation = true)
        {
            if(file_name_ != "")
            {
                // Try to open file twice
                file_ = new std::fstream(file_name_, std::ios::out | std::ios::in);
                if(!file_->is_open())
                {
                    if(force_creation) createFile();
                    file_ = new std::fstream(file_name_, std::ios::out | std::ios::in);
                }
            }
            else
            {
                ROS_ASSERT_MSG(false, "No calibration file provided");
            }

            //ROS_ASSERT_MSG(file_->is_open(), "Problem to open file");

            return file_->is_open();
        }

        inline void goWriteBegin(){ file_->seekp(0, file_->beg); }
        inline void goWriteEnd(){ file_->seekp(0, file_->end); }
        inline void goReadBegin(){ file_->seekg(0, file_->beg); }
        inline void goReadEnd(){ file_->seekg(0, file_->end); }

        bool close()
        {
            if(file_->is_open())
            {
                file_->close();
            }
            else
            {
                ROS_INFO_STREAM("File was not open");
            }
        }

    protected:
        std::string   file_name_;
        std::fstream* file_;
};

class ReturnIMU
{
    public:
        ReturnIMU(  const ExprNode& f1, const ExprNode& f2, const ExprNode& f3, 
                    const ExprNode& f4, const ExprNode& f5, const ExprNode& f6, 
                    const ExprNode& f7, const ExprNode& f8, const ExprNode& f9, 
                    const ExprNode& f10, bool in_rows=false) :
            vec(ExprVector::new_(Array<const ExprNode>(f1,f2,f3,f4,f5,f6,f7,f8,f9,f10),
                in_rows)) 
        {}

        operator const ExprVector&() const     { return vec; }
        operator const ExprNode&() const       { return vec; }
        const ExprIndex& operator[](int index) { return vec[index]; }
        const ExprVector& vec;
};

inline bool eps_equals( IntervalVector v1, IntervalVector v2, double eps = 1e-13)
{
    return v1.is_subset(IntervalVector(v2).inflate(eps)) && v2.is_subset(v1.inflate(eps));
}

inline bool eps_equals( Vector v1, Vector v2, double eps = 1e-13)
{
    bool b = true;
    for(auto i = 0; i < v1.size(); ++i)
        if(std::abs(v1[i] - v2[i]) > eps) b = false;
    return b;
}

#endif

#endif
