
/** FIXME: 
 * Make this file independant from ROS except for logging
 * Make unit tests for the file
**/

/**
 * \file   interfaces.hpp
 * \brief  Interfaces
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef INTERFACES
#define INTERFACES

#include <fstream>

#include <ros/package.h>
#include <message_filters/subscriber.h>

#include "bpf_localization/utils.hpp"
#include "bpf_localization/StartCalibration.h"
#include "bpf_localization/GetDiameters.h"


/*! Interfaces
 *
 *  \brief Generic classes for interfaces
 *
 *  **For now only Sensors are implemented**
 *
 */
namespace Interfaces
{

    class File
    {
        public:
            File(std::string path)
            {
                file_name_ = path;
                ROS_ASSERT_MSG(createFile(), "File not created");
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

            void goWriteBegin()
            {
                if(file_->is_open())
                {
                    file_->seekp(0, file_->beg);
                }
                else
                {
                    ROS_INFO_STREAM("File not open");
                }
            }

            void goReadBegin()
            {
                if(file_->is_open())
                {
                    file_->seekg(0, file_->beg);
                }
                else
                {
                    ROS_INFO_STREAM("File not open");
                }
            }

            void goReadEnd()
            {
                if(file_->is_open())
                {
                    file_->seekg(0, file_->end);
                }
                else
                {
                    ROS_INFO_STREAM("File not open");
                }
            }

            void goWriteEnd()
            {
                if(file_->is_open())
                {
                    file_->seekp(0, file_->end);
                }
                else
                {
                    ROS_INFO_STREAM("File not open");
                }
            }

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

    /*! \class Calibrable
     *
     *
     *
     */
    class Calibrable
    {
        public:
            Calibrable(std::string path, std::string name, unsigned int size, unsigned int decimal):
                calibration_(false), name_(name), nb_(0), 
                precision_(pow(10, decimal)), 
                size_(size), sum_(Vector(size, 0.)), 
                half_diameters_(Vector(size, 0.)),
                calibration_file_(path + name + ".csv")
            {
                data_map_.emplace("lb",   Vector(size, 1e7));
                data_map_.emplace("ub",   Vector(size, -1e7));
                data_map_.emplace("mid",  Vector(size, 1e7));
                data_map_.emplace("mean", Vector(size, 1e7));
            }

        protected:
            void computeHalfDiameters()
            {
                data_map_.at("mid") = 0.5*(data_map_.at("ub") + data_map_.at("lb"));
                half_diameters_ = data_map_.at("ub") - data_map_.at("lb");
            }

            void update()
            {
                for(auto vect = data_.begin(); vect !=data_.end(); vect++)
                {
                    for(unsigned int i = 0; i < size_; ++i)
                    {
                        if(vect->operator[](i) > data_map_.at("ub")[i] + 1./precision_)
                        {
                            data_map_.at("ub")[i] 
                                = roundf(vect->operator[](i) * precision_) 
                                    / precision_ + 1./precision_;
                        }

                        if(vect->operator[](i) < data_map_.at("lb")[i] - 1./precision_) 
                        {
                            data_map_.at("lb")[i] 
                                = roundf(vect->operator[](i) * precision_) 
                                    / precision_ + 1./precision_;
                        }
                    }
                    sum_ += *vect;
                    nb_ += 1.;
                }

                computeHalfDiameters();
                data_map_.at("mean") = 1./nb_*sum_;
                data_.clear();
            }

            void readCalibrationFile( std::vector<std::string>* lines)
            {
                std::string line;
                calibration_file_->seekp(0, calibration_file_->beg);
                while(!calibration_file_->eof())
                {
                    std::getline(*calibration_file_, line);
                    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
                    lines->push_back(line);
                }
            }

            void spliToDouble(std::string* line, std::string* format,
                                                std::vector<double>* values)
            {
                values->clear();
                std::string number;
                line->erase(0, format->size());
                std::stringstream stream(*line);
                while(getline(stream, number, ',')) values->push_back(std::stod(number));
            }

            bool openCalibrationFile()
            {
                if(calibration_file_name_ != "")
                {
                    // Try to open file twice
                    calibration_file_ = new std::fstream(calibration_file_name_, 
                                                         std::ios::out | std::ios::in);
                    if(!calibration_file_->is_open())
                    {
                        calibration_file_ = new std::fstream(
                                calibration_file_name_, std::ios::out | std::ios::in);
                    }
                }
                else
                {
                    ROS_ASSERT_MSG(false, "No calibration file provided");
                }

                if(calibration_file_->is_open()) 
                {
                    calibrationDataFormat();
                }
                else
                {
                    ROS_ASSERT_MSG(false, "File can't be open, please create it");
                }

                return calibration_file_->is_open();
            }

            bool closeCalibrationFile()
            {
                calibration_file_->close();
                calibration_data_format_.clear();
            }

            void loadFromFile()
            {
                if(openCalibrationFile() & !isCalibrationFileEmpty())
                {
                    // If file open
                    std:vector<std::string> lines;
                    readCalibrationFile(&lines);
                    std::string line, format;
                    std::vector<double> values;

                    for(auto u = 0; u < calibration_data_format_.size()-1; ++u)
                    {
                        line   = lines[u*4+1];
                        format = calibration_data_format_[u+1]+",lb,";
                        spliToDouble(&line, &format, &values);
                        data_map_.at("lb")[u] 
                            = *std::min_element(values.begin(), values.end());

                        line   = lines[u*4+2];
                        format = calibration_data_format_[u+1]+",ub,";
                        spliToDouble(&line, &format, &values);
                        data_map_.at("ub")[u] 
                            = *std::max_element(values.begin(), values.end());
                    }

                    computeHalfDiameters();
                }

                closeCalibrationFile();
            }

            bool isCalibrationFileEmpty()
            {
                unsigned int previous_line = calibration_file_->tellg();
                calibration_file_->seekg(0, calibration_file_->end);
                bool res = calibration_file_->tellg() == 0;
                calibration_file_->seekg(previous_line);
                return res;
            }

            void writeCalibrationFile()
            {
                ROS_INFO_STREAM("Begin to write calibration file");

                if(openCalibrationFile())
                {
                    // If file open
                    std:vector<std::string> lines;
                    calibration_file_->seekg(0, calibration_file_->end);
                    if(isCalibrationFileEmpty())
                    {
                        // If file empty : create lines with sensor format
                        ROS_INFO_STREAM("New file");
                        calibrationDataFormat();
                        lines.push_back("vector,component,data");
	                    for(auto u = 0; u < calibration_data_format_.size(); ++u)
	                    {
	                        for(auto it = data_map_.begin(); it != data_map_.end(); ++it)
	                        {
	                            lines.push_back(
	                                calibration_data_format_[u] + "," + it->first);
	                        }
	                    }
                    }
                    else
                    {
                        // If file not empty : read lines from file
                        readCalibrationFile(&lines);
                    }

                    // Update calibration data
	                lines.operator[](0) += "," + to_string(ros::Time::now().toSec());
	                unsigned int u;
	                for(auto i = 0; i < size_; ++i)
	                {
	                    u = 1;
	                    for(auto it = data_map_.begin(); it != data_map_.end(); ++it)
	                    {
	                        lines.operator[](i*data_map_.size()+u) 
	                            += ","+to_string(it->second.operator[](i));
	                        u += 1;
	                    }
	                }

                    // Write lines and close
                    calibration_file_->clear(); 
                    calibration_file_->seekp(0, calibration_file_->beg);
                    for(auto i = 0; i < lines.size(); ++i)
                    {
                        *calibration_file_ << lines[i];
                        if(i != lines.size()-1) *calibration_file_ << "\n";
                    }
                }

                closeCalibrationFile();

                ROS_INFO_STREAM("End to write calibration file");
            }

            virtual void calibrationDataFormat() = 0;

            void endingCalibration()
            {
                ROS_INFO_STREAM("Stoppping calibration");
                calibration_ = false;
                update();
                writeCalibrationFile();
            }

            void feed(const Vector& vect, unsigned int time)
            {
                ROS_INFO_STREAM("Calibrating...");

                data_.push_back(Vector(vect));
                update();
                time_ = time - init_time_; 

                if(time_ >= until_) endingCalibration();
            }

            bool isCalibrating()
            {
                return calibration_;
            }

        protected:
            unsigned int size_;

            bool calibration_;
            std::string name_;
            std::fstream* calibration_file_;

            std::map<std::string, Vector> data_map_;
            std::vector<std::string> calibration_data_format_;

            unsigned int init_time_;
            unsigned int until_;
            unsigned int time_;

            std::vector<Vector> data_;
            Vector half_diameters_;
            Vector sum_;
            double nb_;

            double precision_;

    };

    /*! Sensors
     *  
     *  \brief Sensors classes
     *
     */
    namespace Sensors
    {
        class SensorInterface: public Calibrable
        {
            public:
                SensorInterface(std::string path, std::string name, unsigned int size, unsigned int decimal):
                    Calibrable(path, name, size, decimal)
                {
                }

                std::deque<IntervalVector> getIntervalData()
                {
                    std::deque<IntervalVector> buffer(buffer_);
                    buffer_.clear();
                    return buffer;
                }

                IntervalVector getFirstIntervalValue()
                {
                    if(!buffer_.empty())
                    {
                        auto interval = IntervalVector(buffer_.front());
                        buffer_.pop_front();
                        return interval;
                    }
                    else
                    {
                        throw 1;
                    }
                }

            protected:
                IntervalVector intervalFromVector(const Vector& data)
                {
                    IntervalVector interval(size_);
                    if(half_diameters_.max() < 1e-7) loadFromFile();

                    for(auto i = 0; i < size_; ++i)
                    {
                        interval[i] = Interval( data[i]-half_diameters_[i],
                                                data[i]+half_diameters_[i]);
                    }

                    return interval;
                }

                void feed(const Vector& data, unsigned int time)
                {
                    buffer_.push_back(intervalFromVector(data));
                    if(isCalibrating()) Calibrable::feed(data, time);
                }

            protected:
                // Interval
                std::deque<IntervalVector> buffer_;
        };

        /*! ROS
         *
         *  \brief ROS Interface for Sensors
         *
         */
        namespace ROS
        {
            class ROSInterface: virtual public SensorInterface
            {
                public:
                    ROSInterface(ros::NodeHandle* nh, std::string path, std::string name, 
                        unsigned int size, unsigned int decimal):
                        SensorInterface(path, name, size, decimal),
                        tmp_(Vector(size, 0.))
                    {
                        start_calibration_server_ 
                            = nh->advertiseService(name + "_start_calibration", 
                                    &ROSInterface::startCalibration, this);

                        stop_calibration_server_ 
                            = nh->advertiseService(name + "_stop_calibration", 
                                    &ROSInterface::stopCalibration, this);

                        diameters_server_ 
                            = nh->advertiseService( name + "_diameters", 
                                                    &ROSInterface::getDiameters, this);
                    }

                    bool getDiameters(bpf_localization::GetDiameters::Request  &req,
                                      bpf_localization::GetDiameters::Response &res)
                    {
                        if(half_diameters_.max() < 1e-7) loadFromFile();
                        for(auto i = 0; i < size_; ++i)
                            res.diameters.push_back(2*half_diameters_[i]);
                    }

                protected:
                    bool startCalibration(bpf_localization::StartCalibration::Request &req,
                                     bpf_localization::StartCalibration::Response     &res)
                    {
                        ROS_INFO_STREAM("Start calibration");
                        calibration_ = true;
                        sum_ = Vector(size_, 0.);
                        nb_ = 0.;
                        data_.clear();
                        init_time_ = (unsigned int)(ros::Time::now().toSec()); 
                        until_     = (unsigned int)(ros::Duration(req.duration, 0).toSec());
                        return true;
                    }

                    bool stopCalibration(bpf_localization::GetDiameters::Request  &req,
                                         bpf_localization::GetDiameters::Response &res)
                    {
                        endingCalibration();
                        for(auto i = 0; i < size_; ++i)
                            res.diameters.push_back(2*half_diameters_[i]);
                        return true;
                    }

                protected:
                    ros::ServiceServer start_calibration_server_;
                    ros::ServiceServer stop_calibration_server_;

                    ros::ServiceServer diameters_server_;
                    ros::Subscriber sub_;
                    ros::Publisher pub_;

                    Vector tmp_;
            };

        }
    }
}

#endif
