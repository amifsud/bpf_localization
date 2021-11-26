#include <fstream>

#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

#include "bpf_localization/utils.hpp"
#include "bpf_localization/StartCalibration.h"
#include "bpf_localization/GetDiameters.h"

// FIXME: Make this file independant from ROS except for logging

/*** Available calibrations ***/
//  * midpoint offset : 0
//  * mean offset     : 1

#define CALIBRATION_POLICY 0

class Calibrable
{
    protected:
        std::string name_;
        unsigned int size_;

        bool calibration_;
        std::string calibration_file_name_;
        std::fstream* calibration_file_;
        std::vector<std::string> calibration_data_format_;

        ros::Time     init_time_;
        ros::Duration until_;
        ros::Duration time_;

        ros::ServiceServer start_calibration_server_;
        ros::ServiceServer stop_calibration_server_;

        std::vector<Vector> data_;
        Vector sum_;
        double nb_;
        Vector lb_;
        Vector ub_;
        Vector mid_;
        Vector mean_;

        Vector half_diameters_;
        double precision_;

    public:
        Calibrable( ros::NodeHandle* nh, std::string name, 
                    unsigned int size, unsigned int decimal):
            calibration_(false), name_(name), nb_(0), 
            precision_(pow(10, decimal)), 
            size_(size), sum_(Vector(size, 0.)), 
            half_diameters_(Vector(size, 0.)),
            lb_(Vector(size, 1e7)), ub_(Vector(size, -1e7)),
            mid_(Vector(size, 1e7)), mean_(Vector(size, 1e7))
        {
            start_calibration_server_ 
                = nh->advertiseService(name + "_start_calibration", 
                        &Calibrable::startCalibration, this);
            stop_calibration_server_ 
                = nh->advertiseService(name + "_stop_calibration", 
                        &Calibrable::stopCalibration, this);
            std::string path = ros::package::getPath("bpf_localization");
            calibration_file_name_ = path + "/data/calibrations/" + 
                                name_ + ".csv";
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
            init_time_ = ros::Time::now(); 
            until_     = ros::Duration(req.duration, 0);
            return true;
        }

        bool stopCalibration(bpf_localization::GetDiameters::Request  &req,
                             bpf_localization::GetDiameters::Response &res)
        {
            endingCalibration();
            for(auto i = 0; i < size_; ++i)
                res.diameters.push_back(2*getHalfDiameter(i));
            return true;
        }

        inline double getHalfDiameter(unsigned int i)
        {
            #if CALIBRATION_POLICY == 0 
            return (ub_[i]-lb_[i]);
            #elif CALIBRATION_POLICY == 1
            return 2*std::max(ub_[i]-mean_[i], mean_[i]-lb_[i]);
            #endif
        }

        void update()
        {
            for(auto vect = data_.begin(); vect !=data_.end(); vect++)
            {
                for(unsigned int i = 0; i < size_; ++i)
                {
                    if(vect->operator[](i) > ub_[i] + 1./precision_)
                    {
                        ub_[i] = roundf(vect->operator[](i) * precision_) 
                            / precision_ + 1./precision_;
                        mid_[i] = ub_[i] - lb_[i];
                    }

                    if(vect->operator[](i) < lb_[i] - 1./precision_) 
                    {
                        lb_[i] = roundf(vect->operator[](i) * precision_) 
                            / precision_ + 1./precision_;
                        mid_[i] = ub_[i] - lb_[i];
                    }
                }

                sum_ += *vect;
                nb_ += 1.;
            }

            time_ = ros::Time::now() - init_time_; 
            mean_ = 1./nb_*sum_;
            data_.clear();
        }

        void read_calibration_file( std::vector<std::string>* lines)
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

        void write_calibration_file(std::string filename)
        {
            ROS_INFO_STREAM("Begin to write calibration file");

            // Try to open file twice
            calibration_file_ = 
                new std::fstream(calibration_file_name_, std::ios::out | std::ios::in);
            if(!calibration_file_->is_open())
            {
                calibration_file_ = 
                    new std::fstream(calibration_file_name_, std::ios::out | std::ios::in);
            }

            if(calibration_file_->is_open())
            {
                // If file open
                std:vector<std::string> lines;
                calibration_file_->seekg(0, calibration_file_->end);
                if(calibration_file_->tellg() == 0)
                {
                    // If file empty : create lines with sensor format
                    ROS_INFO_STREAM("New file");
                    calibration_data_format();
                    lines.push_back("vector,component,data");
                    for(auto u = 0; u < calibration_data_format_.size(); ++u)
                    {
                        for(auto i = 0; i < size_; ++i)
                        {
                            lines.push_back(calibration_data_format_[u] + ",lb");
                            lines.push_back(calibration_data_format_[u] + ",ub");
                            lines.push_back(calibration_data_format_[u] + ",mid");
                            lines.push_back(calibration_data_format_[u] + ",mean");
                        }
                    }
                }
                else
                {
                    // If file not empty : read lines from file
                    read_calibration_file(&lines);
                }

                // Update calibration data
                lines.operator[](0) += "," + to_string(ros::Time::now().toSec());
                for(auto i = 0; i < size_; ++i)
                {
                    lines.operator[](i*4+1) += "," + to_string(lb_[i]);
                    lines.operator[](i*4+2) += "," + to_string(ub_[i]);
                    lines.operator[](i*4+3) += "," + to_string(mid_[i]);
                    lines.operator[](i*4+4) += "," + to_string(mean_[i]);
                }

                // Write lines and close
                calibration_file_->clear(); 
                calibration_file_->seekp(0, calibration_file_->beg);
                for(auto i = 0; i < lines.size(); ++i)
                {
                    *calibration_file_ << lines[i];
                    if(i != lines.size()-1) *calibration_file_ << "\n";
                }
                calibration_file_->close();
            }
            else
            {
                ROS_ASSERT_MSG(false, "File can't be open, please create it");
            }

            ROS_INFO_STREAM("End to write calibration file");
        }

        virtual void calibration_data_format()
        {
            ROS_ASSERT_MSG(false, 
                "You have to implement the update_calibration_data method in your sensor");
        }

        void endingCalibration()
        {
            ROS_INFO_STREAM("Stoppping calibration");
            calibration_ = false;
            update();
            write_calibration_file(calibration_file_name_);
        }

        void feed(const Vector& vect)
        {
            ROS_INFO_STREAM("Calibrating...");

            data_.push_back(Vector(vect));
            update();

            if(time_ >= until_)
                endingCalibration();
        }

        bool is_calibrating()
        {
            return calibration_;
        }
};

class Sensor: public Calibrable
{
    protected:
        // ROS
        ros::ServiceServer diameters_server_;
        ros::Subscriber sub_;
        Vector tmp_;

        // Interval
        std::deque<IntervalVector> buffer_;

    public:
        Sensor(ros::NodeHandle* nh, std::string name, unsigned int size, 
                unsigned int decimal):
            Calibrable(nh, name, size, decimal),
            tmp_(Vector(size, 0.))
        {
            diameters_server_ 
                = nh->advertiseService( name + "_diameters", 
                                        &Sensor::getDiameters, this);
        }

        Sensor( ros::NodeHandle* nh, std::string name, unsigned int size, 
                unsigned int decimal, const Vector& half_diameters):
            Calibrable(nh, name, size, decimal),
            tmp_(Vector(size, 0.))
        {
            diameters_server_ 
                = nh->advertiseService( name + "_diameters", 
                                        &Sensor::getDiameters, this);
        }

        bool getDiameters(bpf_localization::GetDiameters::Request  &req,
                          bpf_localization::GetDiameters::Response &res)
        {
            for(auto i = 0; i < size_; ++i)
                res.diameters.push_back(2*getHalfDiameter(i));
        }

        std::deque<IntervalVector> getIntervalData()
        {
            std::deque<IntervalVector> buffer(buffer_);
            buffer_.clear();
            return buffer;
        }

    protected:
        IntervalVector interval_from_vector(const Vector& data)
        {
            IntervalVector interval(size_);

            for(auto i = 0; i < size_; ++i)
            {
                interval[i] = Interval( data[i]-getHalfDiameter(i),
                                        data[i]+getHalfDiameter(i));
            }

            return interval;
        }

        void feed(const Vector& data)
        {
            buffer_.push_back(interval_from_vector(data));
            if(is_calibrating())
                Calibrable::feed(data);
        }
};

/** 
 *      Common sensors
 */

class IMUInterface: public Sensor
{
    public:
        static const unsigned int size = 6;

    public:
        IMUInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            Sensor(nh, name, size, decimal)
        {
            sub_ = nh->subscribe(name + "_in", 50, &IMUInterface::callback, this);
        }

    protected:
        void callback(const sensor_msgs::Imu& imu_data)
        {
            tmp_[0] = imu_data.angular_velocity.x;
            tmp_[1] = imu_data.angular_velocity.y;
            tmp_[2] = imu_data.angular_velocity.z;
            tmp_[3] = imu_data.linear_acceleration.x;
            tmp_[4] = imu_data.linear_acceleration.y;
            tmp_[5] = imu_data.linear_acceleration.z;
            feed(tmp_);
        }

        void calibration_data_format()
        {
            calibration_data_format_.push_back("angular_velocity,x");
            calibration_data_format_.push_back("angular_velocity,y");
            calibration_data_format_.push_back("angular_velocity,z");
            calibration_data_format_.push_back("linear_acceleration,x");
            calibration_data_format_.push_back("linear_acceleration,y");
            calibration_data_format_.push_back("linear_acceleration,z");
        }
};

class GPSInterface: public Sensor
{
    public:
        static const unsigned int size = 3;

    public:
        GPSInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            Sensor(nh, name, size, decimal)
        {
            sub_ = nh->subscribe(name + "_in", 50, &GPSInterface::callback, this);    
        }

    protected:
        void callback(const geometry_msgs::Point& gps_data)
        {
            tmp_[0] = gps_data.x;
            tmp_[1] = gps_data.y;
            tmp_[2] = gps_data.z;
            feed(tmp_);
        }
};
