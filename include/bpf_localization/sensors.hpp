#include <fstream>

#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <interval_msgs/Vector3IntervalStamped.h>

#include "bpf_localization/utils.hpp"
#include "bpf_localization/StartCalibration.h"
#include "bpf_localization/GetDiameters.h"
#include "bpf_localization/IntervalIMU.h"

/** FIXME: 
 * Make this file independant from ROS except for logging
 * Make unit tests for the file
**/

/*** Available calibrations ***/
//  * midpoint offset : 0
//  * mean offset     : 1

//#define CALIBRATION_POLICY 0

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
        Vector lb_;
        Vector ub_;
        Vector mid_;
        Vector half_diameters_;

        Vector sum_;
        double nb_;
        Vector mean_;

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
                res.diameters.push_back(2*half_diameters_[i]);
            return true;
        }

        void computeHalfDiameters()
        {
            mid_ = 0.5*(ub_ + lb_);
            half_diameters_ = ub_ - lb_;
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
                    }

                    if(vect->operator[](i) < lb_[i] - 1./precision_) 
                    {
                        lb_[i] = roundf(vect->operator[](i) * precision_) 
                            / precision_ + 1./precision_;
                    }
                }
                sum_ += *vect;
                nb_ += 1.;
            }

            computeHalfDiameters();
            mean_ = 1./nb_*sum_;
            time_ = ros::Time::now() - init_time_; 
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
            while(getline(stream, number, ','))
                values->push_back(std::stod(number));
        }

        bool openCalibrationFile()
        {
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
            if(openCalibrationFile())
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
                    lb_[u] = *std::min_element(values.begin(), values.end());

                    line   = lines[u*4+2];
                    format = calibration_data_format_[u+1]+",ub,";
                    spliToDouble(&line, &format, &values);
                    ub_[u] = *std::max_element(values.begin(), values.end());
                }

                computeHalfDiameters();
            }

            closeCalibrationFile();
        }

        void writeCalibrationFile()
        {
            ROS_INFO_STREAM("Begin to write calibration file");

            if(openCalibrationFile())
            {
                // If file open
                std:vector<std::string> lines;
                calibration_file_->seekg(0, calibration_file_->end);
                if(calibration_file_->tellg() == 0)
                {
                    // If file empty : create lines with sensor format
                    ROS_INFO_STREAM("New file");
                    calibrationDataFormat();
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
                    readCalibrationFile(&lines);
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

        void feed(const Vector& vect)
        {
            ROS_INFO_STREAM("Calibrating...");

            data_.push_back(Vector(vect));
            update();

            if(time_ >= until_) endingCalibration();
        }

        bool isCalibrating()
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
        ros::Publisher pub_;

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

        bool getDiameters(bpf_localization::GetDiameters::Request  &req,
                          bpf_localization::GetDiameters::Response &res)
        {
            if(half_diameters_.max() < 1e-7) loadFromFile();
            for(auto i = 0; i < size_; ++i)
                res.diameters.push_back(2*half_diameters_[i]);
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

        void feed(const Vector& data)
        {
            buffer_.push_back(intervalFromVector(data));
            if(isCalibrating()) Calibrable::feed(data);
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
            pub_ = nh->advertise<bpf_localization::IntervalIMU>(name+"_out", 1000);
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

            IntervalVector interval = intervalFromVector(tmp_);

            bpf_localization::IntervalIMU msg;
            msg.header = imu_data.header;
            msg.angular_velocity.x.lb    = interval[0].lb();
            msg.angular_velocity.x.ub    = interval[0].ub();
            msg.angular_velocity.y.lb    = interval[1].lb();
            msg.angular_velocity.y.ub    = interval[1].ub();
            msg.angular_velocity.z.lb    = interval[2].lb();
            msg.angular_velocity.z.ub    = interval[2].ub();
            msg.linear_acceleration.x.lb = interval[3].lb();
            msg.linear_acceleration.x.ub = interval[3].ub();
            msg.linear_acceleration.y.lb = interval[4].lb();
            msg.linear_acceleration.y.ub = interval[4].ub();
            msg.linear_acceleration.z.lb = interval[5].lb();
            msg.linear_acceleration.z.ub = interval[5].ub();

            pub_.publish(msg);
        }

        void calibrationDataFormat()
        {
            calibration_data_format_.push_back("vector,component,data");
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
        Vector initial_pose_;
        bool initialized_;

    public:
        GPSInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            Sensor(nh, name, size, decimal),
            initial_pose_(Vector(3,0.)), initialized_(false)
        {
            sub_ = nh->subscribe(name + "_in", 50, &GPSInterface::callbackOdom, this);    
            pub_ = nh->advertise<interval_msgs::Vector3IntervalStamped>(name+"_out", 1000);
            half_diameters_ = 1e-1*Vector(size_, 1.);
        }

    protected:
        void callbackOdom(const nav_msgs::Odometry& gps_data)
        {
            geometry_msgs::PointStamped msg;
            msg.header = gps_data.header;
            msg.point  = gps_data.pose.pose.position;
            callbackPoint(msg);
        }

        void callbackPoint(const geometry_msgs::PointStamped& gps_data)
        {
            ROS_INFO_STREAM("In callback");
            if(!initialized_)
            {
                if( gps_data.point.x != 0. | 
                    gps_data.point.y != 0. |
                    gps_data.point.z != 0. )
                {
                    initial_pose_[0] = gps_data.point.x;
                    initial_pose_[1] = gps_data.point.y;
                    initial_pose_[2] = gps_data.point.z;
                    initialized_ = true;
                }
            }

            tmp_[0] = gps_data.point.x - initial_pose_[0];
            tmp_[1] = gps_data.point.y - initial_pose_[1];
            tmp_[2] = gps_data.point.z - initial_pose_[2];
            feed(tmp_);

            IntervalVector interval = intervalFromVector(tmp_);

            interval_msgs::Vector3IntervalStamped msg;
            msg.header = gps_data.header;
            msg.vector.x.lb = interval[0].lb();
            msg.vector.x.ub = interval[0].ub();
            msg.vector.y.lb = interval[1].lb();
            msg.vector.y.ub = interval[1].ub();
            msg.vector.z.lb = interval[2].lb();
            msg.vector.z.ub = interval[2].ub();

            pub_.publish(msg);
        }

        void calibrationDataFormat()
        {
            calibration_data_format_.push_back("component");
            calibration_data_format_.push_back("x");
            calibration_data_format_.push_back("y");
            calibration_data_format_.push_back("z");
        }
};
