#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

#include "bpf_localization/utils.hpp"
#include "bpf_localization/StartCalibration.h"
#include "bpf_localization/GetDiameters.h"

// FIXME: Make this file independant from ROS except for logging

class Calibrable
{
    protected:
        std::string name_;
        unsigned int size_;

        bool calibration_;
        ros::Time     init_time_;
        ros::Duration until_;
        ros::Duration     time_;

        ros::ServiceServer start_calibration_server_;
        ros::ServiceServer stop_calibration_server_;

        std::vector<Vector> data_;
        Vector sum_;
        double nb_;

        Vector half_diameters_;
        double precision_;

    public:
        Calibrable( ros::NodeHandle* nh, std::string name, 
                    unsigned int size, unsigned int decimal):
            calibration_(false), name_(name), nb_(0), precision_(pow(10, decimal)), 
            size_(size), sum_(Vector(size, 0.)), half_diameters_(Vector(size, 0.)) 
        {
            start_calibration_server_ 
                = nh->advertiseService(name + "_start_calibration", 
                        &Calibrable::startCalibration, this);
            stop_calibration_server_ 
                = nh->advertiseService(name + "_stop_calibration", 
                        &Calibrable::stopCalibration, this);
        }

    protected:
        bool startCalibration(bpf_localization::StartCalibration::Request &req,
                         bpf_localization::StartCalibration::Response     &res)
        {
            calibration_ = true;
            sum_ = Vector(size_, 0.);
            nb_ = 0.;
            data_.clear();
            init_time_ = ros::Time::now(); 
            until_ = ros::Duration(req.duration, 0);
            return true;
        }

        bool stopCalibration(bpf_localization::GetDiameters::Request  &req,
                             bpf_localization::GetDiameters::Response &res)
        {
            calibration_ = false;
            computeHalfDiameters(&half_diameters_);
            for(auto i = 0; i < half_diameters_.size(); ++i)
                res.diameters.push_back(2*half_diameters_[i]);
            return true;
        }

        Vector* computeHalfDiameters(Vector* half_diameters)
        {
            //Vector half_diameters(size_, 0.);

            double number;
            for(auto vect = data_.begin(); vect !=data_.end(); vect++)
            {
                for(unsigned int i = 0; i < size_; ++i)
                {
                    number = 2*std::abs(vect->operator[](i)-sum_[i]/nb_);
                    if(number - half_diameters->operator[](i) > 1./precision_) 
                        half_diameters->operator[](i) 
                            = roundf(number * precision_) / precision_;
                }
            }

            for(unsigned int i = 0; i < size_; ++i)
                half_diameters->operator[](i) += 1./precision_;
        }

        void feed(const Vector& vect)
        {
            ROS_INFO_STREAM("Calibrating...");

            time_ = ros::Time::now() - init_time_; 
            if(time_ < until_)
            {
                sum_ += vect;
                data_.push_back(Vector(vect));
                nb_ += 1.;
                computeHalfDiameters(&half_diameters_); 
            }
            else
            {
                calibration_ = false;
            }    
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
                = nh->advertiseService(name + "_diameters", &Sensor::getDiameters, this);
        }

        Sensor( ros::NodeHandle* nh, std::string name, unsigned int size, 
                unsigned int decimal, const Vector& half_diameters):
            Calibrable(nh, name, size, decimal),
            tmp_(Vector(size, 0.))
        {
            diameters_server_ 
                = nh->advertiseService(name + "_diameters", &Sensor::getDiameters, this);
        }

        bool getDiameters(bpf_localization::GetDiameters::Request  &req,
                          bpf_localization::GetDiameters::Response &res)
        {
            for(auto i = 0; i < half_diameters_.size(); ++i)
                res.diameters.push_back(2*half_diameters_[i]);
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
                interval[i] = Interval( data[i]-half_diameters_[i],
                                        data[i]+half_diameters_[i]);
            }

            return interval;
        }

        void feed(const Vector& data)
        {
            if(!is_calibrating())
            {
                buffer_.push_back(interval_from_vector(data));
            }
            else
            {
                Calibrable::feed(data);
            }
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
