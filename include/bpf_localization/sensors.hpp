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
        ros::Time     init_time_;
        ros::Duration until_;
        ros::Duration     time_;

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
            lb_(Vector(size, 0.)), ub_(Vector(size, 0.)),
            mid_(Vector(size, 0.)), mean_(Vector(size, 0.))
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
            until_     = ros::Duration(req.duration, 0);
            return true;
        }

        bool stopCalibration(bpf_localization::GetDiameters::Request  &req,
                             bpf_localization::GetDiameters::Response &res)
        {
            calibration_ = false;
            update();
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
                    if(vect->operator[](i) - ub_[i] > 1./precision_)
                    {
                        ub_[i] = roundf(vect->operator[](i) * precision_) 
                            / precision_ + 1./precision_;
                        mid_[i] = ub_[i] - lb_[i];
                    }

                    if(lb_[i] - vect->operator[](i) < 1./precision_) 
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

        void feed(const Vector& vect)
        {
            ROS_INFO_STREAM("Calibrating...");

            data_.push_back(Vector(vect));
            update();

            if(time_ >= until_) calibration_ = false;
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
