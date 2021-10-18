#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

#include "bpf_localization/utils.hpp"
#include "bpf_localization/Calibration.h"

class Calibrable
{
    protected:
        std::string name_;
        unsigned int size_;

        bool calibration_;
        ros::Time until_;

        ros::ServiceServer calibration_server_;

        std::vector<Vector> data_;
        Vector mean_;
        double nb_;

        double precision_;

    public:
        Calibrable( ros::NodeHandle* nh, std::string name, 
                    unsigned int size, unsigned int decimal):
            calibration_(false), name_(name), nb_(0), precision_(pow(10, decimal)), 
            size_(size), mean_(Vector(size, 0.)) 
        {
            calibration_server_ 
                = nh->advertiseService(name + "_calibration", &Calibrable::calibration, this);
        }

    protected:
        bool calibration(bpf_localization::Calibration::Request  &req,
                         bpf_localization::Calibration::Response &res)
        {
            calibration_ = true;
            mean_ = Vector(size_, 0.);
            nb_ = 0.;
            data_.clear();
            until_ = ros::Time::now() + ros::Duration(req.calibration_duration, 0);
            return true;
        }

        Vector computeHalfDiameters()
        {
            Vector half_diameters(size_, 0.);

            double number;
            for(auto vect = data_.begin(); vect !=data_.end(); vect++)
            {
                for(unsigned int i = 0; i < size_; ++i)
                {
                    number = 2*std::abs(vect->operator[](i)-mean_[i]/nb_);
                    if(number - half_diameters[i] > 1./precision_) 
                        half_diameters[i] 
                            = roundf(number * precision_) / precision_;
                }
            }

            for(unsigned int i = 0; i < size_; ++i)
            {
                half_diameters[i] += 1./precision_;
                ROS_INFO_STREAM(mean_[i]/nb_);
            }

            ROS_INFO_STREAM(half_diameters);

            return half_diameters;
        }

        void feed(const Vector& vect, Vector& half_diameters)
        {
            ROS_INFO_STREAM("Calibrating...");
            if(ros::Time::now().toSec() < until_.toSec())
            {
                mean_ += vect;
                data_.push_back(Vector(vect));
                nb_ += 1.;

                computeHalfDiameters(); 
            }
            else
            {
                calibration_ = false;
                half_diameters = computeHalfDiameters(); 
            }
        }

        bool is_calibrating()
        {
            return calibration_;
        }
};

class Sensor
{
    protected:
        unsigned int size_;
        
        // Interval
        Vector half_diameters_;
        std::deque<IntervalVector> buffer_;

    public:
        Sensor(ros::NodeHandle* nh, unsigned int size):
            half_diameters_(Vector(size, 0.)), size_(size)
        {
        }

        Sensor( ros::NodeHandle* nh, unsigned int size, 
                const Vector& half_diameters):
            half_diameters_(half_diameters), size_(size)
        {
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
            buffer_.push_back(interval_from_vector(data));
        }
};

class CalibrableSensor: public Calibrable, public Sensor
{
    public:
        CalibrableSensor(ros::NodeHandle* nh, std::string name, 
                         unsigned int size, unsigned int decimal):
            Calibrable(nh, name, size, decimal),
            Sensor(nh, size)
        {
        }

    protected:
        void callback(const Vector& data) 
        {
            if(!is_calibrating())
            {
                Sensor::feed(data);
            }
            else
            {
                ROS_INFO_STREAM("Calibrating...");
                Calibrable::feed(data, half_diameters_);
            }
        }
};

/** 
 *      Common sensors
 */

class IMUInterface: public CalibrableSensor
{
    public:
        static const unsigned int size = 6;

    protected:
        ros::Subscriber sub_;
        Vector tmp_;

    public:
        IMUInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            CalibrableSensor(nh, name, size, decimal),
            tmp_(Vector(size, 0.))
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
            CalibrableSensor::callback(tmp_);
        }
};

class GPSInterface: public CalibrableSensor
{
    public:
        static const unsigned int size = 3;

    protected:
        ros::Subscriber sub_;
        Vector tmp_;

    public:
        GPSInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            CalibrableSensor(nh, name, size, decimal),
            tmp_(Vector(size, 0.))
        {
            sub_ = nh->subscribe(name + "_in", 50, &GPSInterface::callback, this);    
        }

    protected:
        void callback(const geometry_msgs::Point& gps_data)
        {
            tmp_[0] = gps_data.x;
            tmp_[1] = gps_data.y;
            tmp_[2] = gps_data.z;
            CalibrableSensor::callback(tmp_);
        }
};
