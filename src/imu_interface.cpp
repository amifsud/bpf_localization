#include "bpf_localization/utils.hpp"

#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include "bpf_localization/Calibration.h"
#include "bpf_localization/sensors.h"

class Calibrable
{
    protected:
        bool calibration_;
        ros::Time until_;

        ros::ServiceServer calibration_server_;

        std::vector<Vector> data_;
        Vector mean_;
        Vector tmp_;
        double nb_;

        double precision_;

    public:
        Calibrable(ros::NodeHandle* nh, std::string name, unsigned int size, unsigned int decimal):
            calibration_(false), name_(name), nb_(0), precision_(pow(10, decimal)), size_(size)
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

        void feed_calibration(const Vector& vect, Vector& half_diameters)
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

class IMUInterface: public Calibrable
{
    public:
        static const size = 6;

    protected:
        // ROS IMU
        ros::Subscriber imu_sub_;
        Vector imu_tmp_;_
        
        // Interval IMU
        std::string name_;
        Vector half_diameters_;

    public:
        IMUInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            Calibrable(nh, name, IMUInterface::size, decimal) 
        {
            imu_sub_ = nh->subscribe("imu_in", 50, &IMUInterface::callback, this);    
        }

    protected:
        void callback(const sensor_msgs::Imu& imu_in) 
        {
            if(!is_calibrating())
            {
                ROS_INFO_STREAM(half_diameters_);
            }
            else
            {
                imu_tmp_[0] = imu_in.angular_velocity.x;
                imu_tmp_[1] = imu_in.angular_velocity.y;
                imu_tmp_[2] = imu_in.angular_velocity.z;
                imu_tmp_[3] = imu_in.linear_acceleration.x;
                imu_tmp_[4] = imu_in.linear_acceleration.y;
                imu_tmp_[5] = imu_in.linear_acceleration.z;
                feed_calibration(imu_tmp_, half_diameters_);
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_interface");
    ros::NodeHandle nh;

    auto imu = IMUInterface(&nh, "boat_imu", 3);

    ros::Rate r(5.0);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
