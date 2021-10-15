#include "bpf_localization/utils.hpp"

#include <message_filters/subscriber.h>
#include <sensor_msgs/Imu.h>
#include "bpf_localization/Calibration.h"

class IMUInterface
{
    protected:
        // ROS IMU
        ros::Subscriber imu_sub_;
        
        // Calibration
        ros::ServiceServer calibration_server_;
        ros::Time until_;
        std::vector<Vector> data_;
        Vector mean_;
        Vector tmp_;
        double nb_;
        bool calibration_;
        double precision_;

        // Interval IMU
        std::string name_;
        Vector diameters_;

    public:
        IMUInterface(ros::NodeHandle* nh, std::string name, unsigned int decimal):
            calibration_(false), name_(name), diameters_(6, 0.),
            mean_(Vector(6,0.)), nb_(0), tmp_(6), precision_(pow(10, decimal))
        {
            imu_sub_ = nh->subscribe("imu_in", 50, &IMUInterface::callback, this);
            
            calibration_server_ 
                = nh->advertiseService("imu_calibration", &IMUInterface::calibration, this);
        }

    protected:
        bool calibration(bpf_localization::Calibration::Request  &req,
                         bpf_localization::Calibration::Response &res)
        {
            calibration_ = true;
            mean_ = Vector(6, 0.);
            nb_ = 0.;
            data_.clear();
            until_ = ros::Time::now() + ros::Duration(req.calibration_duration, 0);
            return true;
        }

        void callback(const sensor_msgs::Imu& imu_in) 
        {
            if(!calibration_)
            {
                ROS_INFO_STREAM(diameters_);
            }
            else
            {
                ROS_INFO_STREAM("Calibrating...");
                feed_calibration(imu_in);
            }
        }

        Vector computeInterval()
        {
            Vector diameters(6, 0.);

            double number;
            for(auto vect = data_.begin(); vect !=data_.end(); vect++)
            {
                for(unsigned int i = 0; i < 6; ++i)
                {
                    number = 2*std::abs(vect->operator[](i)-mean_[i]/nb_);
                    if(number - diameters[i] > 1./precision_) 
                        diameters[i] 
                            = roundf(number * precision_) / precision_;
                }
            }

            for(unsigned int i = 0; i < 6; ++i)
            {
                diameters[i] += 1./precision_;
                ROS_INFO_STREAM(mean_[i]/nb_);
            }

            ROS_INFO_STREAM(diameters);

            return diameters;
        }

        void feed_calibration(const sensor_msgs::Imu& imu_in)
        {
            if(ros::Time::now().toSec() < until_.toSec())
            {
                tmp_[0] = imu_in.angular_velocity.x;
                tmp_[1] = imu_in.angular_velocity.y;
                tmp_[2] = imu_in.angular_velocity.z;
                tmp_[3] = imu_in.linear_acceleration.x;
                tmp_[4] = imu_in.linear_acceleration.y;
                tmp_[5] = imu_in.linear_acceleration.z;

                mean_ += tmp_;
                data_.push_back(Vector(tmp_));
                nb_ += 1.;

                diameters_ = computeInterval(); 
            }
            else
            {
                calibration_ = false;
                diameters_ = computeInterval(); 
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
