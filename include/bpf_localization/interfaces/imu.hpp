
/**
 * \file   imu.hpp
 * \brief  IMU interfaces
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef IMU_HPP
#define IMU_HPP

#include <sensor_msgs/Imu.h>
#include "bpf_localization/IntervalIMU.h"

#include "bpf_localization/interfaces/sensors.hpp"

namespace Interfaces
{
    namespace Sensors
    {
        class IMU: virtual public SensorInterface
        {
            public:
                IMU(std::string path, std::string name, unsigned int decimal):
                    SensorInterface(path, name, size, decimal)
                {
                }

            public:
                static const unsigned int size = 6;

            protected:
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

        namespace ROS
        {
            class IMU: public Interfaces::Sensors::IMU, public ROSInterface
            {
                public:
                    IMU(ros::NodeHandle* nh, std::string path, std::string name, unsigned int decimal):
                        SensorInterface(path, name, IMU::size, decimal), 
                        Interfaces::Sensors::IMU(path, name, decimal), 
                        ROSInterface(nh, path, name, IMU::size, decimal)
                    {
                        sub_ = nh->subscribe(name + "_in", 50, &IMU::callback, this);
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
                        unsigned int time = (unsigned int)(ros::Time::now().toSec());
                        feed(tmp_, time);

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
            };
        }
    }
}

#endif
