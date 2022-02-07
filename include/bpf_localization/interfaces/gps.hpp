
/**
 * \file   gps.hpp
 * \brief  GPS interfaces
 * \author Alexis Mifsud
 * \date   2022 January
 */

#ifndef GPS_HPP
#define GPS_HPP

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <interval_msgs/Vector3IntervalStamped.h>

#include "bpf_localization/interfaces/interfaces.hpp"

namespace Interfaces
{
    namespace Sensors
    {
        class GPS: virtual public SensorInterface
        {
            public:
                GPS(std::string path, std::string name, unsigned int decimal):
                    SensorInterface(path, name, size, decimal),
                    initial_pose_(Vector(3,0.)), initialized_(false)
                {
                    half_diameters_ = 1e-1*Vector(size_, 1.);
                }

            protected:
                void calibrationDataFormat()
                {
                    calibration_data_format_.push_back("component");
                    calibration_data_format_.push_back("x");
                    calibration_data_format_.push_back("y");
                    calibration_data_format_.push_back("z");
                }

            public:
                static const unsigned int size = 3;
                Vector initial_pose_;
                bool initialized_;
        };

        namespace ROS
        {
            class GPS: public Interfaces::Sensors::GPS, public ROSInterface
            {
                public:
                    GPS(ros::NodeHandle* nh, std::string path, std::string name, unsigned int decimal):
                        SensorInterface(path, name, IMU::size, decimal), 
                        Interfaces::Sensors::GPS(path, name, decimal), 
                        ROSInterface(nh, path, name, IMU::size, decimal)
                    {
                        sub_ = nh->subscribe(name + "_in", 50, &GPS::callbackOdom, this);
                        pub_ = nh->advertise<interval_msgs::Vector3IntervalStamped>(name+"_out", 1000);
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
                        unsigned int time = (unsigned int)(ros::Time::now().toSec());
                        feed(tmp_, time);

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
            };
        }
    }
}

#endif
